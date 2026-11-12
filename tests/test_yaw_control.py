"""Run production yaw/pitch tasks and the DM4340 driver with a simulated HAL.

Usage: python tests/test_yaw_control.py (requires host g++).
"""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
STUB = r"""
#pragma once
#include <cstdint>
struct CAN_HandleTypeDef {};
struct TIM_HandleTypeDef {};
struct IWDG_HandleTypeDef {};
extern CAN_HandleTypeDef hcan1, hcan2;
extern TIM_HandleTypeDef htim6;
extern IWDG_HandleTypeDef hiwdg;
extern float euler_angles[3];
extern float gyro_data[3];
enum { HAL_OK = 0, CAN_IT_RX_FIFO0_MSG_PENDING = 1, CAN_IT_RX_FIFO1_MSG_PENDING = 2 };
uint32_t HAL_GetTick();
uint32_t __get_PRIMASK();
void __disable_irq();
void __set_PRIMASK(uint32_t);
void ImuInit();
void ImuUpdate();
void MX_IWDG_Init();
void CanFilter_Init(CAN_HandleTypeDef *);
int HAL_CAN_Start(CAN_HandleTypeDef *);
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, int);
int HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *);
void HAL_IWDG_Refresh(IWDG_HandleTypeDef *);
void Error_Handler();
void enable_motor_mode(CAN_HandleTypeDef *, uint16_t, uint16_t);
void mit_ctrl(CAN_HandleTypeDef *, uint16_t, float, float, float, float, float);
void CAN_Send_Msg(CAN_HandleTypeDef *, uint8_t *, uint32_t, uint8_t);
int float_to_uint(float, float, float, int);
float uint_to_float(int, float, float, int);
#include "dm4310_drv.hpp"
#include "dm4340_drv.hpp"
"""
HARNESS = r"""
#include "stub.hpp"
#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iostream>
extern "C" void MainInit();
extern "C" void MainTask();
void Decode_Chassis(const uint8_t *);
void Send_To_Chassis();
extern uint32_t tick;
extern volatile uint32_t yaw_feedback_count, yaw_feedback_last_tick;
extern float position, torque;
extern bool gimbal_ready;
extern DM4340_Motor_t pitch_motor_t;
extern volatile uint32_t pitch_feedback_count, pitch_feedback_last_tick;
extern bool pitch_control_enabled;
extern float pitch_motor_min, pitch_motor_max, pitch_motor_direction;
extern float pitch_position, pitch_omega, pitch_torque;
CAN_HandleTypeDef hcan1, hcan2;
TIM_HandleTypeDef htim6;
IWDG_HandleTypeDef hiwdg;
float euler_angles[3] = {0.6f, 0, 0};
float gyro_data[3] = {};
uint32_t now = 0;
float sent_torque = 0;
uint8_t feedback[8] = {};
uint8_t pitch_frame[8] = {};
int pitch_enables = 0;
int enables = 0, watchdog_refreshes = 0;
uint32_t HAL_GetTick() { return now; }
uint32_t __get_PRIMASK() { return 0; }
void __disable_irq() {}
void __set_PRIMASK(uint32_t) {}
void ImuInit() {}
void ImuUpdate() {}
void MX_IWDG_Init() {}
void CanFilter_Init(CAN_HandleTypeDef *) {}
int HAL_CAN_Start(CAN_HandleTypeDef *) { return HAL_OK; }
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, int) { return HAL_OK; }
int HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *) { return HAL_OK; }
void HAL_IWDG_Refresh(IWDG_HandleTypeDef *) { watchdog_refreshes++; }
void Error_Handler() { std::abort(); }
void joint_motor_init(Joint_Motor_t *, uint16_t id, uint16_t) { assert(id == 3); }
void enable_motor_mode(CAN_HandleTypeDef *, uint16_t id, uint16_t) {
  assert(id == 3); enables++;
}
void mit_ctrl(CAN_HandleTypeDef *, uint16_t id, float p, float v,
              float kp, float kd, float t) {
  assert(id == 3 && p == 0 && v == 0 && kp == 0 && kd == 0);
  assert(std::isfinite(t) && std::fabs(t) <= 4.0f);
  sent_torque = t;
}
void CAN_Send_Msg(CAN_HandleTypeDef *bus, uint8_t *data, uint32_t id, uint8_t len) {
  assert(len == 8);
  if (bus == &hcan2) {
    assert(id == 9);
    if (data[0] == 0xFF && data[7] == 0xFC) pitch_enables++;
    else {
      // MIT kp/kd must both remain zero for the external cascaded PID.
      assert((data[3] & 0x0F) == 0 && data[4] == 0 && data[5] == 0 &&
             (data[6] & 0xF0) == 0);
      std::memcpy(pitch_frame, data, 8);
    }
  } else {
    assert(bus == &hcan1 && id == 0x114);
    std::memcpy(feedback, data, 8);
  }
}
int float_to_uint(float x, float lo, float hi, int bits) {
  return static_cast<int>((x-lo) * ((1<<bits)-1) / (hi-lo));
}
float uint_to_float(int x, float lo, float hi, int bits) {
  return x * (hi-lo) / ((1<<bits)-1) + lo;
}
void command(bool enabled, float input = 0) {
  uint8_t data[8] = {};
  data[2] = enabled;
  data[3] = float_to_uint(0, -1, 1, 8);
  data[4] = float_to_uint(input, -1, 1, 8);
  Decode_Chassis(data);
}
void step(bool motor_feedback = true) {
  now++;
  if (motor_feedback) {
    yaw_feedback_last_tick = now;
    yaw_feedback_count++;
  }
  MainTask();
}
// Exercise pitch through MainTask, just like the production timer callback.
void pitch_step(bool enabled, float input) {
  uint8_t data[8] = {};
  data[2] = enabled;
  data[3] = float_to_uint(input, -1, 1, 8);
  data[4] = float_to_uint(0, -1, 1, 8);
  Decode_Chassis(data);
  yaw_feedback_last_tick = now;
  MainTask();
}
int main() {
  MainInit();
  assert(enables == 1);
  assert(pitch_enables == 1);
  for (int i = 0; i < 2000; ++i) {
    if (i % 10 == 0) command(true, 1);
    step();
    assert(sent_torque == 0 && !gimbal_ready);
  }
  command(true);
  step();
  assert(gimbal_ready && std::fabs(position - 0.6f) < 1e-6f);
  assert(std::fabs(sent_torque) < 1e-6f);
  for (int i = 0; i < 100; ++i) {
    if (i % 10 == 0) command(true, 1);
    step();
  }
  assert(position < 0.41f && sent_torque < 0);
  for (int i = 0; i < 101; ++i) step();
  assert(sent_torque == 0); // Commands stale even while motor feedback continues.
  euler_angles[0] = 1.2f;
  command(true);
  step();
  assert(std::fabs(position - 1.2f) < 1e-6f && sent_torque == 0);
  command(false);
  step();
  assert(sent_torque == 0);
  for (int i = 0; i < 101; ++i) {
    if (i % 10 == 0) command(true, 1);
    step(false);
  }
  assert(!gimbal_ready && sent_torque == 0); // Motor feedback timeout.
  now = UINT32_MAX - 20;
  for (int i = 0; i < 50; ++i) {
    if (i % 10 == 0) command(true);
    step();
    assert(gimbal_ready);
  }
  euler_angles[0] = -0.6f;
  Send_To_Chassis();
  int32_t raw = (feedback[0] << 16) | (feedback[1] << 8) | feedback[2];
  if (raw & 0x800000) raw -= 0x1000000;
  assert(std::abs(raw + 6000) <= 1 && feedback[4] == 1);
  assert(watchdog_refreshes > 0);
  std::cout << "yaw task: startup, control, disable, timeouts, recovery, wrap and telemetry passed\n";

  // Feedback packing and independent model ranges (yaw stays unchanged).
  uint8_t raw_pitch[8] = {0x19, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 45, 50};
  dm4340_fbdata(&pitch_motor_t, raw_pitch, 8);
  assert(pitch_motor_t.para.id == 9 && pitch_motor_t.para.state == 1);
  assert(pitch_motor_t.para.p_int == 65535);
  assert(std::fabs(pitch_motor_t.para.pos - DM4340_P_MAX) < 1e-6f);
  assert(pitch_motor_t.para.vel == 10 && pitch_motor_t.para.tor == 28);
  dm4340_fbdata(&pitch_motor_t, raw_pitch, 7);
  assert(pitch_motor_t.para.pos == DM4340_P_MAX);
  std::memset(raw_pitch, 0, 8);
  raw_pitch[0] = 0x19;
  dm4340_fbdata(&pitch_motor_t, raw_pitch, 8);
  assert(pitch_motor_t.para.pos == DM4340_P_MIN);
  assert(pitch_motor_t.para.vel == -10 && pitch_motor_t.para.tor == -28);

  // Unknown travel: calibration mode sends zero torque despite stick input.
  pitch_motor_t.para.pos = 0;
  pitch_feedback_count = 1;
  pitch_feedback_last_tick = now;
  euler_angles[1] = 0.2f;
  pitch_step(true, 1);
  assert(pitch_torque == 0);
  assert(((pitch_frame[6] & 15) << 8 | pitch_frame[7]) == 2047);
  pitch_control_enabled = true;
  pitch_step(true, 1);
  assert(pitch_torque == 0); // Invalid min == max.

  pitch_motor_min = -0.4f;
  pitch_motor_max = 0.6f;
  pitch_step(true, 0);
  assert(std::fabs(pitch_position - 0.2f) < 1e-6f && pitch_torque == 0);
  pitch_step(true, 1);
  assert(pitch_position > 0.2f && pitch_torque > 0);
  float held = pitch_position;
  pitch_step(true, 0);
  assert(pitch_position == held);
  // Changing earth attitude at fixed joint angle creates compensation torque.
  euler_angles[1] += 0.1f;
  pitch_step(true, 0);
  assert(pitch_torque < 0);
  gyro_data[1] = 0.2f;
  pitch_step(true, 0);
  assert(std::fabs(pitch_omega - 0.2f) < 1e-6f);
  gyro_data[1] = 0;

  // Upper bound: suppress outward torque, retain inward recovery.
  pitch_motor_t.para.pos = pitch_motor_max;
  pitch_position = euler_angles[1] + 1;
  for (int i = 0; i < 100; ++i) pitch_step(true, 1);
  assert(pitch_position <= euler_angles[1] + 1e-6f && pitch_torque == 0);
  pitch_step(true, -1);
  assert(pitch_torque < 0);
  // An external inward movement must not cause outward braking at the stop.
  gyro_data[1] = -0.2f;
  pitch_step(true, 0);
  assert(pitch_torque == 0);
  gyro_data[1] = 0;
  pitch_motor_t.para.pos = pitch_motor_max + 0.05f;
  pitch_step(true, 0);
  assert(pitch_torque < 0); // Out-of-range recovery.

  pitch_motor_t.para.pos = pitch_motor_min;
  pitch_position = euler_angles[1] - 1;
  pitch_step(true, -1);
  assert(pitch_torque == 0);
  pitch_step(true, 1);
  assert(pitch_torque > 0);
  // Reversed installation: clamp and torque sign both follow motor coordinates.
  pitch_motor_direction = -1;
  pitch_position = euler_angles[1];
  pitch_step(true, 1);
  assert(pitch_torque == 0);
  pitch_step(true, -1);
  assert(pitch_torque > 0);
  pitch_motor_t.para.pos = pitch_motor_max;
  pitch_position = euler_angles[1];
  pitch_step(true, -1);
  assert(pitch_torque == 0);
  pitch_step(true, 1);
  assert(pitch_torque < 0);

  pitch_step(false, 1);
  assert(pitch_torque == 0);
  now += 101;
  pitch_step(true, 1);
  assert(pitch_torque == 0); // Stale pitch feedback.
  pitch_feedback_last_tick = now;
  pitch_motor_t.para.pos = 0;
  euler_angles[1] = -0.1f;
  pitch_step(true, 0);
  assert(pitch_position == euler_angles[1] && pitch_torque == 0);

  // Full task wiring: byte 3 from the lower board drives pitch.
  uint8_t command_frame[8] = {0, 0, 1, 255, 127, 0, 0, 0};
  Decode_Chassis(command_frame);
  yaw_feedback_last_tick = now;
  MainTask();
  assert(pitch_torque < 0); // Reversed motor direction.
  std::cout << "pitch: DM4340 ranges/packing, calibration, hold, travel limits, reverse direction and recovery passed\n";
}
"""

with tempfile.TemporaryDirectory(prefix="gimbal-yaw-test-") as directory:
    temp = Path(directory)
    (temp / "stub.hpp").write_text(STUB, encoding="utf-8")
    for header in ["system_user.hpp", "imu_task.hpp", "HW_can.hpp"]:
        (temp / header).write_text('#include "stub.hpp"\n', encoding="utf-8")
    for source, target in [("Tasks/main_task.cpp", "main_task.cpp"),
                           ("Tasks/main_task.hpp", "main_task.hpp"),
                           ("Resources/Inc/dm4310_drv.hpp", "dm4310_drv.hpp"),
                           ("Resources/Inc/dm4340_drv.hpp", "dm4340_drv.hpp"),
                           ("Resources/Src/dm4340_drv.cpp", "dm4340_drv.cpp"),
                           ("Resources/Src/PID.cpp", "PID.cpp"),
                           ("Resources/Inc/PID.hpp", "PID.hpp")]:
        shutil.copyfile(ROOT / source, temp / target)
    (temp / "test.cpp").write_text(HARNESS, encoding="utf-8")
    executable = temp / "test.exe"
    subprocess.run(["g++", "-std=c++17", "-I", str(temp), str(temp / "main_task.cpp"),
                    str(temp / "PID.cpp"),
                    str(temp / "dm4340_drv.cpp"), str(temp / "test.cpp"), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
