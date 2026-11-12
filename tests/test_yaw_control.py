"""Run the production yaw task with a simulated HAL and the real PID code.

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
struct MotorPara { float pos = 0; };
struct Joint_Motor_t { MotorPara para; };
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim6;
extern IWDG_HandleTypeDef hiwdg;
extern float euler_angles[3];
extern float gyro_data[3];
enum { HAL_OK = 0, CAN_IT_RX_FIFO0_MSG_PENDING = 1, MIT_MODE = 0 };
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
void joint_motor_init(Joint_Motor_t *, uint16_t, uint16_t);
void enable_motor_mode(CAN_HandleTypeDef *, uint16_t, uint16_t);
void mit_ctrl(CAN_HandleTypeDef *, uint16_t, float, float, float, float, float);
void CAN_Send_Msg(CAN_HandleTypeDef *, uint8_t *, uint32_t, uint8_t);
int float_to_uint(float, float, float, int);
float uint_to_float(int, float, float, int);
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
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim6;
IWDG_HandleTypeDef hiwdg;
float euler_angles[3] = {0.6f, 0, 0};
float gyro_data[3] = {};
uint32_t now = 0;
float sent_torque = 0;
uint8_t feedback[8] = {};
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
void CAN_Send_Msg(CAN_HandleTypeDef *, uint8_t *data, uint32_t id, uint8_t len) {
  assert(id == 0x114 && len == 8);
  std::memcpy(feedback, data, 8);
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
int main() {
  MainInit();
  assert(enables == 1);
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
}
"""

with tempfile.TemporaryDirectory(prefix="gimbal-yaw-test-") as directory:
    temp = Path(directory)
    (temp / "stub.hpp").write_text(STUB, encoding="utf-8")
    for header in ["system_user.hpp", "imu_task.hpp", "HW_can.hpp", "dm4310_drv.hpp"]:
        (temp / header).write_text('#include "stub.hpp"\n', encoding="utf-8")
    for source, target in [("Tasks/main_task.cpp", "main_task.cpp"),
                           ("Tasks/main_task.hpp", "main_task.hpp"),
                           ("Resources/Src/PID.cpp", "PID.cpp"),
                           ("Resources/Inc/PID.hpp", "PID.hpp")]:
        shutil.copyfile(ROOT / source, temp / target)
    (temp / "test.cpp").write_text(HARNESS, encoding="utf-8")
    executable = temp / "test.exe"
    subprocess.run(["g++", "-std=c++17", "-I", str(temp), str(temp / "main_task.cpp"),
                    str(temp / "PID.cpp"), str(temp / "test.cpp"), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
