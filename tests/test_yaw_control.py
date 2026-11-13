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
#include "PID.hpp"
#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iostream>
// Fail on stderr instead of opening a Windows CRT assertion dialog.
#undef assert
#define assert(condition) do { if (!(condition)) { \
  std::cerr << "Assertion failed at line " << __LINE__ << ": " << #condition << '\n'; \
  std::exit(1); \
} } while (false)
extern "C" void MainInit();
extern "C" void MainTask();
void Decode_Chassis(const uint8_t *);
void Send_To_Chassis();
extern uint32_t tick;
extern volatile uint32_t yaw_feedback_count, yaw_feedback_last_tick;
extern float position, torque, gimbal_omega, yaw_target_omega;
extern Joint_Motor_t yaw_motor_t;
extern bool gimbal_ready;
extern DM4340_Motor_t pitch_motor_t;
extern volatile uint32_t pitch_feedback_count, pitch_feedback_last_tick;
extern bool pitch_control_enabled;
extern float pitch_motor_min, pitch_motor_max, pitch_motor_direction;
extern float pitch_position, pitch_omega, pitch_torque;
extern float pitch_target_omega, pitch_pid_torque;
extern float pitch_gravity_gain, pitch_gravity_angle, pitch_gravity_torque;
extern PID_Controller pitch_torque_pid;
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
  assert(std::isfinite(t) && t >= T_MIN && t <= T_MAX);
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
  assert(pitch_gravity_gain == 7.0f);
  // Isolate feedback-only regressions before testing the configured feedforward.
  pitch_gravity_gain = 0;
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

  // The IMU is horizontal on the yaw bracket: use gyro Z directly.
  command(false);
  step();
  euler_angles[0] = 0.6f;
  euler_angles[1] = euler_angles[2] = 0;
  command(true);
  step();
  const float held_yaw = position;
  gyro_data[1] = 3.0f; // Other gyro axes are not yaw feedback.
  gyro_data[2] = 2.0f;
  step();
  assert(gimbal_omega == 2.0f && sent_torque < 0);
  assert(position == held_yaw && yaw_target_omega == 0);
  gyro_data[2] = 0;
  step();
  assert(gimbal_omega == 0 && position == held_yaw);
  std::memset(gyro_data, 0, sizeof(gyro_data));

  // The old +/-4 Nm application cap is removed. Large errors can use the
  // full configured protocol torque range, in both directions.
  command(false);
  step();
  command(true);
  step();
  euler_angles[0] = position - 1.0f;
  step();
  assert(sent_torque == T_MAX && sent_torque > 4.0f);
  euler_angles[0] = position + 1.0f;
  step();
  assert(sent_torque == T_MIN && sent_torque < -4.0f);

  // Cross +/-pi using the short angular error; the target must stay fixed.
  command(false);
  step();
  euler_angles[0] = 3.14159265359f - 0.02f;
  command(true);
  step();
  euler_angles[0] = -3.14159265359f + 0.02f;
  step();
  assert(yaw_target_omega < -0.31f && yaw_target_omega > -0.33f);

  // A held heading must oppose both displacement and continued pushing.
  // At a small 0.05 rad offset the two gains give about 1.6 Nm,
  // 3.2 times the archived proportional response, without moving the target.
  command(false);
  step();
  euler_angles[0] = 0.6f;
  gyro_data[2] = 0;
  command(true);
  step();
  const float yaw_hold_target = position;
  euler_angles[0] += 0.05f;
  step();
  assert(position == yaw_hold_target);
  assert(sent_torque < -1.59f && sent_torque > -1.61f);
  gyro_data[2] = 0.1f;
  step();
  assert(position == yaw_hold_target && sent_torque < -1.99f);

  // Illustrative joint dynamics, not a model of the real vehicle. Verify that
  // a displaced heading recovers despite a constant 0.2 Nm load, both with a
  // stationary chassis and a spinning chassis (relative motor angle changes).
  for (float chassis_rate : {0.0f, 4.712389f}) {
    command(false);
    step();
    euler_angles[0] = 0.6f;
    gyro_data[2] = 0;
    command(true);
    step();
    assert(sent_torque == 0); // Integral cleared on disable/recovery.
    euler_angles[0] += 0.1f; // External heading displacement.
    float speed = 0;
    for (int i = 0; i < 20000; ++i) {
      if (i % 10 == 0) command(true);
      yaw_motor_t.para.pos = std::remainder(
          euler_angles[0] - chassis_rate * i * 0.001f, 6.28318530718f);
      step();
      assert(std::fabs(position - 0.6f) < 1e-6f);
      speed += (sent_torque + 0.2f - 0.05f * speed) / 0.15f * 0.001f;
      euler_angles[0] += speed * 0.001f;
      gyro_data[2] = speed;
    }
    assert(std::fabs(euler_angles[0] - position) < 0.002f);
    assert(std::fabs(sent_torque + 0.2f) < 0.01f);
  }
  // A blocked joint cannot accumulate more than 0.5 Nm integral torque.
  euler_angles[0] = position - 0.2f;
  gyro_data[2] = 0;
  for (int i = 0; i < 3000; ++i) {
    if (i % 10 == 0) command(true);
    step();
  }
  assert(std::fabs(sent_torque - 4.0f * yaw_target_omega) <= 0.50001f);
  command(false);
  step();
  command(true);
  step();
  assert(sent_torque == 0);
  std::cout << "yaw hold: horizontal IMU, protocol torque range, wrap, disturbance recovery, rotating chassis and bounded/reset integral passed\n";

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

  // The IMU stays on the yaw bracket: pitch must use DM4340 pos/vel only.
  const float motor_middle = (pitch_motor_min + pitch_motor_max) * 0.5f;
  pitch_motor_t.para.pos = motor_middle;
  pitch_motor_t.para.vel = 0;
  pitch_feedback_count = 1;
  pitch_feedback_last_tick = now;
  euler_angles[1] = -0.036f;
  pitch_step(true, 0);
  assert(pitch_position == motor_middle && pitch_torque == 0);
  pitch_step(true, 1);
  assert(pitch_position < motor_middle && pitch_torque < 0); // Up = negative.
  const float held = pitch_position;
  pitch_step(true, 0);
  assert(pitch_position == held);
  pitch_motor_t.para.pos = held + 0.03f;
  pitch_step(true, 0);
  assert(pitch_torque < 0); // Displaced downward: lift back to target.
  pitch_motor_t.para.pos = held - 0.03f;
  pitch_step(true, 0);
  assert(pitch_torque > 0); // Overshot upward: correct downward.
  pitch_motor_t.para.pos = held;
  pitch_motor_t.para.vel = -0.2f;
  pitch_step(true, 0);
  assert(pitch_omega == -0.2f && pitch_torque > 0); // Brake upward motion.
  pitch_motor_t.para.vel = 0.2f;
  pitch_step(true, 0);
  assert(pitch_torque < 0); // Brake downward motion.
  pitch_motor_t.para.vel = 0;
  pitch_torque_pid.reset(); // Isolate IMU independence from prior corrections.
  euler_angles[1] += 0.1f;
  gyro_data[1] = 1.0f;
  pitch_step(true, 0);
  assert(pitch_position == held && pitch_omega == 0 && pitch_torque == 0);
  gyro_data[1] = 0;

  // Full upward stick moves the pitch target at 0.8 rad/s, independently
  // of the faster outer-loop catch-up limit. Return to center holds it.
  pitch_step(false, 0);
  pitch_motor_t.para.pos = motor_middle;
  pitch_step(true, 0);
  for (int i = 0; i < 200; ++i) pitch_step(true, 1);
  assert(std::fabs(pitch_position - (motor_middle - 0.16f)) < 2e-5f);
  const float stick_target = pitch_position;
  pitch_step(true, 0);
  assert(pitch_position == stick_target);
  pitch_step(false, 0);
  pitch_step(true, 0);

  // The diagnostic P-only speed loop must not accumulate integral correction.
  pitch_position = motor_middle - 0.1f;
  for (int i = 0; i < 10000; ++i) {
    pitch_step(true, 0);
    assert(std::fabs(pitch_torque + 9.6f) < 1e-4f);
    assert(pitch_torque < -9.0f); // No former 9 Nm application cap.
    assert(pitch_torque_pid.getIntegral() == 0);
  }
  assert(pitch_torque_pid.getIntegral() == 0);
  assert(std::fabs(pitch_torque + 9.6f) < 1e-4f);
  assert(std::fabs(pitch_target_omega + 0.8f) < 1e-6f);
  const int lift_raw = ((pitch_frame[6] & 15) << 8) | pitch_frame[7];
  assert(std::fabs(uint_to_float(lift_raw, -28, 28, 12) - pitch_torque) < 0.014f);
  pitch_position = motor_middle - 0.3f;
  pitch_step(true, 0);
  assert(std::fabs(pitch_target_omega + 1.2f) < 1e-6f);
  assert(std::fabs(pitch_torque + 14.4f) < 1e-4f); // Catch-up speed is limited.
  pitch_motor_t.para.vel = 5;
  pitch_step(true, 0);
  assert(pitch_torque == -DM4340_T_MAX);
  pitch_motor_t.para.vel = 0;

  // Illustrative motor dynamics, matched gravity, fixed bracket IMU.
  pitch_step(false, 0);
  pitch_motor_t.para.pos = motor_middle;
  pitch_gravity_gain = 7.0f;
  pitch_gravity_angle = motor_middle;
  euler_angles[1] = -0.036f;
  pitch_step(true, 0);
  assert(std::fabs(pitch_torque + 7.0f) < 1e-6f);
  const float target = motor_middle - 0.05f;
  pitch_position = target;
  for (int i = 0; i < 30000; ++i) {
    now++;
    pitch_feedback_last_tick = now;
    pitch_step(true, 0);
    assert(std::fabs(pitch_torque) <= DM4340_T_MAX);
    assert(pitch_position == target);
    assert(std::fabs(pitch_target_omega) <= 1.2f);
    const float load = 7.0f * std::cos(pitch_motor_t.para.pos - motor_middle);
    pitch_motor_t.para.vel += (pitch_torque + load -
        0.2f * pitch_motor_t.para.vel) / 0.2f * 0.001f;
    pitch_motor_t.para.pos += pitch_motor_t.para.vel * 0.001f;
    assert(pitch_motor_t.para.pos > pitch_motor_min &&
           pitch_motor_t.para.pos < pitch_motor_max);
    if (i == 499) assert(std::fabs(pitch_motor_t.para.pos - target) < 0.002f);
  }
  assert(std::fabs(pitch_motor_t.para.pos - target) < 0.002f);
  assert(euler_angles[1] == -0.036f); // No gun-mounted IMU required.

  // Push the gun downward at a fixed target, then release it. The controller
  // must restore the held position, with an immediate proportional response
  // and the same production gravity feedforward / protocol torque range.
  pitch_motor_t.para.pos += 0.03f;
  pitch_motor_t.para.vel = 0;
  pitch_step(true, 0);
  assert(pitch_target_omega < -0.23f && pitch_pid_torque < -2.8f);
  for (int i = 0; i < 5000; ++i) {
    now++;
    pitch_feedback_last_tick = now;
    pitch_step(true, 0);
    assert(pitch_position == target);
    assert(std::fabs(pitch_target_omega) <= 1.2f);
    assert(std::fabs(pitch_torque) <= DM4340_T_MAX);
    const float load = 7.0f * std::cos(pitch_motor_t.para.pos - motor_middle);
    pitch_motor_t.para.vel += (pitch_torque + load -
        0.2f * pitch_motor_t.para.vel) / 0.2f * 0.001f;
    pitch_motor_t.para.pos += pitch_motor_t.para.vel * 0.001f;
    assert(pitch_motor_t.para.pos > pitch_motor_min &&
           pitch_motor_t.para.pos < pitch_motor_max);
  }
  assert(std::fabs(pitch_motor_t.para.pos - target) < 0.002f);
  pitch_step(false, 0);
  pitch_motor_t.para.vel = 0;
  pitch_step(true, 0);
  assert(pitch_position == pitch_motor_t.para.pos);
  assert(std::fabs(pitch_pid_torque) < 1e-6f);
  assert(std::fabs(pitch_torque - pitch_gravity_torque) < 1e-6f);

  // With integral disabled, a 1 Nm gravity/load mismatch leaves a predictable
  // steady position error. Verify force balance and zero integral state.
  // Keep the production 7 Nm feedforward and fixed bracket IMU; simulate
  // an 8 Nm load, exercising both lifting and lowering with the same gains.
  for (float direction : {-1.0f, 1.0f}) {
    pitch_step(false, 0);
    pitch_motor_t.para.pos = motor_middle;
    pitch_motor_t.para.vel = 0;
    pitch_step(true, 0);
    const float held_target = motor_middle + direction * 0.05f;
    pitch_position = held_target;
    for (int i = 0; i < 60000; ++i) {
      now++;
      pitch_feedback_last_tick = now;
      pitch_step(true, 0);
      assert(pitch_position == held_target);
      assert(std::fabs(pitch_target_omega) <= 1.200001f);
      assert(std::fabs(pitch_torque) <= DM4340_T_MAX);
      assert(pitch_torque_pid.getIntegral() == 0);
      const float load = 8.0f * std::cos(pitch_motor_t.para.pos - motor_middle);
      pitch_motor_t.para.vel += (pitch_torque + load -
          0.2f * pitch_motor_t.para.vel) / 0.2f * 0.001f;
      pitch_motor_t.para.pos += pitch_motor_t.para.vel * 0.001f;
      assert(pitch_motor_t.para.pos > pitch_motor_min &&
             pitch_motor_t.para.pos < pitch_motor_max);
    }
    const float expected_error = std::cos(pitch_motor_t.para.pos - motor_middle) / 96.0f;
    assert(std::fabs((pitch_motor_t.para.pos - held_target) - expected_error) < 0.0002f);
    assert(std::fabs(pitch_motor_t.para.vel) < 0.001f);
    assert(euler_angles[1] == -0.036f);
    assert(pitch_torque_pid.getIntegral() == 0);
    pitch_step(false, 0);
    assert(pitch_torque == 0 && pitch_torque_pid.getIntegral() == 0);
    pitch_motor_t.para.vel = 0;
    pitch_step(true, 0);
    assert(pitch_position == pitch_motor_t.para.pos);
    assert(pitch_pid_torque == 0 && pitch_torque_pid.getIntegral() == 0);
  }

  // Gravity phase is a motor position; bracket IMU pitch cannot change it.
  pitch_step(false, 0);
  pitch_motor_t.para.pos = 0;
  pitch_gravity_angle = 0;
  pitch_step(true, 0);
  assert(std::fabs(pitch_gravity_torque + 7.0f) < 1e-6f);
  euler_angles[1] = 0.2f;
  pitch_step(true, 0);
  assert(std::fabs(pitch_torque + 7.0f) < 1e-6f);
  pitch_motor_t.para.pos = pitch_position = -0.2f;
  pitch_step(true, 0);
  assert(std::fabs(pitch_gravity_torque + 7.0f * std::cos(0.2f)) < 1e-6f);
  pitch_gravity_gain = 2 * DM4340_T_MAX;
  pitch_step(true, 0);
  assert(pitch_torque == -DM4340_T_MAX);
  pitch_gravity_gain = 7;
  pitch_motor_t.para.pos = pitch_motor_min;
  pitch_position = pitch_motor_min;
  pitch_step(true, 1);
  assert(pitch_torque == 0); // Feedforward must obey the raising limit.
  pitch_motor_t.para.pos = pitch_motor_max;
  pitch_step(true, 0);
  assert(pitch_torque < 0); // Inward feedforward remains permitted.
  pitch_step(false, 0);
  assert(pitch_torque == 0 && pitch_gravity_torque == 0);
  assert(pitch_target_omega == 0 && pitch_pid_torque == 0);
  now += 101;
  pitch_step(true, 0);
  assert(pitch_torque == 0 && pitch_gravity_torque == 0);
  pitch_gravity_gain = 0;
  pitch_feedback_last_tick = now;
  pitch_motor_t.para.pos = motor_middle;
  pitch_step(true, 0);
  assert(pitch_position == motor_middle && pitch_torque == 0);

  // Motor-coordinate limits clamp the target and allow inward recovery.
  pitch_motor_t.para.pos = pitch_motor_min;
  pitch_position = pitch_motor_min - 1;
  pitch_step(true, 1);
  assert(pitch_position == pitch_motor_min && pitch_torque == 0);
  pitch_step(true, -1);
  assert(pitch_torque > 0);
  pitch_motor_t.para.pos = pitch_motor_max;
  pitch_position = pitch_motor_max + 1;
  pitch_step(true, -1);
  assert(pitch_position == pitch_motor_max && pitch_torque == 0);
  pitch_step(true, 1);
  assert(pitch_torque < 0);
  pitch_motor_t.para.vel = -0.2f;
  pitch_step(true, 0);
  assert(pitch_torque == 0); // Outward braking is still forbidden at a stop.
  pitch_motor_t.para.vel = 0;
  pitch_motor_t.para.pos = pitch_motor_max + 0.05f;
  pitch_step(true, 0);
  assert(pitch_torque < 0); // Outside travel: recover inward.

  // Calibration, invalid feedback and recovery recapture motor position.
  pitch_control_enabled = false;
  pitch_step(true, 1);
  assert(pitch_torque == 0);
  pitch_control_enabled = true;
  pitch_motor_min = pitch_motor_max = 0;
  pitch_step(true, 1);
  assert(pitch_torque == 0);
  pitch_motor_min = -0.4f;
  pitch_motor_max = 0.6f;
  pitch_motor_t.para.pos = 0;
  pitch_motor_t.para.vel = NAN;
  pitch_step(true, 1);
  assert(pitch_torque == 0);
  pitch_motor_t.para.vel = 0;
  pitch_step(true, 0);
  assert(pitch_position == 0 && pitch_torque == 0);
  pitch_motor_direction = 1;
  pitch_step(true, 1);
  assert(pitch_position > 0 && pitch_torque > 0);
  pitch_step(false, 0);
  pitch_motor_direction = -1;
  pitch_step(true, 0);
  assert(pitch_position == 0 && pitch_torque == 0);

  // Full task wiring: byte 3 from the lower board raises the gun.
  uint8_t command_frame[8] = {0, 0, 1, 255, 127, 0, 0, 0};
  Decode_Chassis(command_frame);
  yaw_feedback_last_tick = now;
  MainTask();
  assert(pitch_torque < 0);
  std::cout << "pitch: DM4340 feedback/packing, cascaded P/P, disabled integral, load mismatch force balance, fixed IMU, gravity and travel limits passed\n";

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
