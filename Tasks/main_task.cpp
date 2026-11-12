/**
*******************************************************************************
* @file      :main_task.cpp
* @brief     :
* @history   :
*  Version     Date            Author          Note
*  V0.9.0      yyyy-mm-dd      <author>        1. <note>
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2024 Hello World Team，Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/

#include "main_task.hpp"
#include "system_user.hpp"
#include "imu_task.hpp"
#include "HW_can.hpp"
#include "dm4310_drv.hpp"
#include "PID.hpp"
#include <cmath>

uint32_t tick = 0;
Joint_Motor_t yaw_motor_t;
PID_Controller yaw_pid;
PID_Controller torque_pid;
float position = 0.0f;       // IMU yaw target, rad
float torque = 0.0f;
float gimbal_omega = 0.0f;   // IMU yaw angular velocity, rad/s
float yaw_input = 0.0f;
float pitch_input = 0.0f;    // Reserved until pitch parameters are confirmed
bool gimbal_ready = false;
volatile uint32_t yaw_feedback_count = 0;
volatile uint32_t yaw_feedback_last_tick = 0;
volatile uint32_t chassis_command_count = 0;
volatile uint32_t chassis_command_last_tick = 0;
static bool chassis_enable = false;
static bool yaw_running = false;
extern float gyro_data[3];

void Gimbal_Yaw_Control();
void Send_To_Chassis();

// Called only for a validated 0x510 frame from HW_can.cpp.
void Decode_Chassis(const uint8_t *data) {
  chassis_enable = data[2] == 1;
  pitch_input = uint_to_float(data[3], -1.0f, 1.0f, 8);
  yaw_input = uint_to_float(data[4], -1.0f, 1.0f, 8);
  chassis_command_last_tick = HAL_GetTick();
  chassis_command_count++;
  HAL_IWDG_Refresh(&hiwdg);
}

void MainInit(void) {
  yaw_pid.init(5.0f, 0.0f, 0.0f, 7.5f, 0.0f);
  torque_pid.init(1.0f, 0.0f, 0.0f, 4.0f, 0.0f);
  ImuInit();
  MX_IWDG_Init();
  CanFilter_Init(&hcan1);
  if (HAL_CAN_Start(&hcan1) != HAL_OK ||
      HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }
  // Only the upper board enables/commands yaw. No pitch/feed/friction commands.
  joint_motor_init(&yaw_motor_t, 3, MIT_MODE);
  enable_motor_mode(&hcan1, 3, MIT_MODE);
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();
}

void MainTask(void) {
  tick++;
  ImuUpdate();
  // IMU is on the gimbal: use its Z gyro, not relative motor speed or a
  // derivative of the quantized board-to-board yaw angle.
  gimbal_omega = gyro_data[2];
  const uint32_t irq_state = __get_PRIMASK();
  __disable_irq();
  const uint32_t now = HAL_GetTick();
  gimbal_ready = tick > 2000 && yaw_feedback_count > 0 &&
                 static_cast<uint32_t>(now - yaw_feedback_last_tick) <= 100;
  const bool run = gimbal_ready && chassis_command_count > 0 &&
                   static_cast<uint32_t>(now - chassis_command_last_tick) <= 100 &&
                   chassis_enable;
  const float input = yaw_input;
  __set_PRIMASK(irq_state);

  if (run) {
    if (!yaw_running) {
      position = euler_angles[0];
      yaw_pid.reset();
      torque_pid.reset();
    }
    // Preserve the previous joystick direction, dead zone and 2 rad/s maximum.
    if (std::fabs(input) > 0.2f) {
      position = std::remainder(position - input * 0.002f, 2.0f * 3.14159265358979323846f);
    }
    Gimbal_Yaw_Control();
  } else {
    position = euler_angles[0];
    torque = 0.0f;
    yaw_pid.reset();
    torque_pid.reset();
    mit_ctrl(&hcan1, 3, 0, 0, 0, 0, 0);
  }
  yaw_running = run;
  if (tick % 10 == 0) Send_To_Chassis();
}

void Gimbal_Yaw_Control() {
  const float target_omega = yaw_pid.calculateRad(position, euler_angles[0], 0.001f);
  torque = torque_pid.calculate(target_omega, gimbal_omega, 0.001f);
  mit_ctrl(&hcan1, 3, 0, 0, 0, 0, torque);
}

void Send_To_Chassis() {
  // Preserve legacy 0x114 byte layout. Lower board now reads motor position
  // directly from 0x13; byte 3 remains for diagnostics/compatibility.
  const int32_t angle = static_cast<int32_t>(euler_angles[0] * 10000.0f);
  const uint32_t raw = static_cast<uint32_t>(angle);
  uint8_t data[8] = {};
  data[0] = static_cast<uint8_t>(raw >> 16);
  data[1] = static_cast<uint8_t>(raw >> 8);
  data[2] = static_cast<uint8_t>(raw);
  data[3] = float_to_uint(yaw_motor_t.para.pos, -3.14f, 3.14f, 8);
  data[4] = gimbal_ready ? 1 : 0;
  CAN_Send_Msg(&hcan1, data, 0x114, 8);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) MainTask();
}
