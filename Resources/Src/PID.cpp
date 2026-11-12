/**
 * @file      : PID.cpp
 * @brief     : PID控制器类
 */
#include "PID.hpp"
#include "stdint.h"
#include <algorithm>

float PID_Controller::calculate(float target, float actual, float dt) {
    if (dt <= 0.0f) {
        return 0.0f;
    }
    
    float error = target - actual;

    float p_term = kp_ * error;
    
    // 积分项（带抗饱和）
    integral_ += error * dt;
    
    // 积分限幅
    if (integral_ > integral_limit_) {
        integral_ = integral_limit_;
    } else if (integral_ < -integral_limit_) {
        integral_ = -integral_limit_;
    }
    
    float i_term = ki_ * integral_;
    
    // 微分项
    float derivative = (error - prev_error_) / dt;
    float d_term = kd_ * derivative;
    prev_error_ = error;
    
    // 计算总输出
    float output = p_term + i_term + d_term;
    
    // 输出限幅
    if (output > output_limit_) {
        output = output_limit_;
    } else if (output < -output_limit_) {
        output = -output_limit_;
    }
    
    return output;
}

float PID_Controller::calculateRad(float target, float actual, float dt) {
    if (dt <= 0.0f) {
        return 0.0f;
    }
    
    float error = target - actual;

    // 处理角度缠绕：将目标角度限制在 [-π, π]
    if (error > 3.14159f) {
        error -= 2 * 3.14159f;
    } else if (error < -3.14159f) {
        error += 2 * 3.14159f;
    }

    float p_term = kp_ * error;
    
    // 积分项（带抗饱和）
    integral_ += error * dt;
    
    // 积分限幅
    if (integral_ > integral_limit_) {
        integral_ = integral_limit_;
    } else if (integral_ < -integral_limit_) {
        integral_ = -integral_limit_;
    }
    
    float i_term = ki_ * integral_;
    
    // 微分项
    float derivative = (error - prev_error_) / dt;
    float d_term = kd_ * derivative;
    prev_error_ = error;
    
    // 计算总输出
    float output = p_term + i_term + d_term;
    
    // 输出限幅
    if (output > output_limit_) {
        output = output_limit_;
    } else if (output < -output_limit_) {
        output = -output_limit_;
    }
    
    return output;
}

void PID_Controller::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
}

void PID_Controller::setParameters(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID_Controller::setLimits(float output_limit, float integral_limit) {
    output_limit_ = output_limit;
    integral_limit_ = integral_limit;
}

void PID_Controller::init(float kp, float ki, float kd, 
                         float output_limit, float integral_limit) {
    setParameters(kp, ki, kd);
    setLimits(output_limit, integral_limit);
    reset();
}