/**
 * @file      : PID.hpp
 * @brief     : PID控制器类
 */
#ifndef HW_COMPONENTS_ALGORITHMS_PID_HPP_
#define HW_COMPONENTS_ALGORITHMS_PID_HPP_

class PID_Controller {
public:
    /**
     * @brief PID控制器计算
     * @param target 目标值
     * @param actual 当前值
     * @param dt 时间间隔(秒)
     * @return PID输出值
     */
    float calculate(float target, float actual, float dt);

    /**
     * @brief PID控制器计算(角度环跳变处理)
     * @param target 目标值
     * @param actual 当前值
     * @param dt 时间间隔(秒)
     * @return PID输出值
     */
    float calculateRad(float target, float actual, float dt);

    /**
     * @brief 重置PID控制器
     */
    void reset();

    /**
     * @brief 设置PID参数
     * @param kp 比例系数
     * @param ki 积分系数
     * @param kd 微分系数
     */
    void setParameters(float kp, float ki, float kd);

    /**
     * @brief 设置限幅参数
     * @param output_limit 输出限幅
     * @param integral_limit 积分限幅
     */
    void setLimits(float output_limit, float integral_limit);

    /**
     * @brief 初始化PID控制器
     * @param kp 比例系数
     * @param ki 积分系数
     * @param kd 微分系数
     * @param output_limit 输出限幅
     * @param integral_limit 积分限幅
     */
    void init(float kp, float ki, float kd, 
              float output_limit, float integral_limit);

    /**
     * @brief 获取比例系数
     */
    float getKp() const { return kp_; }

    /**
     * @brief 获取积分系数
     */
    float getKi() const { return ki_; }

    /**
     * @brief 获取微分系数
     */
    float getKd() const { return kd_; }

    /**
     * @brief 获取积分项
     */
    float getIntegral() const { return integral_; }

    /**
     * @brief 获取上一次误差
     */
    float getPrevError() const { return prev_error_; }

private:
    float kp_ = 0.0f;              ///< 比例系数
    float ki_ = 0.0f;              ///< 积分系数  
    float kd_ = 0.0f;              ///< 微分系数
    float integral_ = 0.0f;        ///< 积分项
    float prev_error_ = 0.0f;      ///< 上一次误差
    float output_limit_ = 0.0f;    ///< 输出限幅
    float integral_limit_ = 0.0f;  ///< 积分限幅
};

#endif /* HW_COMPONENTS_ALGORITHMS_PID_HPP_ */