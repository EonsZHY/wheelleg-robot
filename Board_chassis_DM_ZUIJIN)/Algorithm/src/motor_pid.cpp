/**
 * @file motor_pid.cpp
 * @author centre
 * @brief PID控制算法实现模块
 * @version 0.2
 * @date 2025-08-21
 * @copyright Copyright (c) 2021
 */
#include "motor_pid.h"

/**
 * @brief 绝对值限制函数
 * @param a 指向要限制的浮点数的指针
 * @param ABS_MAX 绝对值的最大限制
 * @note 此函数会直接修改传入的指针值
 */
static void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}
/**
 * @brief 设置目标值
 * @param 
 */
void Class_MotorPid::SetTarget(float _target)
{
    Target = _target;
}
/**
 * @brief 设置真实值
 * @param 
 */
void Class_MotorPid::SetMeasured(float _measured)
{
    Measured = _measured;
}
/**
 * @brief Yaw轴模糊位置式PID控制器
 * @param pid_t PID控制器结构体指针
 * @param fuzzy_t 模糊PID数据结构体指针
 * @param target 目标值
 * @param measured 测量值
 * @return PID控制输出值
 * @note 此函数结合模糊控制优化PID参数，适用于Yaw轴控制
 */
void Class_MotorPid::Position_PID_Yaw(FUZZYPID_Data_t *fuzzy_t)
{
    // 模糊计算更新PID参数
    FuzzyComputation(fuzzy_t, err, err_last);
    
    // 更新状态变量
    err = Target - Measured;
    err_change = err - err_last;
    error_target = Target - last_set_point;

    // 计算各项输出（使用模糊调整后的参数）
    p_out = (Kp + fuzzy_t->deta_kp) * err;
    i_out += (Ki + fuzzy_t->date_ki) * err;
    d_out = (Kd + fuzzy_t->date_kd) * (Measured - err_last);
    f_out = Kf * error_target;
    
    // 积分分离策略
    if (err >= Integral_Separation) {
        i_out = 0;
    } else {
        // 积分限幅
        abs_limit(&i_out,IntegralLimit);
    }

    // 计算总输出
    pwm = (p_out + i_out + d_out + f_out);

    // 输出限幅
    abs_limit(&pwm, MaxOutput);

    // 更新历史状态
    err_last = Measured;
    last_set_point = Target;
}

/**
 * @brief 增量式PID控制器
 * @note 适用于需要平滑控制的场景
 */
void Class_MotorPid::Incremental_PID()
{
    // 更新状态变量
    err = Target - Measured;

    // 计算各项输出
    p_out = Kp * (err - err_last);
    i_out = Ki * err;
    d_out = Kd * (err - 2.0f * err_last + err_beforeLast);

    // 积分限幅
    abs_limit(&i_out, IntegralLimit);

    // 计算总输出
    pwm += (p_out + i_out + d_out);

    // 输出限幅
    abs_limit(&pwm, MaxOutput);

    // 更新历史状态
    err_beforeLast = err_last;
    err_last = err;
}

/**
 * @brief 初始化增量式PID控制器
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 * @param MaxOutput 最大输出限制
 * @param IntegralLimit 积分项限幅值
 */
void Class_MotorPid::Incremental_PIDInit(float _Kp, float _Ki, float _Kd, float _ecd_max, uint32_t _MaxOutput, uint32_t _IntegralLimit)
{
    // 设置PID参数
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;

    ecd_max = _ecd_max;
    
    // 设置限制参数
    MaxOutput = _MaxOutput;
    IntegralLimit = _IntegralLimit;
    
    // 初始化状态变量
    p_out = 0;
    d_out = 0;
    i_out = 0;
    err = 0;
    err_last = 0;
    err_beforeLast = 0;
    pwm = 0;
    Measured = 0;
    Target = 0;
}
/**
 * @brief 初始化位置式PID控制器
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 * @param Kf 前馈系数
 * @param MaxOutput 最大输出限制
 * @param Integral_Separation 积分分离阈值
 * @param IntegralLimit 积分项限幅值
 */
void Class_MotorPid::Position_PIDInit(float _Kp, float _Ki, float _Kd, float _Kf, float _ecd_max, float _MaxOutput, float _Integral_Separation, float _IntegralLimit)
{
    // 设置PID参数
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
    Kf = _Kf;
    
    ecd_max = _ecd_max;
    // 设置限制参数
    MaxOutput = _MaxOutput;
    Integral_Separation = _Integral_Separation;
    IntegralLimit = _IntegralLimit;
    
    // 初始化状态变量
    p_out = 0;
    d_out = 0;
    i_out = 0;
    err = 0;
    err_last = 0;
    err_change = 0;
    error_target = 0;
    pwm = 0;
    Measured = 0;
    Target = 0;
}

/**
 * @brief 位置式PID控制器
 * @note 标准位置式PID实现，带前馈控制
 */
void Class_MotorPid::Position_PID()
{
    // 更新状态变量
    err = Target - Measured;
    err_change = Measured - err_last;
    error_target = Target - last_set_point;
    
    // 计算各项输出
    p_out = Kp * err;
    i_out += Ki * err;
    d_out = Kd * (err - err_last);
    f_out = Kf * error_target;
    
    // 积分限幅
    abs_limit(&i_out, IntegralLimit);

    // 计算总输出
    pwm = (p_out + i_out + d_out + f_out);

    // 输出限幅
    abs_limit(&pwm, MaxOutput);

    // 更新历史状态
    err_last = err;
    last_set_point = Target;
}
/**
 * @brief 清除位置式PID控制器数据
 * @note 重置所有状态变量，保留参数设置
 */
void Class_MotorPid::Clear_PositionPIDData()
{
    Target = 0;
    Measured = 0;
    err = 0;
    err_change = 0;
    err_last = 0;
    p_out = 0;
    i_out = 0;
    d_out = 0;
    pwm = 0;
}
/**
 * @brief 清除增量式PID控制器数据
 * @note 重置所有状态变量，保留参数设置
 */
void Class_MotorPid::Clear_IncrementalPIDData()
{
    Target = 0;
    Measured = 0;
    err = 0;
    err_last = 0;
    err_beforeLast = 0;
    p_out = 0;
    i_out = 0;
    d_out = 0;
    pwm = 0;
}
/**
 * @brief 角度位置式PID控制器
 * @note 专门处理电机角度值的PID控制器，自动处理角度环绕问题，编码值需要从0开始
 */
void Class_MotorPid::Angle_PID()
{
    // 更新状态变量
    err = Target - Measured;
    
    // 处理角度环绕问题（8192对应360度）
    if(fabs(err) > ecd_max/2) {
        if(err > 0) {
            err = err - (ecd_max+1);
        } else {
            err = err + (ecd_max+1);
        }
    }
    
    err_change = Measured - err_last;
    error_target = Target - last_set_point;
    
    // 计算各项输出
    p_out = Kp * err;
    i_out += Ki * err;
    d_out = Kd * (err - err_last);
    f_out = Kf * error_target;
    
    // 积分限幅
    abs_limit(&i_out, IntegralLimit);

    // 计算总输出
    pwm = (p_out + i_out + d_out + f_out);

    // 输出限幅
    abs_limit(&pwm, MaxOutput);

    // 更新历史状态
    err_last = err;
    last_set_point = Target;
}