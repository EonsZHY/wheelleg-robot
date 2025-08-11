/**
 *******************************************************************************
 * @file      : vmc.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "vmc.h"
#include "arm_math.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// const float k_thigh_len = 0.15f;//并联腿上半部分长度
// const float k_calf_len = 0.27f;//并联腿下半部分长度
// const float k_joint_len = 0.15f;//髋关节两电机间距
const float k_m_wheel = 1.62f;//轮的质量
const float k_m_leg=0.92f;
const float length1 = 0.0955f;
const float length2 = 0.1142f;
const float length3 = 0.1142f;
const float length4 = 0.09556f;
const float length5 = 0.215f;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 根据关节角度和角速度,计算单杆长度和角度以及变化率
 *
 * @note 右侧视图
 *  ___x
 * |   A1  _____ E4
 * |y     /     \
 *     B2 \     /D3
 *         \   /
 *          \./
 *          C5
 * @param p 5连杆和腿的参数
 */
void Vmc::LegCalc() {
	//确定BD两点的横纵坐标
  coord_[0] = x_b_ = length1 * arm_cos_f32(phi1_);
  coord_[1] = y_b_ = length1 * arm_sin_f32(phi1_);
  coord_[4] = x_d_ = length4 * arm_cos_f32(phi4_);
  coord_[5] = y_d_ = length4 * arm_sin_f32(phi4_);
//计算phi2
  bd_ = sqrtf(powf(x_d_ - x_b_, 2) + powf(y_d_ - y_b_, 2));
  a0_ = 2 * length2 * (x_d_ - x_b_);
  b0_ = 2 * length2 * (y_d_ - y_b_);
  c0_ = powf(length2, 2) + powf(bd_, 2) - powf(length3 , 2);
  phi2_ = 2 * atan2f(b0_ + sqrtf(powf(a0_, 2) + powf(b0_, 2) - powf(c0_, 2)),
                     a0_ + c0_);
	//确定C点的横纵坐标
  coord_[2] = x_c_ = length5/length4*(x_b_ + length2 * arm_cos_f32(phi2_));
  coord_[3] = y_c_ = length5/length4*(y_b_ + length2 * arm_sin_f32(phi2_));

  phi0_ = atan2f(y_c_, x_c_);
  l0_ = sqrtf(powf(x_c_, 2) + powf(y_c_, 2));
  phi3_ = atan2f(y_c_ - y_d_, x_c_ - x_d_);
	//机体倾角得到腿部倾角
  theta_ = phi0_ - 0.5 * PI- phi_;
  height_ = l0_ * arm_cos_f32(theta_);

  // static float predict_dt = 0.001f;
  // float phi1_pred =
  //     phi1_ + w_phi1_ * predict_dt;  // 预测下一时刻的关节角度(利用关节角速度)
  // float phi4_pred = phi4_ + w_phi4_ * predict_dt;
  // // 重新计算腿长和腿角度
  // x_b_ = length1 * arm_cos_f32(phi1_pred);
  // y_b_ = length1 * arm_sin_f32(phi1_pred);
  // x_d_ = length4 * arm_cos_f32(phi4_pred);
  // y_d_ = length4 * arm_sin_f32(phi4_pred);
  // bd_ = sqrtf(powf(x_d_ - x_b_, 2) + powf(y_d_ - y_b_, 2));
  // a0_ = 2 * length2 * (x_d_ - x_b_);
  // b0_ = 2 * length2 * (y_d_ - y_b_);
  // c0_ = powf(length2, 2) + powf(bd_, 2) - powf(length3 , 2);
	// float phi2_pred;
//  phi2_pred =2 * atan2f(b0_ + sqrtf(powf(a0_, 2) + powf(b0_, 2) - powf(c0_, 2)),
  //                a0_ + bd_);
  // x_c_ = length5/length4*(x_b_ + length2 * arm_cos_f32(phi2_pred));
  // y_c_ = length5/length4*(y_b_ + length2 * arm_sin_f32(phi2_pred));

  // float phi0_pred = atan2f(y_c_, x_c_);
  // // 差分计算腿长变化率和腿角速度
  // w_phi2_ = (phi2_pred - phi2_) / predict_dt;
  // w_phi0_ = (phi0_pred - phi0_) / predict_dt;
  // v_l0_ = ((sqrtf(powf(x_c_ , 2) + powf(y_c_, 2))) - l0_) /
  //         predict_dt;
  // w_theta_ = (phi0_pred - 0.5 * PI - phi_ - theta_) / predict_dt;
  // v_height_ =
  //     v_l0_ * arm_cos_f32(theta_) - l0_ * arm_sin_f32(theta_) * w_theta_;
  // dot_v_l0_ = (v_l0_ - last_v_l0_)*2/ (0.002f + 0.008f) +
  //             dot_v_l0_ * 0.008f / (0.002f + 0.008f);
  // dotw_theta_ = (w_theta_ - last_w_theta_) *2/ (0.002f + 0.008f) +
  //               dotw_theta_ * 0.008f / (0.002f + 0.008f);
  // ddot_z_w_ = ddot_z_M_ - dot_v_l0_ * arm_cos_f32(theta_) +
  //             2.0f * v_l0_ * w_theta_ * arm_sin_f32(theta_) +
  //             l0_ * dotw_theta_ * arm_sin_f32(theta_) +
  //             l0_ * powf(w_theta_, 2) * arm_cos_f32(theta_);
  // last_w_theta_ = w_theta_;
  // last_v_l0_ = v_l0_;
}

/**
 * @brief Calculate the torque for Vmc.
 */
void Vmc::TorCalc() {
  T1_ = j_[0] * F_ + j_[2] * Tp_;
  T2_ = j_[1] * F_ + j_[3] * Tp_;
}

// void Vmc::Jacobian() {
// 	inv_j_[0]=-cos(phi0_ - phi2_)/(k_thigh_len*sin(phi1_ - phi2_));
// 	inv_j_[1]=l0_*sin(phi0_ - phi2_)/(k_thigh_len*sin(phi1_ - phi2_));
// 	inv_j_[2]=cos(phi0_ - phi3_)/(k_thigh_len*sin(phi3_ - phi4_));
// 	inv_j_[3]=-l0_*sin(phi0_ - phi3_)/(k_thigh_len*sin(phi3_ - phi4_));
// j_[0]=(k_thigh_len*(0.5*arm_cos_f32(phi0_ + phi1_ - phi2_ - phi3_) - 0.5*arm_cos_f32(phi0_ - phi1_ + phi2_ - phi3_)))/arm_sin_f32(phi2_ - phi3_);
// j_[1]=(k_thigh_len*(0.5*arm_cos_f32(phi0_ - phi2_ + phi3_ - phi4_) - 0.5*arm_cos_f32(phi0_ - phi2_ - phi3_ + phi4_)))/arm_sin_f32(phi2_ - phi3_);
// j_[2]=-(k_thigh_len*(0.5*arm_sin_f32(phi0_ + phi1_ - phi2_ - phi3_) - 0.5*arm_sin_f32(phi0_ - phi1_ + phi2_ - phi3_)))/(l0_*arm_sin_f32(phi2_ - phi3_));
// j_[3]=-(k_thigh_len*(0.5*arm_sin_f32(phi0_ - phi2_ + phi3_ - phi4_) - 0.5*arm_sin_f32(phi0_ - phi2_ - phi3_ + phi4_)))/(l0_*arm_sin_f32(phi2_ - phi3_));
// }
void Vmc::Jacobian()
{
  j_[0] = -length1*length5*arm_sin_f32(phi0_-phi3_)*arm_sin_f32(phi1_ - phi2_)/length4/arm_sin_f32(phi2_-phi3_);
  j_[1] = -length5*arm_sin_f32(phi0_-phi2_)*arm_sin_f32(phi3_-phi4_)/arm_sin_f32(phi2_-phi3_);
  j_[2] = -length5*length1*arm_cos_f32(phi0_-phi3_)*arm_sin_f32(phi1_-phi2_)/l0_/length4/arm_sin_f32(phi2_-phi3_);
  j_[3] = -length5*arm_cos_f32(phi0_-phi2_)*arm_sin_f32(phi3_-phi4_)/l0_/arm_sin_f32(phi2_-phi3_);

  inv_j_[0] = -length4*arm_cos_f32(phi0_-phi2_)/length1/length5/arm_sin_f32(phi1_-phi2_);
  inv_j_[1] = l0_*length4*arm_sin_f32(phi0_-phi2_)/length1/length5/arm_sin_f32(phi1_-phi2_);
  inv_j_[2] = arm_cos_f32(phi0_-phi3_)/length5/arm_sin_f32(phi3_-phi4_);
  inv_j_[3] = -l0_*arm_sin_f32(phi0_-phi3_)/length5/arm_sin_f32(phi3_-phi4_);

}
void Vmc::LegForceCalc() {
  mea_F_ = inv_j_[0] * mea_t1_ + inv_j_[2] * mea_t2_;
  mea_Tp_ = inv_j_[1] * mea_t1_ + inv_j_[3] * mea_t2_;
  p_ = mea_F_ * arm_cos_f32(theta_) + mea_Tp_ * arm_sin_f32(theta_) / l0_;
  F_N_ = p_ + k_m_wheel * (9.8f +ddot_z_w_)+k_m_leg*9.8;
}
