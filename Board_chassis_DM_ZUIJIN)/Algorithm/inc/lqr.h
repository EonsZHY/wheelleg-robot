/**
 *******************************************************************************
 * @file      : lqr.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LQR_H_
#define __LQR_H_

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class Lqr {
 public:
  void Calc();
  void SetData(const float _dist, const float _speed, 
								const float _rotation,const float _w_rotation,
								const float _theta_l,const float _w_theta_l,
								const float _theta_r,const float _w_theta_r,
								const float _body,const float _w_body, 
								const float _leg_len_l, const float _F_N_L,
								const float _leg_len_r, const float _F_N_R) {
    dist_ = _dist;
    speed_ = _speed;
		rotation_=_rotation;
		w_rotation_=_w_rotation;
		theta_l_=_theta_l;
		w_theta_l_=_w_theta_l;
		theta_r_=_theta_r;
		w_theta_r_=_w_theta_r;
		body_=_body;
		w_body_=_w_body;					 
    leg_len_l_ = _leg_len_l;
    F_N_L_ = _F_N_L;
		leg_len_r_=_leg_len_r;
		F_N_R_=_F_N_R;							
  }
  void SetSpeed(const float _speed) { target_speed_ = _speed; };
  void SetDist(const float _dist) { target_dist_ = _dist; };
	void SetRotation(const float _rotation){target_rotation_=_rotation;};
	void SetWRotation(const float _w_rotation){target_w_rotation_=_w_rotation;};
  float GetWheelTorL() { return T_[0]; };
	float GetWheelTorR() { return T_[1]; };
  float GetLegTorL() { return T_[2]; };
  float GetLegTorR() { return T_[3]; };
 private:
  float dist_, speed_,rotation_, w_rotation_, theta_l_, w_theta_l_,theta_r_,w_theta_r_,body_,w_body_, leg_len_l_,leg_len_r_;
  float target_speed_, target_dist_, target_rotation_,target_w_rotation_,F_N_L_,F_N_R_, T_[4], T_K_[4][10];
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif /* __LQR_H_ */
