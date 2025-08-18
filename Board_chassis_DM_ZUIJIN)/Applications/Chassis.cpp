#include "Chassis.h"
#include "ins.h"
#include "bsp_dwt.h"
#include "usart.h"
#include "board_comm.h"
#include "fdcan.h"
#include "BSP_fdcan.h"
/*************const***************/
const float k_gravity_comp =5.96 * 9.8f;
const float k_wheel_radius=0.091;
const float k_phi1_bias =0.9124f + 3.1415f;
const float k_phi4_bias = 3.1415f;

const float k_lf_joint_bias =0.4832;
const float k_lb_joint_bias =6.0972;
const float k_rf_joint_bias =1.6409;
const float k_rb_joint_bias =2.9341;

const float k_jump_force = 120.0f;
const float k_jump_time = 0.1f;
const float k_retract_force = -100.0f;
const float k_retract_time = 0.15f;                                                                 

const float leg_max = 0.4057;
const float leg_min = 0.2226;

static float target_yaw;

float test_data = 0;
float input_angel = 0;


/***********************************/

balance_Chassis chassis;
/**
 * @brief 腿部电机初始化
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::MotorInit()
{
	// M1505B_FUN.M1505B_Disable();
	// M1505B_FUN.M1505B_Enable();

	M6020_Fun.M6020_Enable();
  M3508_FUN.M3508_Init();
	lf_joint_.Init(&hfdcan1,0x01,0x01,Motor_DM_Control_Method_NORMAL_MIT,P_MAX,V_MAX,T_MAX,I_MAX);
	lb_joint_.Init(&hfdcan1,0x02,0x02,Motor_DM_Control_Method_NORMAL_MIT,P_MAX,V_MAX,T_MAX,I_MAX);
  rf_joint_.Init(&hfdcan1,0x03,0x03,Motor_DM_Control_Method_NORMAL_MIT,P_MAX,V_MAX,T_MAX,I_MAX);
  rb_joint_.Init(&hfdcan1,0x04,0x04,Motor_DM_Control_Method_NORMAL_MIT,P_MAX,V_MAX,T_MAX,I_MAX);
  //设置电机零点位置，只在需要重新设置零点时使用
  // lf_joint_.Save_Pos_Zero();
  // lb_joint_.Save_Pos_Zero();
  //  rf_joint_.Save_Pos_Zero();
//    rb_joint_.Save_Pos_Zero();


}
/**
 * @brief 底盘状态初始化
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::StatusInit()
{
     robot_status = STATE_NORMAL;
     jump_status = JUMP_NONE;
     recover_status = RECOVER_NONE;
}
/**
 * @brief 所有所需PID初始化
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::PidInit() {
  left_leg_len_.Init(300.0f, 0.0f,10.0f, 200.0f, 0.001f);
  right_leg_len_.Init(300.0f, 0.0f, 10.0f,200.0f, 0.001f);
  anti_crash_.Init(60.0f, 1.0f, 4.0f, 10.0f, 0.001f);
	roll_comp_.Init(600.0f,0.0f,10.0f,80.0f,0.001f);
}

/**
 * @brief 速度和加速度卡尔曼滤波观测器
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::SpeedEstInit() {
  // 使用kf同时估计速度和加速度
  Kalman_Filter_Init(&kf, 2, 0, 2);
  float F[4] = {1, 0.005, 0, 1};//A矩阵由于d_t为0.001所以1行2列处为0.001
  float Q[4] = {VEL_PROCESS_NOISE, 0, 0, ACC_PROCESS_NOISE};//状态变量过程噪声矩阵
  float R[4] = {VEL_MEASURE_NOISE, 0, 0, ACC_MEASURE_NOISE};//测量噪声矩阵
  float P[4] = {100000, 0, 0, 100000};//状态估计误差协方差矩阵
  float H[4] = {1, 0, 0, 1};
  memcpy(kf.F_data, F, sizeof(F));
  memcpy(kf.Q_data, Q, sizeof(Q));
  memcpy(kf.R_data, R, sizeof(R));
  memcpy(kf.P_data, P, sizeof(P));
  memcpy(kf.H_data, H, sizeof(H));
}

/**
 * @brief 腿部状态正运动学计算
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::LegCalc() {
  // left_leg_.SetBodyData(INS.Pitch * DEGREE_2_RAD, INS.MotionAccel_n[2]);
  // right_leg_.SetBodyData(INS.Pitch * DEGREE_2_RAD, INS.MotionAccel_n[2]);
  // left_leg_.SetLegData(lb_joint_.GetAngle() + k_phi1_bias, lb_joint_.GetSpeed(),
  //                      lf_joint_.GetAngle() + k_phi4_bias, lf_joint_.GetSpeed(),
  //                      lb_joint_.GetTor(), lf_joint_.GetTor());
  right_leg_.SetLegData(
      -rb_joint_.GetAngle() , -rb_joint_.GetSpeed(),
      -rf_joint_.GetAngle() + k_phi4_bias, -rf_joint_.GetSpeed(),
      -rb_joint_.GetTor(), -rf_joint_.GetTor());

  // left_leg_.LegCalc();
  right_leg_.LegCalc();
  // left_leg_.Jacobian();
  right_leg_.Jacobian();
  // left_leg_.LegForceCalc();
  // right_leg_.LegForceCalc();
}

/**
 * @brief LQR计算
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::LQRCalc() {
  lqr_body_.SetSpeed(target_speed_);
	lqr_body_.SetDist(target_dist_);
	lqr_body_.SetRotation(target_rotation_);
	lqr_body_.SetWRotation(target_w_rotation_);
	//避免纹波干扰
	if(fabs(vel_)<0.1f)
	{dist_ = vel_ * controller_dt_;
	}
	if(fabs(w_rotation_)<0.1f)
	{
		rotation_=w_rotation_ * controller_dt_;
	}
  lqr_body_.SetData(dist_,  vel_, 
	rotation_,INS.Gyro[2],
  left_leg_.GetTheta(), left_leg_.GetDotTheta(),
	right_leg_.GetTheta(), right_leg_.GetDotTheta(),
	INS.Pitch * DEGREE_2_RAD, INS.Gyro[0],
  left_leg_.GetLegLen(), left_leg_.GetForceNormal(),
	right_leg_.GetLegLen(),right_leg_.GetForceNormal()
	);
	
  lqr_body_.Calc();
  SynthesizeMotion();
}

/**
 * @brief 控制力矩计算
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::TorCalc() {
  left_leg_.SetTor(left_leg_F_, left_leg_T_);
  right_leg_.SetTor(right_leg_F_, right_leg_T_);
  left_leg_.TorCalc();
  right_leg_.TorCalc();
}

/**
 * @brief 力矩控制下发电机条件判断
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::TorControl() {
//if(lf_joint_.DM_Rev.id==0x01&&lb_joint_.DM_Rev.id==0x02&&rf_joint_.DM_Rev.id==0x03&&rb_joint_.DM_Rev.id==0x04)
//	{		
//		if(lf_joint_.GetAngle()<1.7&&lf_joint_.GetAngle()>-0.15&&lb_joint_.GetAngle()<0.15&&lb_joint_.GetAngle()>-1.7&&rf_joint_.GetAngle()<0.15&&rf_joint_.GetAngle()>-1.7&&rb_joint_.GetAngle()<1.7&&rb_joint_.GetAngle()>-0.15)
//		{
//		SetMotorTor();
//	//JStopMotor();
//		}
//		else
//		{
//			StopMotor();
//		}
//	}
//	else
//	{
//		StopMotor();
//	}

	SetMotorTor();
}

/**
 * @brief 腿长计算
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::LegLenCalc() {
  left_leg_len_.SetMeasure(left_leg_.GetLegLen());
  right_leg_len_.SetMeasure(right_leg_.GetLegLen());
  roll_comp_.SetRef(0);
  roll_comp_.SetMeasure(INS.Roll);
if (left_leg_.GetForceNormal() < 15.0f &&right_leg_.GetForceNormal() < 15.0f)
   {
 	   left_leg_F_ = left_leg_len_.Calculate() + k_gravity_comp ;
 	   right_leg_F_ = right_leg_len_.Calculate() + k_gravity_comp ;
   }
  else
 {
  left_leg_F_ = left_leg_len_.Calculate() + k_gravity_comp +roll_comp_.Calculate();
  right_leg_F_ = right_leg_len_.Calculate() + k_gravity_comp - roll_comp_.Calculate();
 }
  // left_leg_F_ = left_leg_len_.Calculate();
  // right_leg_F_ = right_leg_len_.Calculate();
}


/**q
 * @brief LQR运动控制量整合
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::SynthesizeMotion() {

    l_wheel_T_ = lqr_body_.GetWheelTorL();
    r_wheel_T_ = lqr_body_.GetWheelTorR();
  anti_crash_.SetRef(0.0f);
  anti_crash_.SetMeasure(left_leg_.GetPhi0() - right_leg_.GetPhi0());
  anti_crash_.Calculate();
  left_leg_T_ = lqr_body_.GetLegTorL() + anti_crash_.GetOutput();
  right_leg_T_ = lqr_body_.GetLegTorR() - anti_crash_.GetOutput();
}


/**
 * @brief 主控制指令计算循环
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::Controller() {
  SetState();
  LegCalc();
   SpeedCalc();
   UpdateChassisStatus();
   ChassissControl();
  //  LQRCalc();
  // SynthesizeMotion();
  // if (jump_state_ == true)
  //   Jump();
  LegLenCalc();
  TorCalc();
	TorControl();
}

/**
 * @brief 力矩指令设置
 * @param 
 * @param 
 * @param
 */
// 电机力矩输入模式
void balance_Chassis::SetMotorTor() {
//  lf_joint_.SetMotorT(left_leg_.GetT2());
//   lb_joint_.SetMotorT(left_leg_.GetT1());
  rf_joint_.SetMotorT(-right_leg_.GetT2());
  rb_joint_.SetMotorT(-right_leg_.GetT1());
  // rf_joint_.SetMotorT(0);
  // rb_joint_.SetMotorT(0);
  
	M3508_Array[0].targetTorque=l_wheel_T_;
	M3508_Array[1].targetTorque=-r_wheel_T_;
	M3508_FUN.M3508_SetTor();
}
/**
 * @brief 力矩指令设置为0-急停
 * @param 
 * @param 
 * @param
 */
// 电机急停模式
void balance_Chassis::StopMotor() {
  lf_joint_.SetMotorT(0.0f);
  rf_joint_.SetMotorT(0.0f);
  lb_joint_.SetMotorT(0.0f);
  rb_joint_.SetMotorT(0.0f);
	M3508_Array[0].targetTorque=0;
	M3508_Array[1].targetTorque=0;
	M3508_FUN.M3508_SetTor();
	M3508_Array[0].sendCurrent=0;
	M3508_Array[1].sendCurrent=0;
}

/**
 * @brief 设置腿长
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::SetLegLen() {
target_len_=(float)(board_comm.rece_.len/660.0*0.1)+0.31415;
left_leg_len_.SetRef(target_len_);
right_leg_len_.SetRef(target_len_);

}

/**
 * @brief 设置YAW
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::SetFollow()
{
if (board_comm.rece_.AutoAimFlag == 1)
{
static float delta_aim_yaw = 0.0f;
M6020s_Yaw.AutoAimFlag = 1;
delta_aim_yaw -= board_comm.rece_.yaw_speed * 0.006;
M6020s_Yaw.targetAngle = board_comm.rece_.yaw_position + delta_aim_yaw + Saber_Angle.Yaw/360.0f*8192;
if (M6020s_Yaw.targetAngle > 8192)
{
M6020s_Yaw.targetAngle -= 8192;
}
else if (M6020s_Yaw.targetAngle < 0)
{
M6020s_Yaw.targetAngle += 8192;
}
float delta_yaw= M6020s_Yaw.targetAngle - M6020s_Yaw.realAngle;
if(delta_yaw<-4096)
delta_yaw+=8192;
if(delta_yaw>4096)
delta_yaw-=8192;
//M6020s_Yaw.targetSpeed = Position_PID(&M6020s_Yaw.Aim_position_PID, 0, delta_yaw);
//M6020s_Yaw.outCurrent = Position_PID(&M6020s_Yaw.Aim_position_PID, M6020s_Yaw.realSpeed, M6020s_Yaw.targetSpeed);
}
else if (board_comm.rece_.AutoAimFlag == 0)
{
M6020s_Yaw.AutoAimFlag = 0;
M6020s_Yaw.targetAngle -= board_comm.rece_.yaw_speed * 0.006f - 0* Saber_Angle.Gyro_z;
if (M6020s_Yaw.targetAngle > 8192)
{
M6020s_Yaw.targetAngle -= 8192;
}
else if (M6020s_Yaw.targetAngle < 0)
{
M6020s_Yaw.targetAngle += 8192;
}
float delta_yaw= M6020s_Yaw.targetAngle - M6020s_Yaw.realAngle;
if(delta_yaw<-4096)
delta_yaw+=8192;
if(delta_yaw>4096)
delta_yaw-=8192;
M6020s_Yaw.targetSpeed = Position_PID(&M6020s_Yaw.position_PID, 0, delta_yaw);
M6020s_Yaw.outCurrent = Position_PID(&M6020s_Yaw.velocity_PID, M6020s_Yaw.realSpeed, M6020s_Yaw.targetSpeed);
}
}
/**
 * @brief 设置速度
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::SetSpd() {
// target_speed_=(float)(board_comm.rece_.speed/660.0)*SPEED_MAX;
// target_w_rotation_=(float)(board_comm.rece_.w_speed/1320.0)*	W_SPEED_MAX;
right_leg_T_ = (float)(board_comm.rece_.speed/660)*2;
target_dist_=0;
target_rotation_=0;
}

/**
 * @brief 设置机器人状态
 * @param 
 * @param ------------------------------
 * @param
 */
void balance_Chassis::SetState() {
  controller_dt_ = DWT_GetDeltaT(&dwt_cnt_controller_);
  
  if(robot_status==STATE_NORMAL)
  {
    SetLegLen();
    SetSpd();
    SetFollow();
  }
	if(jump_cnt==0&&board_comm.rece_.jump_flag==1)
	{
		jump_state_=true;
		jump_cnt=1;
	}
  
}

/**
 * @brief 跳跃
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::Jump() {

    if (jump_state_ == true && last_jump_state_ == false) {
      jump_start_time_ = HAL_GetTick() / 1000.0f;
    }

    jump_now_time_ = HAL_GetTick() / 1000.0f;

    if (fabs(jump_now_time_ - jump_start_time_) <= k_jump_time) {
      left_leg_F_ = k_jump_force;
      right_leg_F_ = k_jump_force;
    }
		 jump_now_time_ = HAL_GetTick() / 1000.0f;
		if ((jump_now_time_ - jump_start_time_ - k_jump_time) <= k_retract_time &&
        (jump_now_time_ - jump_start_time_) > k_jump_time) {
      left_leg_F_ = k_retract_force;
      right_leg_F_ = k_retract_force;
			l_wheel_T_=0;
			r_wheel_T_=0;
    }
    if ((jump_now_time_ - jump_start_time_ - k_jump_time) > k_retract_time) {
      jump_state_ = false;
    } 

    last_jump_state_ = jump_state_;

}

/**
 * @brief 速度计算
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::SpeedCalc() {
  left_w_wheel_ = M3508_Array[0].realSpeed/60*2*3.14159+ left_leg_.GetPhi2Speed() - INS.Gyro[0];
  right_w_wheel_ =-M3508_Array[1].realSpeed/60*2*3.14159+ right_leg_.GetPhi2Speed() - INS.Gyro[0];

  left_v_body_ = left_w_wheel_ * k_wheel_radius +
                 left_leg_.GetLegLen() * left_leg_.GetDotTheta()*arm_cos_f32(left_leg_.GetTheta()) +
                 left_leg_.GetLegSpeed() * arm_sin_f32(left_leg_.GetTheta());
  right_v_body_ = right_w_wheel_ * k_wheel_radius +
                  right_leg_.GetLegLen() * right_leg_.GetDotTheta()*arm_cos_f32(left_leg_.GetTheta()) +
                  right_leg_.GetLegSpeed() * arm_sin_f32(right_leg_.GetTheta());
  vel_m = (left_v_body_ + right_v_body_) / 2;
  if (left_leg_.GetForceNormal() < 20.0f &&right_leg_.GetForceNormal() < 20.0f) {
    vel_m = 0;

  }

  // 使用kf同时估计加速度和速度,滤波更新
  kf.MeasuredVector[0] = vel_m;
  kf.MeasuredVector[1] = INS.MotionAccel_n[1];
  kf.F_data[1] = controller_dt_;  // 更新F矩阵
  Kalman_Filter_Update(&kf);
  vel_ = kf.xhat_data[0];
  acc_ = kf.xhat_data[1];
	
}

/**
 * @brief 根据机器人的各个状态量判断机器人所处状态
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::UpdateChassisStatus()
{
  //判断是否需要自起
  if(fabs(INS.Pitch)>3.1415926f/4.0f)
  {
    robot_status = STATE_RECOVERING;
		recover_status = RECOVER_SHRINK;
  }
  else if(jump_state_==true)
  {
    robot_status = STATE_JUMPING;
  }

  else{
    robot_status = STATE_NORMAL;
  }
} 
/**
 * @brief 底盘控制
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::ChassissControl()
{
   switch(robot_status)
  {
    case STATE_NORMAL:
         LQRCalc();
         break;
    case STATE_RECOVERING:
         RecoverCalc();
         break;
    case STATE_JUMPING:
         JumpCalc();
         break;
  }
}
/**
 * @brief 倒地自起控制
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::RecoverCalc()
{
  switch(recover_status)
  {
    case RECOVER_SHRINK:
         left_leg_len_.SetRef(0.2);   //调整腿长
         right_leg_len_.SetRef(0.2);
         l_wheel_T_ = 0;
         r_wheel_T_ = 0;

         recover_timer++;

        if(recover_timer >= 20 && left_leg_.GetLegLen() < 0.3 && right_leg_.GetLegLen() < 0.3)
        {
          recover_status = RECOVER_ADJUST;
          recover_timer = 0;
        }
        break;
    case RECOVER_ADJUST:
         left_leg_T_ = 0;   //调整髋关节角度
         right_leg_T_ = 0;
         l_wheel_T_ = 0;
         r_wheel_T_ = 0;

         recover_timer++;

         if(recover_timer >= 20 && left_leg_.GetPhi0() < 0.3 && right_leg_.GetPhi0() < 0.3)
         {
          recover_status = RECOVER_EXTEND;
          recover_timer++;
         }
         break;
    case RECOVER_EXTEND:
         left_leg_len_.SetRef(0.5);   //伸长腿
         right_leg_len_.SetRef(0.5);
         l_wheel_T_ = 0;
         r_wheel_T_ = 0;
         recover_timer++;

         if(recover_timer >= 20 && left_leg_.GetLegLen() > 0.45 && right_leg_.GetLegLen() > 0.45)
         {
          recover_status = RECOVER_EXTEND;
          recover_timer = 0;
          recover_status = RECOVER_NONE;
          robot_status = STATE_NORMAL;  // 恢复站立模式
         }
         break;
    case RECOVER_NONE:
         recover_timer = 0;
         break;
  }
}
/**
 * @brief 跳跃控制
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::JumpCalc()
{
    switch(jump_status)
    {
      case JUMP_COMPRESS:
           left_leg_len_.SetRef(0.2);   //调整腿长
           right_leg_len_.SetRef(0.2);
           l_wheel_T_ = 0;
           r_wheel_T_ = 0;
           jump_timer++;

           if(jump_timer >= 20 && left_leg_.GetLegLen() < 0.25 && right_leg_.GetLegLen() < 0.25)
           {
            jump_status = JUMP_ASCEND;
            jump_timer = 0;
           }
           break;
      case JUMP_ASCEND:
            left_leg_len_.SetRef(0.5);   //调整腿长
            right_leg_len_.SetRef(0.5);
            l_wheel_T_ = 0;
            r_wheel_T_ = 0;

            jump_timer++;

            if(jump_timer >= 20 && left_leg_.GetLegLen() > 0.48 && right_leg_.GetLegLen() > 0.48)
            {
               jump_status = JUMP_RETRACT;
               jump_timer = 0;
            }
            break;
      case JUMP_RETRACT:
            left_leg_len_.SetRef(0.2);   //调整腿长
            right_leg_len_.SetRef(0.2);
            l_wheel_T_ = 0;
            r_wheel_T_ = 0;

            jump_timer++;
            if(jump_timer >= 20 && left_leg_.GetLegLen() < 0.25 && right_leg_.GetLegLen() < 0.25)
            {
               jump_status = JUMP_NONE;
               jump_timer = 0;
               robot_status = STATE_NORMAL;
            }
            break;
      case JUMP_NONE:
            jump_timer = 0;
            break;
           

    }
    
}
// 实现C接口函数
extern "C" {
    void chassis_lf_joint_getInfo(FDCan_Export_Data_t data) {
        chassis.lf_joint_.DM_8009P_getInfo(data);
    }

    void chassis_lb_joint_getInfo(FDCan_Export_Data_t data) {
        chassis.lb_joint_.DM_8009P_getInfo(data);
    }

    void chassis_rf_joint_getInfo(FDCan_Export_Data_t data) {
        chassis.rf_joint_.DM_8009P_getInfo(data);
    }

    void chassis_rb_joint_getInfo(FDCan_Export_Data_t data) {
        chassis.rb_joint_.DM_8009P_getInfo(data);
    }
}

