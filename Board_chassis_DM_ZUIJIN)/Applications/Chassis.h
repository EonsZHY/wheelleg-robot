#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "lqr.h"
#include "pid.h"
#include "vmc.h"
#include "tpid.h"
#include "kalman_filter.h"
#include "M6020_Motor.h"
#include "M3508.h"
#include "DM_8009P.h"
#include "dji_motor.h"
#include "GM6020.h"
#include "motor_pid.h"

#define RAD_2_DEGREE 57.2957795f     // 180/pi
#define DEGREE_2_RAD 0.01745329252f  // pi/180
#define VEL_PROCESS_NOISE 16
#define VEL_MEASURE_NOISE 800
#define ACC_PROCESS_NOISE 2000  
#define ACC_MEASURE_NOISE 0.01  

// #define SPEED_MAX 2
// #define W_SPEED_MAX  4

#define MAX_LEG_LENGTH 0.4f //最大腿长
#define MIN_LEG_LENGTH 0.2f //最短腿长
// ������״̬ö��
typedef enum {
    STATE_NORMAL,       // ��������/ƽ��
    STATE_JUMPING,      // ��Ծ��
    STATE_RECOVERING,   // ����������
} RobotState;

// ��Ծ�׶�ö��
typedef enum {
    JUMP_NONE,          // δ��Ծ
    JUMP_COMPRESS,      // ѹ�������׶�
    JUMP_ASCEND,        // �����׶�
    JUMP_RETRACT,       // ���Ƚ׶�
} JumpPhase;

// ����׶�ö��
typedef enum {
	RECOVER_NONE,
    RECOVER_SHRINK,     // �����Ȳ�
    RECOVER_ADJUST,     // ������̬
    //RECOVER_EXTEND,     // ��չ�Ȳ�
} RecoverPhase;
//底盘数据结构体
typedef struct Speed_ToCloud
 {
        float vx;
        float vy;
        float wz;
 }Speed_ToCloud;

typedef struct Speed_ToChassis
{
        float vx;
        float vy;
        float wz;
}Speed_ToChassis;
#ifdef __cplusplus
class balance_Chassis
{
	public:
		// Unitree_Motor joint,lf_joint_, lb_joint_, rf_joint_, rb_joint_;
		Class_Motor_DM_8009P lf_joint_, lb_joint_, rf_joint_, rb_joint_;
		Class_Motor_3508 left_wheel,right_wheel;
		Vmc left_leg_, right_leg_;
		KalmanFilter_t kf,kf_l,kf_r;
		Lqr lqr_body_;
		Pid left_leg_len_, right_leg_len_, anti_crash_,roll_comp_, left_leg_phi0, right_leg_phi0;
		Speed_ToCloud speed_to_cloud;
		Speed_ToChassis speed_to_chassis;
		void motor_test_init();
	  float MotorAngle();
		void LegCalc();
		void MotorInit();
		void PidInit();
		void StatusInit();
		void Controller();
		void TorCalc();
		void TorControl();
		void LQRCalc();
		void LegLenCalc();
		void SpeedCalc();
		void SynthesizeMotion();
		void Jump();
		void SpeedEstInit();
		void SetMotorTor();
		void StopMotor();
		void SetLegLen();
		void SetFollow();
		void SetState();
		void SetSpd();
		void RecoverCalc();
		void JumpCalc();
		void UpdateChassisStatus();
		void ChassissControl();
		void getangle(float angle);

	private:
		float left_leg_F_, right_leg_F_, roll_comp;
		float l_wheel_T_, r_wheel_T_, left_leg_T_, right_leg_T_;
		float  target_rotation_,target_w_rotation_, target_speed_, target_dist_, vel_, dist_, acc_,rotation_,w_rotation_,target_len_;
		float vel_m, left_v_body_, right_v_body_, left_w_wheel_, right_w_wheel_;
		float jump_start_time_, jump_now_time_;
		uint32_t dwt_cnt_controller_, dwt_cnt_observer;
		bool jump_state_ = false, last_jump_state_ = false;
		bool recover_state_ = false;
		float controller_dt_, observer_dt_;
		uint8_t jump_cnt=0;
		RobotState robot_status;
		JumpPhase jump_status;
		RecoverPhase recover_status;
		float recover_timer;
		float jump_timer;
		float Angle_ChassisToCloud;
};


extern balance_Chassis chassis;
#endif
#endif