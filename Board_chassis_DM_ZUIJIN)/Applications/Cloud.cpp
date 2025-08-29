/**
 * @file Cloud.cpp
 * @author Cyx
 * @brief 
 * @version 0.1
 * @date 2023-08-15
 * 
 * @copyright 
 * 
 */

#include "Cloud.h"

Class_Cloud Cloud;

/**
 * @brief  云台初始化，配置参数并归位云台
 * @param  None
 * @retval None
 */
void Class_Cloud::Cloud_Init(void)
{

	//保存启动时刻的机械角度
	Target_Yaw = YAW.realAngle + AHRSData_Packet.Heading /(2*pi) *  8192.0f;
    //卡尔曼滤波器初始化 
    float F[1] = {1};
    float Q[1] = {1};//状态变量过程噪声矩阵
    float R[1] = {10};//测量噪声矩阵
    float P[1] = {100000};//状态估计误差协方差矩阵
    float H[1] = {1};
    Kalman_Filter_Init(&Cloud_YawMotorAngle_Error_Kalman, 1, 0, 1);
    Kalman_Filter_Init(&Cloud_YAWODKalman, 1, 0, 1);
    Kalman_Filter_Init(&Cloud_YawCurrent_Kalman, 1, 0, 1);
    Kalman_Filter_Init(&Cloud_YawCurrent_Kalman_manul, 1, 0, 1);
    memcpy(Cloud_YawMotorAngle_Error_Kalman.F_data, F, sizeof(F));
    memcpy(Cloud_YawMotorAngle_Error_Kalman.Q_data, Q, sizeof(Q));
    memcpy(Cloud_YawMotorAngle_Error_Kalman.R_data, R, sizeof(R));
    memcpy(Cloud_YawMotorAngle_Error_Kalman, P, sizeof(P));
    memcpy(Cloud_YawMotorAngle_Error_Kalman, H, sizeof(H));

    
    memcpy(Cloud_YAWODKalman.F_data, F, sizeof(F));
    memcpy(Cloud_YAWODKalman.Q_data, Q, sizeof(Q));
    memcpy(Cloud_YAWODKalman.R_data, R, sizeof(R));
    memcpy(Cloud_YAWODKalman.P_data, P, sizeof(P));
    memcpy(Cloud_YAWODKalman.H_data, H, sizeof(H));

    float R[1] = {6};//测量噪声矩阵
    memcpy(Cloud_YawCurrent_Kalman.F_data, F, sizeof(F));
    memcpy(Cloud_YawCurrent_Kalman.Q_data, Q, sizeof(Q));
    memcpy(Cloud_YawCurrent_Kalman.R_data, R, sizeof(R));
    memcpy(Cloud_YawCurrent_Kalman.P_data, P, sizeof(P));
    memcpy(Cloud_YawCurrent_Kalman.H_data, H, sizeof(H));

    memcpy(Cloud_YawCurrent_Kalman_manul.F_data, F, sizeof(F));
    memcpy(Cloud_YawCurrent_Kalman_manul.Q_data, Q, sizeof(Q));
    memcpy(Cloud_YawCurrent_Kalman_manul.R_data, R, sizeof(R));
    memcpy(Cloud_YawCurrent_Kalman_manul.P_data, P, sizeof(P));
    memcpy(Cloud_YawCurrent_Kalman_manul.H_data, H, sizeof(H));

}

/**
  * @brief  M6020_Yaw电机PID清除
  * @param  void
  * @retval void
  * @attention
  */
void Class_Cloud::PID_Clear_Yaw(void)
{
	YAW.Clear_PositionPIDData();
	YAW.Clear_PositionPIDData();
}

/**
  * @brief  M6020_Yaw电机角度调整（陀螺仪），修正电机电流数据
  * @param  void
  * @retval void
  * @attention
  */
void Class_Cloud::Cloud_Yaw_Angle_Set(void)
{
	/**************************云台Yaw6020电机双环控制计算*****************************/
	if(YAW.InfoUpdateFrame <= 30)
	{
		Target_Yaw = AHRSData_Packet.Heading /(2.0f*pi) *  8192.0f + YAW.realAngle ;
	}
	static uint8_t time=5;

	if (Target_Yaw > 8192)
	{
		Target_Yaw -= 8192;
	}
	else if (Target_Yaw < -8192)
	{
		Target_Yaw += 8192;
	}

	/**************************Yaw轴电机控制，遥控器数据映射到位置角度*****************/
	float Angle_Yaw_Chassis = AHRSData_Packet.Heading /(2.0f*pi) *  8192.0f ;/* 8192/360*/  //世界坐标系下的底盘yaw轴角度（转化为0-8191）
	float Angle_Yaw_Cloud = YAW.realAngle + Angle_Yaw_Chassis;	          //世界坐标系下的云台yaw轴角度（底盘角度+yaw轴电机角度）
	/*Angle_Yaw_Cloud值为-4096 ~ 8192+4096，Target为 0 ~ 8191，第一次调整Angle_Yaw_Cloud为 -4096 ~ 4096 */
	/*解决跨圈问题*/
	if (Angle_Yaw_Cloud > 4096 )
	{
		Angle_Yaw_Cloud -= 8192 ;
	}
	else if (Angle_Yaw_Cloud < -4096)
	{
		Angle_Yaw_Cloud += 8192 ;
	}
	ControlMes.yaw_realAngle = Angle_Yaw_Cloud;
	
	//Gimbal_Chassis_Pitch_Translate();    //云台相对底盘pitch轴角度赋值函数
	
	float Delta_Yaw = Angle_Yaw_Cloud - Target_Yaw + Linear*Saber_Angle.Z_Vel /*补偿saber传输yaw轴姿态角的滞后性*/  ;
	
	
	/*Derta的值 -4096-8191 ~ 4096 + 8191*/
	if ( Delta_Yaw <=  -4096)
	{
		Delta_Yaw += 8192 ;
	}
	else if ( Delta_Yaw >= 4096 )
	{
		Delta_Yaw -= 8192 ;
	}


	/*外环、内环，目标角度差，计算电流并滤波*/ /*Target_xxx为控制值*/

	if(ControlMes.AutoAimFlag==0)
	{
					/*死区*/
			if(Delta_Yaw < 5 && Delta_Yaw > -5)
			{
				Delta_Yaw = 0;
			}
			/*角度差值滤波*/
		  Delta_Yaw = One_Kalman_Filter(&Cloud_YawMotorAngle_Error_Kalman, Delta_Yaw);
			if( time >= kk )
			{
				YAW.targetSpeed = Position_PID(&M6020s_YawOPID,  0 ,Delta_Yaw);	
				time = 0;
			}
			YAW.outCurrent = Position_PID_Yaw(&M6020s_YawIPID, &FuzzyPID_Yaw, M6020s_Yaw.targetSpeed, M6020s_Yaw.realSpeed);
			YAW.outCurrent = One_Kalman_Filter(&Cloud_YawCurrent_Kalman_manul, M6020s_Yaw.outCurrent);
			time ++;
	}
	else if(ControlMes.AutoAimFlag == 1)
	{
			/*死区*/
		if(Delta_Yaw < 10 && Delta_Yaw > -10)
		{
			Delta_Yaw = 0;
		}
		YAW.targetSpeed = Position_PID(&AutoAim_M6020s_YawOPID,  0 ,Delta_Yaw);	
        YAW.outCurrent = Position_PID_Yaw(&AutoAim_M6020s_YawIPID, &FuzzyPID_AimYaw, M6020s_Yaw.targetSpeed, M6020s_Yaw.realSpeed);
		YAW.outCurrent = One_Kalman_Filter(&Cloud_YawCurrent_Kalman, M6020s_Yaw.outCurrent);
	}
}


/**
  * @brief  M6020电机输出
  * @param  void
  * @retval void
  * @attention
  */
void Class_Cloud::Cloud_Sport_Out(void)
{
	    /**********电流参数的设置**********/
		if(ControlMes.modelFlag == model_Record)
		{
			YAW.InfoUpdateFrame = 0;
			return;
		}
		else if(M6020s_Yaw.InfoUpdateFlag == 1)
		{
			Cloud_Yaw_Angle_Set();
		}
		else
		{
			return;
		}

	uint8_t data[8] = {0};
	
	 /**********传递Yaw编码器数值**********/
	  float Angle_Cloud = M6020s_Yaw.realAngle;
		Angle_Cloud = M6020s_Yaw.realAngle +Setup_Angleoffset;
		if(Angle_Cloud > 4096)
		{
			Angle_Cloud -= 8192;
		}
		else if (Angle_Cloud < -4096)
		{
			Angle_Cloud += 8192;
		}
	 steer_getangle(-1*Angle_Cloud/8192.0f*360);

	/***************************将电流参数发送给电机*******************************/
	YAW.M6020_setVoltage(M6020s_Yaw.outCurrent, 0, 0, 0, data);
	Can_Fun.CAN_SendData(CAN_SendHandle, &hcan1, CAN_ID_STD, M6020_SENDID, data);
}