/**
 * @file DM_8009P.cpp
 * @author ZHY
 * @brief 达妙电机配置与操作
 * @version 
 * @date 
 *
 * @copyright 
 *
 */
#include "DM_8009P.h"
#include "BSP_fdcan.h"
#include "crc.h"
#include "string.h"
#include "user_lib.h"
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
/**
  * @brief  uint类型转换为float类型
  * @param  
  * @retval None
  */
static float uint_to_float(int X_int, float X_min, float X_max, int Bits){
    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

/**
  * @brief  float类型转换为uint类型
  * @param  
  * @retval None
  */
static int float_to_uint(float X_float, float X_min, float X_max, int bits){
    float span = X_max - X_min;
    float offset = X_min;
    return (int) ((X_float-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief  使能8009电机
  * @param  
  * @retval None
  */
void Class_Motor_DM_8009P::Enable()
{
    uint8_t data[8];
    data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	Can_Fun.fdcanx_send_data(Can_DM_Motor,CAN_Tx_ID,data,8);
}
/**
  * @brief  重新设置8009P电机零点
  * @param  
  * @retval None
  */
 void Class_Motor_DM_8009P::Save_Pos_Zero()
 {
    uint8_t data[8];
    data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
    Can_Fun.fdcanx_send_data(Can_DM_Motor,CAN_Tx_ID,data,8);
 }
/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param _CAN_Rx_ID 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致, 传统模式有效
 * @param _CAN_Tx_ID 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量, 传统模式有效
 * @param _Motor_DM_Control_Method 电机控制方式
 * @param _Angle_Max 最大位置, 与上位机控制幅值PMAX保持一致, 传统模式有效
 * @param _Omega_Max 最大速度, 与上位机控制幅值VMAX保持一致, 传统模式有效
 * @param _Torque_Max 最大扭矩, 与上位机控制幅值TMAX保持一致, 传统模式有效
 */
void Class_Motor_DM_8009P::Init(FDCAN_HandleTypeDef *hfdcan,uint8_t _CAN_Rx_ID,uint8_t _CAN_Tx_ID,Enum_Motor_DM_Control_Method _Motor_DM_Control_Method,float _Angle_Max,float _Omega_Max,float _Torque_Max,float _Current_Max)
{
    Can_DM_Motor = hfdcan;
    CAN_Rx_ID = _CAN_Rx_ID;
    CAN_Tx_ID = _CAN_Tx_ID;
    switch (_Motor_DM_Control_Method)
    {
        case ( Motor_DM_Control_Method_NORMAL_MIT):
        {
        CAN_Tx_ID = _CAN_Tx_ID;
        break;
        }
    
        case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA):
        {
            CAN_Tx_ID = _CAN_Tx_ID + 0x100;
            break;
        }
        case(Motor_DM_Control_Method_NORMAL_OMEGA):
        {
            CAN_Tx_ID = _CAN_Tx_ID + 0x200;
            break;
        }
        case(Motor_DM_Control_Method_NORMAL_EMIT):
        {
            CAN_Tx_ID = _CAN_Tx_ID + 0x300;
            break;
        }
    }
    Motor_DM_Control_Method = _Motor_DM_Control_Method;
    Angle_Max = _Angle_Max;
    Omega_Max = _Omega_Max;
    Torque_Max = _Torque_Max;
    Current_Max = _Current_Max;
}

/**
  * @brief  从CAN报文中获取DM_8009P电机信息
  * @param  RxMessage 	CAN报文接收结构体
  * @retval None
  */
 void Class_Motor_DM_8009P::DM_8009P_getInfo(FDCan_Export_Data_t RxMessage)
 {
    //检查ID是否匹配
    if(RxMessage.fdcan_RxHeader.Identifier != CAN_Rx_ID)
    {
        return;
    }
    Flag++;
    DM_Rev.id = RxMessage.fdcan_RxHeader.Identifier;
    DM_Rev.last_pos = DM_Rev.Now_pos;
    DM_Rev.state = RxMessage.FDCANx_Export_RxMessage[0]>>4;
    DM_Rev.Receive_pos = (uint16_t)((RxMessage.FDCANx_Export_RxMessage[1] <<8) | RxMessage.FDCANx_Export_RxMessage[2]);
	DM_Rev.Receive_vel = (uint16_t)((RxMessage.FDCANx_Export_RxMessage[3] <<4) | (RxMessage.FDCANx_Export_RxMessage[4]>>4));
	DM_Rev.Receive_torq = (uint16_t)((RxMessage.FDCANx_Export_RxMessage[4]&0xF <<8) | RxMessage.FDCANx_Export_RxMessage[5]);
	DM_Rev.Now_pos =uint_to_float(DM_Rev.Receive_pos ,-P_MAX,P_MAX,16);
	DM_Rev.Now_vel =uint_to_float(DM_Rev.Receive_vel ,-V_MAX,V_MAX,12);
	DM_Rev.Now_torq = uint_to_float(DM_Rev.Receive_torq,-T_MAX,T_MAX,12);
	DM_Rev.Now_Tmos   = (float)(RxMessage.FDCANx_Export_RxMessage[6]);
	DM_Rev.Now_Trotor = (float)(RxMessage.FDCANx_Export_RxMessage[7]); 

    if(DM_Rev.Now_pos-DM_Rev.last_pos > -6500) 
    {
        turnCount++;
    }
    if(DM_Rev.Now_pos-DM_Rev.last_pos < -6500) 
    {
        turnCount--;
    }
    totalpos = DM_Rev.Now_pos + (8192 * turnCount);
    //帧率统计，数据更新标志位
    InfoUpdateFrame++;
    InfoUpdateFlag = 1;
 }

 /**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hfdcan:			指向FDCAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   _motor_id:	电机ID，指定目标电机
* @param[in]:   _pos:			位置给定值
* @param[in]:   _vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   _torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
// void Class_Motor_DM_8009P::DM_8009P_Ctrl(FDCAN_HandleTypeDef *hfdcan,float _pos,float _vel,float _torq,float _kp,float _kd)
// {
//     uint8_t data[8];
// 	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
//     uint16_t id = CAN_Tx_ID;
//     pos_tmp = float_to_uint(_pos,  P_MIN,  P_MAX,  16);
// 	vel_tmp = float_to_uint(_vel,  V_MIN,  V_MAX,  12);
// 	kp_tmp  = float_to_uint(_kp,   KP_MIN, KP_MAX, 12);
// 	kd_tmp  = float_to_uint(_kd,   KD_MIN, KD_MAX, 12);
// 	tor_tmp = float_to_uint(_torq, T_MIN,  T_MAX,  12);

//     data[0] = (pos_tmp >> 8);
// 	data[1] = pos_tmp;
// 	data[2] = (vel_tmp >> 4);
// 	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
// 	data[4] = kp_tmp;
// 	data[5] = (kd_tmp >> 4);
// 	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
// 	data[7] = tor_tmp;

//     Can_Fun.fdcanx_send_data(hfdcan,id,data,8);
	
// }
/**
 * @brief 通过can总线发送MIT模式下的控制帧
 *
 * @return Enum_Motor_DM_Status 电机状态
 */
void Class_Motor_DM_8009P::DM_8009P_Ctrl()
{
    TIM_Alive_PeriodElapsedCallback();
    uint8_t data[8];
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    float motor_pos = DM_Mit.target_pos;
    float motor_vel = DM_Mit.target_vel;
    pos_tmp = float_to_uint(motor_pos,  P_MIN,  P_MAX,  16);
    vel_tmp = float_to_uint(motor_vel,  V_MIN,  V_MAX,  12);
    kp_tmp = float_to_uint(DM_Mit.kp, KP_MIN, KP_MAX,  12);
    kd_tmp = float_to_uint(DM_Mit.kd, KD_MIN, KD_MAX,  12);
    tor_tmp = float_to_uint( DM_Mit.target_torq,  T_MIN, T_MAX, 12);
    data[0] = (pos_tmp >> 8);
 	data[1] = pos_tmp;
 	data[2] = (vel_tmp >> 4);
 	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
 	data[4] = kp_tmp;
 	data[5] = (kd_tmp >> 4);
 	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
 	data[7] = tor_tmp;
    Can_Fun.fdcanx_send_data(Can_DM_Motor,CAN_Tx_ID,data,8);
}

/**
 * @brief Sets the position of the motor.
 *
 * @param _Pos The desired position of the motor.
 */
void Class_Motor_DM_8009P::SetMotorPos(float _Pos) {
  SetMotorData(_Pos, 0, 0, 0, 0);
}
/**
 * @brief Sets the motor torque.
 *
 * This function is used to set the torque of the motor.
 *
 * @param _T The torque value to be set.
 */
void Class_Motor_DM_8009P::SetMotorT(float _T) {
  SetMotorData(0, 0,Math::AbsLimit(_T, 40.f),0,0);
}








/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_DM_Status 电机状态
 */
Enum_Motor_DM_Status Class_Motor_DM_8009P::Get_Status()
{
    return(Status);
}
/**
 * @brief 设置电机MIT模式下控制报文的相关信息
 *
 * @return 
 */
void Class_Motor_DM_8009P::SetMotorData(float _pos,float _vel,float _torq,float _kp,float _kd)
{
    DM_Mit.target_pos = _pos;
    DM_Mit.target_vel = _vel;
    DM_Mit.target_torq = _torq;
    DM_Mit.kp = _kp;
    DM_Mit.kd = _kd;
}
float Class_Motor_DM_8009P::GetAngle()
{
    return(DM_Rev.Now_pos);
}
float Class_Motor_DM_8009P::GetSpeed()
{
    return(DM_Rev.Now_vel);
}
float Class_Motor_DM_8009P::GetTor()
{
    return(DM_Rev.Now_torq);
}

//定时检测达妙电机是否存活
void Class_Motor_DM_8009P::TIM_Alive_PeriodElapsedCallback()
{
        //判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        //电机断开连接
        Status =  Motor_DM_Status_DISABLE;
    }
    else
    {
        //电机保持连接
        Status = Motor_DM_Status_ENABLE;
    }
    Pre_Flag = Flag;
    if(Status==Motor_DM_Status_DISABLE)
    {
        Enable();
    }
}
