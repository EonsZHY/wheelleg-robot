/**
 * @file DM.h
 * @author ZHY
 * @brief 
 * @version 0.1
 * @date 2025-5-18
 * 
 * @copyright 
 * 
 */
#ifndef __DM_8009P_H__
#define __DM_8009P_H__

#include "fdcan.h"
#include "BSP_fdcan.h"
#include "string.h"
#include <stdbool.h>
#include <stdint.h>
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -15.0f
#define T_MAX 15.0f
#define I_MAX 30.0f 
#define DM_8009P_TXID_START 0x01
#define DM_8009P_TXID_END 0x04
#define DM_8009P_RXID_START 0x01
#define DM_8009P_RXID_END 0x04


/**
 * @brief 达妙电机状态
 *
 */
enum Enum_Motor_DM_Status
{
    Motor_DM_Status_DISABLE = 0,
    Motor_DM_Status_ENABLE,
};
/**
 * @brief 达妙电机控制方式
 *
 */
enum Enum_Motor_DM_Control_Method
{
    Motor_DM_Control_Method_NORMAL_MIT = 0,
    Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA,
    Motor_DM_Control_Method_NORMAL_OMEGA,
    Motor_DM_Control_Method_NORMAL_EMIT,
//    Motor_DM_Control_Method_1_TO_4_CURRENT,
//    Motor_DM_Control_Method_1_TO_4_OMEGA,
//    Motor_DM_Control_Method_1_TO_4_ANGLE,
};
typedef struct Motor_rev
{
	uint16_t id;
	uint16_t state; //读回的电机状态
    float Receive_pos; //读回来的位置
    float Receive_vel; //读回来的速度
    float Receive_torq; //读回来的扭矩
	float Now_pos;
	float Now_vel;
	float Now_torq;
	float Now_Tmos;
	float Now_Trotor;
    uint16_t last_pos; //上次的位置
    long long resv_time; //接受反馈的时间

}Motor_rev; //达妙电机的反馈参数结构体
/**
 * @brief 达妙电机常规源数据, MIT控制报文
 *
 */
typedef struct Motor_mit
{
    uint16_t motor_id;
    float target_pos;
    float target_vel;
    float kp;  //位置比例系数
    float kd;  //位置微分系数
    float target_torq;
}Motor_mit;  

class Class_Motor_DM_8009P
{
public:
    void Init(FDCAN_HandleTypeDef *hfdcan,uint8_t _CAN_Rx_ID,uint8_t _CAN_Tx_ID,Enum_Motor_DM_Control_Method _Motor_DM_Control_Method,float _Angle_Max,float _Omega_Max,float _Torque_Max,float _Current_Max);
    // void DM_8009P_Ctrl(FDCAN_HandleTypeDef *hfdcan,float _pos,float _vel,float _torq,float _kp,float _kd);
    void DM_8009P_Ctrl();
    void SetMotorData(float _pos,float _vel,float _torq,float _kp,float _kd);
    float GetAngle();
    float GetSpeed();
    float GetTor();
    void Enable();
    void Disable();
    void Save_Pos_Zero();
    void SetMotorT(float _T);
    void SetMotorPos(float _Pos);
    void TIM_Alive_PeriodElapsedCallback();
    Enum_Motor_DM_Status Get_Status();
    void DM_8009P_getInfo(FDCan_Export_Data_t RxMessage);
    Motor_rev DM_Rev;
private:
    uint16_t CAN_Rx_ID; 
    uint16_t CAN_Tx_ID;
    Enum_Motor_DM_Control_Method Motor_DM_Control_Method;
    float Angle_Max;
    float Omega_Max;
    float Torque_Max;
    float Current_Max;
    int16_t turnCount; //转过的圈数
    float totalpos; //累计位置
    FDCAN_HandleTypeDef *Can_DM_Motor;  //电机绑定的can总线
    Motor_mit DM_Mit;
    Enum_Motor_DM_Status Status = Motor_DM_Status_ENABLE;
    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
     // 当前时刻的电机接收flag
    uint32_t Flag = 0;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;
};

#endif