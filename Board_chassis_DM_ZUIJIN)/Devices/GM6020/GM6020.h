/**
 * @file GM6020.h
 * @author ZHY
 * @brief 
 * @version 0.1
 * @date 2025-08-29
 * 
 * @copyright 
 * 
 */

#ifndef __GM6020_H
#define __GM6020_H

#include "fdcan.h"
#include "BSP_fdcan.h"
#include "tpid.h"
#include "string.h"
#include <stdbool.h>
#include <stdint.h>
#include "arm_math.h"
#include "dji_motor.h"
#include "tpid.h"

#define M6020_READID_START 0x205 //当ID为1时的报文ID
#define M6020_SENDID 0x1FF //1~4的电机，0x2FF为5~7

#define M6020_MaxOutput 30000 //发送给电机的最大控制值
#define M6020_mAngle 8191     //6020的机械角度最大值0~8191。MachineAngle

#define M6020_mAngleRatio 22.7527f //机械角度与真实角度的比率

// 大疆电机CAN通信发送缓冲区
extern uint8_t CAN3_0x1ff_Tx_Data[8];
extern uint8_t CAN3_0x200_Tx_Data[8];
extern uint8_t CAN3_0x2ff_Tx_Data[8];

extern uint8_t CAN_Supercap_Tx_Data[8];
/**
 * @brief 大疆6020电机, 自带扭矩环, 单片机控制输出扭矩
 *
 */
class Class_GM6020
{
public:
    static FDCAN_HandleTypeDef *Can_Motor;  //电机绑定的can总线
    Enum_Control_Method Get_Control_Method();
    void Init(Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method = Control_Method_OMEGA, float __Gearbox_Rate = 3591.0f / 187.0f, float __Torque_Max = 30000.0f);
    void PID_Init(void);  //M6020pid初始化
    static void SetVoltage(void);
    void FDCAN_RxCpltCallback(FDCan_Export_Data_t RxMessage);
    void SetTargetAngle(int32_t angle);
	float GetTargetAngle(){return targetAngle;}
    void Output();
    void Set_Out(int16_t __Out);
	void SetAutoAimFlag(uint8_t Flag){AutoAimFlag = Flag;}
	void Reset(void);
	float targetSpeed; //目标速度
    int32_t targetAngle; //目标角度
	uint16_t realAngle;  //读回来的机械角度
	int16_t outCurrent; //输出电流
	int32_t realSpeed;   //读回来的速度
	struct Struct_PID_Manage_Object position_PID;
	struct Struct_PID_Manage_Object velocity_PID;	
	struct Struct_PID_Manage_Object Aim_position_PID;
private:
    //发送缓存区
    uint8_t *CAN_Tx_Data;
    int16_t realCurrent; //读回来的实际转矩电流
    uint8_t temperture;  //读回来的电机温度
    uint16_t lastAngle;  //上次的角度
    int16_t turnCount;   //转过的圈数
    float totalAngle;  //累积总共角度

    

    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
	uint8_t AutoAimFlag;
    //最大电压
    float Voltage_Max = 30000;
    //收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_CAN_Motor_ID CAN_ID;
	//输出量
    float Out = 0.0f;
    //一圈编码器刻度
    uint16_t Encoder_Num_Per_Round = 8192;
	   //电机控制方式
    Enum_Control_Method Control_Method = Control_Method_ANGLE;
	    //减速比, 默认带减速箱
    float Gearbox_Rate = 3591.0f / 187.0f;

    
};



#endif /* __GM6020_H */
