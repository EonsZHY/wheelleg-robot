/**
 * @file M2006.h
 * @author ZHY
 * @brief 
 * @version 0.1
 * @date 2025-08-30
 * 
 * @copyright 
 * 
 */


#ifndef __M2006_H
#define __M2006_H


#include "fdcan.h"
#include "BSP_fdcan.h"
#include "tpid.h"
#include "string.h"
#include <stdbool.h>
#include <stdint.h>
#include "arm_math.h"
#include "dji_motor.h"

#define M2006_READID_START 0x201
#define M2006_READID_END 0x204
#define M2006_SENDID 0x200   //控制1-4的电机ID
#define M2006_MaxOutput 10000 //发送给电机的最大控制值
#define M2006_LOADANGLE 36864
#define M2006_ReductionRatio 36 //电机减速比
//#define M2006_LOADANGLE		42125			/* 电机拨一个弹需要转的角度数  6*8191 （7孔拨弹）*/

//#define M2006_LOADCIRCLE	5			/* 电机拨一个弹需要转的圈数 */
//#define M2006_LOADSPEED		1800		/* 电机拨弹时的转速 */
#define M2006_FIRSTANGLE 3800 /* 电机初始位置 */


/**
 * @brief 大疆2006电机, 自带扭矩环, 单片机控制输出扭矩
 *
 */

 #ifdef __cplusplus
class Class_M2006
{
public:
    static FDCAN_HandleTypeDef *Can_Motor;  //电机绑定的can总线
    Enum_Control_Method Get_Control_Method();
    void Init(Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method = Control_Method_OMEGA, float __Gearbox_Rate = 3591.0f / 187.0f, float __Torque_Max = 30000.0f);
    static void SetCurrent(void);
    void FDCAN_RxCpltCallback(FDCan_Export_Data_t RxMessage);
    void Output();
    void Set_Out(int16_t __Out);
    int16_t realTorque; //读回来的实际转矩
private:
    //发送缓存区
    uint8_t *CAN_Tx_Data;
    uint16_t realAngle; //读回来的机械角度
    int16_t realSpeed;  //读回来的速度
    
    int16_t targetSpeed; //目标速度
    int32_t targetAngle; //目标角度

    uint16_t lastAngle; //上次的角度
    int32_t totalAngle; //累积总共角度
    int16_t turnCount;  //转过的圈数

    int16_t outCurrent;      //输出电流
    int16_t inneroutCurrent; //输出电流

    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
	uint8_t temperture;
	//收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_CAN_Motor_ID CAN_ID;
	   //电机控制方式
    Enum_Control_Method Control_Method = Control_Method_ANGLE;
		    //减速比, 默认带减速箱
    float Gearbox_Rate = 3591.0f / 187.0f;
	float Current_Max;


};

#endif
#endif