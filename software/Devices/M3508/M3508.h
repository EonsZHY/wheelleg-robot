/**
 * @file M3508_Motor.h
 * @author ZHY
 * @brief 
 * @version 0.1
 * @date 2025-5-20
 * 
 * @copyright 
 * 
 */

#ifndef __M3508_MOTOR_H
#define __M3508_MOTOR_H
#include "fdcan.h"
#include "BSP_fdcan.h"
#include "tpid.h"
#include "string.h"
#include <stdbool.h>
#include <stdint.h>

/* 记录M3508各个电机ID
*/
#define M3508_READID_START 0x201
#define M3508_READID_END 0x204
#define M3508_SENDID_Chassis 0x200   //控制轮毂电机
#define M3508_SENDID_Fric_Dial 0x1FF //控制摩擦轮和拨盘电机
#define M3508_MaxOutput 16384        //发送给电机的最大控制值
#define M3508_CurrentRatio 819.2f    //16384/20A = 819.2->1A
#define M3508_ReductionRatio 17.0642f //3508电机减速比
#define KT 2.5f  //2.5A/Nm,根据C620电调得到
#define M3508_CURRENT_MAX_A  20.0f
#define M3508_CURRENT_RAW_MAX 16384.0f
#define gear_ratio 19.2f   //减速比
typedef enum
{
	Wheel_L = 0,
	Wheel_R,
	TotalNum,
}M3508_MotorName;

typedef struct
{   float targetTorqueI;	//目标转矩得到的目标电流
	int16_t receiveTorqueI; //读回来的转矩电流
	int16_t receiveSpeed;    //读回速度
	  float realTorqueI;		//实际转矩电流
		float targetTorque;//目标转矩
    float realSpeed;   	//实际速度
    float targetCurrent;		//经过PID计算的目标电流
	int16_t sendCurrent;  //输出电流
    uint16_t circleAngle; //读回位置
    uint16_t lastAngle;   //上次的角度
    int32_t totalAngle;   //累积总共角度
	uint8_t feedbackSeq;//读回反馈时间间隔
    uint8_t temperture;  //读回来的电机温度
    int16_t turnCount;    //转过的圈数
    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
	struct Struct_PID_Manage_Object TPID;
} M3508_t;


typedef struct {
    void (*M3508_setCurrent)(void);
    void (*M3508_getInfo)(FDCan_Export_Data_t RxMessage);
    void (*M3508_SetTor)(void);
    void (*M3508_Init)(void);
} M3508_FUN_t;

// ✅ 注意：结构体初始化宏要用花括号 {} 括起来
#define M3508_FunGroundInit \
{                            \
    M3508_setCurrent,        \
    M3508_getInfo,           \
    M3508_SetTor,            \
    M3508_Init,              \
}

extern M3508_FUN_t M3508_FUN;


// 外部声明
extern M3508_t M3508_Array[2];
extern M3508_FUN_t M3508_FUN;

#endif