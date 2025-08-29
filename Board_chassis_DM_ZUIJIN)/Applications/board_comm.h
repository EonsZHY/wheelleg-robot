/**
 * @file BSP_BoardCommunication.h
 * @author lxr(784457420@qq.com)
 * @brief 
 * @version 1.0
 * @date 2023-9-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef BSP_BOARDCOMMUNICATION_H 
#define	BSP_BOARDCOMMUNICATION_H

#include "main.h"
#include "BSP_fdcan.h"
#include "Cloud.h"
#include "N100.h"


// CAN报文的标识符和数据长度
#define CAN_ID_CHASSIS 0x10f // 假设CAN报文底盘数据ID为0x10f
#define CAN_ID_GIMBAL  0x11f // 云台数据ID为0x11f
#define CAN_ID_KEYCOMMAND 0x22f // 图传鼠标数据ID位0x22f

/*操作模式 0为正常，1为检录（左为mid，右为down）*/
#define model_Normal 0
#define model_Record 1


#define Board2_FunGroundInit   \
	{                          \
		&Board2_getChassisInfo,       \
		&Board2_getGimbalInfo,		  \
		&Board2_To_1,                 \
	}


// 定义CAN报文的结构体
typedef struct {
    int16_t x_velocity;
    int16_t y_velocity;
    int16_t z_rotation_velocity;
	  int16_t pitch_velocity;
		float   yaw_velocity;
		uint8_t shoot_state;
		int16_t yaw_realAngle;      //世界坐标系下云台yaw轴的机械角度
		uint8_t modelFlag;
		uint8_t shoot_Speed;
		uint8_t AutoAimFlag;         // 自瞄开关
		uint8_t change_Flag;				//变速
		uint8_t fric_Flag;					//摩擦轮
		uint8_t tnndcolor;
		int16_t Gimbal_Chassis_Pitch_Angle;
		int8_t feipo_Flag;            //飞坡开关
		uint16_t Blood_Volume;         //???????
		uint8_t game_start;
		uint32_t controlid;
		uint8_t rece_len;
	    uint8_t send_len;
		FDCAN_HandleTypeDef *boardfdcan
} ControlMessge;

extern ControlMessge ControlMes;
extern uint16_t Auto_Aim_Yaw;

typedef struct
{
	void (*Board2_getChassisInfo)(Can_Export_Data_t RxMessage);
	void (*Board2_getGimbalInfo)(Can_Export_Data_t RxMessage);
	void (*Board2_To_1)(void);
}Board2_FUN_t;

/********全局变量声明********/
extern Board2_FUN_t Board2_FUN;
extern ControlMessge ControlMes;

/********函数声明********/
void Board2_To_1(void);
void Board2_getChassisInfo(Can_Export_Data_t RxMessage);
void Board2_getGimbalInfo(Can_Export_Data_t RxMessage);

#endif
