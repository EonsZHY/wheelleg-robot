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
#include "Chassis.h"
#include "Cloud.h"

// CAN���ĵı�ʶ�������ݳ���
#define CAN_ID_CHASSIS 0x10f // ����CAN���ĵ�������IDΪ0x10f
#define CAN_ID_GIMBAL  0x11f // ��̨����IDΪ0x11f
#define CAN_ID_KEYCOMMAND 0x22f // ͼ���������IDλ0x22f

/*����ģʽ 0Ϊ������1Ϊ��¼����Ϊmid����Ϊdown��*/
#define model_Normal 0
#define model_Record 1


#define Board2_FunGroundInit       \
    {                              \
        &Board2_getChassisInfo,    \
        &Board2_getGimbalInfo,     \
        &Board2_To_1,              \
        &BoardCommInit             \
    }

typedef struct {
    int16_t x_velocity;
    int16_t y_velocity;
    int16_t z_rotation_velocity;
	int16_t pitch_velocity;
	float  yaw_velocity;
	float len;
	uint8_t rece_len;
	uint8_t AutoAimFlag;         // ���鿪��
	uint8_t jump_flag;
} rece_pack;

typedef struct {
	int16_t yaw_realAngle;      //��������ϵ����̨yaw��Ļ�е�Ƕ�
	uint8_t send_len;
    float bullet_speed;
	FDCAN_HandleTypeDef *boardfdcan;
} send_pack;
// ����CAN���ĵĽṹ��
typedef struct {
		uint8_t shoot_state;
		uint8_t modelFlag;
		uint8_t shoot_Speed;
		uint8_t change_Flag;				//����
		uint8_t fric_Flag;					//Ħ����
		uint8_t tnndcolor;
		int16_t Gimbal_Chassis_Pitch_Angle;
		int8_t feipo_Flag;            //���¿���
		uint16_t Blood_Volume;         //???????
		uint8_t game_start;
		uint32_t controlid;
		rece_pack rece_;
		send_pack send_;
} ControlMessge;

extern ControlMessge ControlMes;
extern uint16_t Auto_Aim_Yaw;

typedef struct
{
	void (*Board2_getChassisInfo)(FDCan_Export_Data_t RxMessage);
	void (*Board2_getGimbalInfo)(FDCan_Export_Data_t RxMessage);
	void (*Board2_To_1)();
	void (*BoardCommInit)(FDCAN_HandleTypeDef* _phcan);
}Board2_FUN_t;

/********ȫ�ֱ�������********/
extern Board2_FUN_t Board2_FUN;
extern ControlMessge ControlMes;

/********��������********/
void Board2_To_1();
void Board2_getChassisInfo(FDCan_Export_Data_t RxMessage);
void Board2_getGimbalInfo(FDCan_Export_Data_t RxMessage);
void BoardCommInit(FDCAN_HandleTypeDef* _phcan);
#endif
