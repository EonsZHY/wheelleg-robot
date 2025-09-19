/**
 * @file BSP_BoardCommunication.c
 * @author lxr(784457420@qq.com)/ZS(2729511164@qq.com)
 * @brief
 * @version 1.0
 * @date 2023-9-15
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "board_comm.h"

/********变量定义********/

Board2_FUN_t Board2_FUN = Board2_FunGroundInit;
ControlMessge ControlMes;

/**
  * @brief 初始化板间通讯
  * @param None
  * @retval None
  */

void BoardCommInit(FDCAN_HandleTypeDef* _phcan)
{
	ControlMes.send_ = {0};
	ControlMes.rece_ = {0};
	ControlMes.send_.boardfdcan=_phcan;
	ControlMes.send_.send_len=8;
	ControlMes.rece_.rece_len=8;
	
	
}

/**
  * @brief 按照报文规则生成数据并发送
  * @param None
  * @retval None
  */
void Board2_To_1()
{
	int16_t bullet_speed;
	uint8_t IT_keycommand[8]={0};
	uint8_t data[8] = {0};

	// 分别是yaw轴角度，Saber的pitch轴角度，上一发的弹速，装甲颜色
	data[0] = ControlMes.send_.yaw_realAngle >> 8;
	data[1] = ControlMes.send_.yaw_realAngle;	
	data[2] = ControlMes.Blood_Volume >> 8;
	data[3] = ControlMes.Blood_Volume;
	//bullet_speed = (int16_t)(g_referee.shoot_data_.initial_speed * 1000);
	data[4] = bullet_speed >> 8;
	data[5] = bullet_speed;
  data[6] |= (uint8_t)(ControlMes.tnndcolor & 0x01) <<0;
	data[7] |= (uint8_t)(ControlMes.game_start & 0x01) <<1;
	
	//图传键鼠映射，分别是鼠标x轴、y轴、z轴（z轴是滚轮）、左键、右键，此时只传了鼠标的信息
	//云台只需要鼠标操控，因此在上板接收时需要扩展IT_keycommand[8]至IT_keycommand[12]（后四位直接赋0），
	//这样才能与遥控器键鼠映射的格式一致
	//	IT_keycommand[0] = ext_robot_keycommand.data.mouse_x >> 8;
	//	IT_keycommand[1] = ext_robot_keycommand.data.mouse_x;
	//	IT_keycommand[2] = ext_robot_keycommand.data.mouse_y >> 8;
	//	IT_keycommand[3] = ext_robot_keycommand.data.mouse_y;
	//	IT_keycommand[4] = ext_robot_keycommand.data.mouse_z >> 8;
	//	IT_keycommand[5] = ext_robot_keycommand.data.mouse_z;
	//	IT_keycommand[6] = ext_robot_keycommand.data.left_button_down;
	//	IT_keycommand[7] = ext_robot_keycommand.data.right_button_down;
	
  //数据发送
   Can_Fun.fdcanx_send_data(ControlMes.send_.boardfdcan,CAN_ID_CHASSIS,data, ControlMes.send_.send_len);
  //Can_Fun.CAN_SendData(CAN_SendHandle, &hcan2, CAN_ID_STD, CAN_ID_KEYCOMMAND, IT_keycommand);
}

/**
  * @brief 解析CAN数据，同时将结果直接赋值给底盘
  * @param RxMessage 接收到的数据
  * @retval None
  */
void Board2_getChassisInfo(FDCan_Export_Data_t RxMessage)
{
    float vx = (int16_t)(RxMessage.FDCANx_Export_RxMessage[0] << 8 | RxMessage.FDCANx_Export_RxMessage[1]);
    float vy = (int16_t)(RxMessage.FDCANx_Export_RxMessage[2] << 8 | RxMessage.FDCANx_Export_RxMessage[3]);
    float vw = (int16_t)(RxMessage.FDCANx_Export_RxMessage[4] << 8 | RxMessage.FDCANx_Export_RxMessage[5]);
    ControlMes.rece_.yaw_velocity = (int16_t)(RxMessage.FDCANx_Export_RxMessage[6] << 8 | RxMessage.FDCANx_Export_RxMessage[7]);
    //注意cloud角度还未更新，后续需要加上

    chassis.speed_to_cloud.vx = vx; //左手上下
    chassis.speed_to_cloud.vy = vy; //左手左右
    chassis.speed_to_cloud.wz = -1 * vw / 200; //滑轮
		if(!ControlMes.rece_.AutoAimFlag )
  		{
			Cloud.Target_Yaw += -1 * ControlMes.rece_.yaw_velocity * 0.06f; // 右手左右
		}
}

void Board2_getGimbalInfo(FDCan_Export_Data_t RxMessage)
{
		static float AutoAim_Offset = 0;
    float yaw_position 			= (int16_t)(RxMessage.FDCANx_Export_RxMessage[0] << 8 | RxMessage.FDCANx_Export_RxMessage[1]);
 		ControlMes.shoot_Speed 	= (uint8_t)RxMessage.FDCANx_Export_RxMessage[2];
	  ControlMes.shoot_Speed/=2;
		ControlMes.fric_Flag 		= (uint8_t)(RxMessage.FDCANx_Export_RxMessage[3]>>0)&0x01;
		ControlMes.rece_.AutoAimFlag 	= (uint8_t)(RxMessage.FDCANx_Export_RxMessage[3]>>1)&0x01;
		ControlMes.change_Flag 	= (uint8_t)(RxMessage.FDCANx_Export_RxMessage[3]>>2)&0x01;
		ControlMes.modelFlag 		= (uint8_t)(RxMessage.FDCANx_Export_RxMessage[3]>>3)&0x01;
		Cloud.Dial_2006.realTorque    = (uint16_t)(RxMessage.FDCANx_Export_RxMessage[4] << 8 | RxMessage.FDCANx_Export_RxMessage[5]);

		if(ControlMes.rece_.AutoAimFlag == 1 )
		{
			if(yaw_position == 0.0f) 
			{
				yaw_position = Cloud.Target_Yaw;
			}
			AutoAim_Offset = -1 * ControlMes.rece_.yaw_velocity * 0.03f; // 右手左右
			Cloud.Target_Yaw = yaw_position + AutoAim_Offset; // 此处的值应与上位机传来的数据相同
		}
		else
		{
			AutoAim_Offset = 0;
		}
		

	
}
