/**
 *******************************************************************************
 * @file      : board_comm.h
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_COMM_H_
#define __BOARD_COMM_H_

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
#include "Bsp_fdcan.h"
#include "stdlib.h"
#include "string.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

typedef struct {
  int16_t speed;
  int16_t w_speed;
	int16_t len;
	int16_t yaw_speed;
	int16_t yaw_position;
	uint8_t AutoAimFlag ;
  uint8_t jump_flag ;
	uint8_t boardcomm_ready_flag;
} rece_pack;

typedef struct {
  int16_t yaw_real_position;
  int16_t bullet_speed;
} send_pack;


typedef struct {
	send_pack send_;
  rece_pack rece_;
 	uint8_t rece_len;
	uint8_t send_len;
  FDCAN_HandleTypeDef *boardfdcan;
	uint32_t boardid;
	uint8_t senddata[8];
}BoardComm;

/* Exported variables --------------------------------------------------------*/
extern BoardComm board_comm;
//extern void BoardCommSend(BoardComm *_board_comm);
//extern void BoardCommReceive(BoardComm *_board_comm,FDCan_Export_Data_t RxMessage);
//extern void BoardCommInit(BoardComm *_board_comm,FDCAN_HandleTypeDef* _phcan, uint16_t _id);	

/* Exported function prototypes ----------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

// 声明 C 函数(由于BSP_fdcan.c中使用了C++中的类，所以将该文件以C++的方式编译，修改此处使该文件能够正确识别到c函数)
void BoardCommInit(BoardComm *_board_comm, FDCAN_HandleTypeDef* _phcan, uint16_t _id);
void BoardCommSend(BoardComm *_board_comm);
void BoardCommReceive(BoardComm *_board_comm, FDCan_Export_Data_t RxMessage);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_COMM_H_ */
