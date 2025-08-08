/**
 * @file BSP_fdcan.h
 * @author Cyx(1686143358@qq.com)
 * @brief Init of FDCAN
 * @version 0.1
 * @date 2024-10-19
 *
 * @copyright 
 */

#ifndef _BSP_FDCAN
#define _BSP_FDCAN

#include "fdcan.h"
#include "cmsis_os.h"
#include "queue.h"
//#include "typedef.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>






#define Can_FunGroundInit           \
    {                               \
        &can_bsp_init,              \
        &fdcanx_send_data,          \
        &fdcanx_receive,            \
    }

typedef struct
{
	void (*can_bsp_init)(void);
	uint8_t (*fdcanx_send_data)(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
	uint8_t (*fdcanx_receive)(FDCAN_HandleTypeDef *hfdcan);
} Can_Fun_t;



typedef struct
{
    FDCAN_RxHeaderTypeDef fdcan_RxHeader;
    uint8_t FDCANx_Export_RxMessage[8];
} FDCan_Export_Data_t;

extern Can_Fun_t Can_Fun;
#endif /*__BSP_FDCAN*/
