/**
 * @file BSP_fdcan.c
 * @author Cyx(1686143358@qq.com)
 * @brief Init of FDCAN
 * @version 0.1
 * @date 2024-10-19
 *
 * @copyright 
 */

#include "BSP_fdcan.h"
#include "M6020_Motor.h"
#include "board_comm.h"
#include "DM_8009P.h"
#include "M3508.h"
#include "Chassis.h"
//#include "Extern_Handles.h"




void can_bsp_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan);

Can_Fun_t Can_Fun = Can_FunGroundInit;
#undef Can_FunGroundInit



/**
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
**/
static void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
	fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
	fdcan_filter.FilterID1 = 0x00;                               
	fdcan_filter.FilterID2 = 0x00;                               
	HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter); 		 				  //配置
	HAL_FDCAN_ConfigFilter(&hfdcan2,&fdcan_filter); 		 				  //配置
	HAL_FDCAN_ConfigFilter(&hfdcan3,&fdcan_filter); 
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
//	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
}


/**
* @brief:      	can_bsp_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN使能
**/
void can_bsp_init(void)
{
	can_filter_init();
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);		//开启中断
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_Start(&hfdcan3);
}

/**
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
**/
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
	FDCAN_TxHeaderTypeDef TxHeader;
	
  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;																// 标准ID 
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;														// 数据帧 
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;																		// 发送数据长度 
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;										// 设置错误状态指示 								
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;															// 不开启可变波特率 
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;															// 普通CAN格式 
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;										// 用于发送事件FIFO控制, 不存储 
  TxHeader.MessageMarker = 0x00; 			// 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF                
    
  if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data)!=HAL_OK) 
		return 1;//发送
	return 0;	
}

/**
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
**/
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan)
{	
		if(hfdcan==&hfdcan1)  //CAN1上挂载关节电机
{
	FDCan_Export_Data_t FDCan_Export_Data;
  if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &FDCan_Export_Data.fdcan_RxHeader, FDCan_Export_Data.FDCANx_Export_RxMessage)==HAL_OK)
	{
		//判断是否是关节电机返回的数据
		if( FDCan_Export_Data.fdcan_RxHeader.Identifier >= DM_8009P_RXID_START  && FDCan_Export_Data.fdcan_RxHeader.Identifier <= DM_8009P_RXID_END  )
		{
			chassis.lf_joint_.DM_8009P_getInfo(FDCan_Export_Data);
			chassis.lb_joint_.DM_8009P_getInfo(FDCan_Export_Data);
			chassis.rf_joint_.DM_8009P_getInfo(FDCan_Export_Data);
			chassis.rb_joint_.DM_8009P_getInfo(FDCan_Export_Data);
			return FDCan_Export_Data.fdcan_RxHeader.DataLength;//接收数据
		}
		
	}

}
	if(hfdcan==&hfdcan2) //can2用于板间通信
{
	FDCan_Export_Data_t FDCan_Export_Data;
  if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &FDCan_Export_Data.fdcan_RxHeader, FDCan_Export_Data.FDCANx_Export_RxMessage)==HAL_OK)
	{
		if( FDCan_Export_Data.fdcan_RxHeader.Identifier ==0x10f||FDCan_Export_Data.fdcan_RxHeader.Identifier ==0x11f )
			BoardCommReceive(&board_comm,FDCan_Export_Data);
		return FDCan_Export_Data.fdcan_RxHeader.DataLength;//接收数据
				
	}
}
	if(hfdcan==&hfdcan3) //can3用于控制轮毂电机，yaw轴电机和拨盘电机
	{
		FDCan_Export_Data_t FDCan_Export_Data;
		if( FDCan_Export_Data.fdcan_RxHeader.Identifier==M6020_READID )
		{
			M6020_Fun.M6020_getInfo(FDCan_Export_Data,&M6020s_Yaw);
		}
		if(FDCan_Export_Data.fdcan_RxHeader.Identifier >= M3508_READID_START && FDCan_Export_Data.fdcan_RxHeader.Identifier <= M3508_READID_END)
		{
			M3508_FUN.M3508_getInfo(FDCan_Export_Data);
		}
		return FDCan_Export_Data.fdcan_RxHeader.DataLength;//接收数据

	}
	return 0;
}

/**
* @brief:      	fdcan_rx_callback(void)
* @param:       void
* @retval:     	void
* @details:    	供用户调用的接收弱函数
**/
//uint8_t rx_data1[8] = {0};
//uint16_t reid;
static void fdcan1_rx_callback(void)
{
	Can_Fun.fdcanx_receive(&hfdcan1);
}

static  void fdcan2_rx_callback(void)
{
	Can_Fun.fdcanx_receive(&hfdcan2);
}
static  void fdcan3_rx_callback(void)
{
	Can_Fun.fdcanx_receive(&hfdcan3);
}
/**
* @brief:      	HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
* @param:       hfdcan；FDCAN句柄
* @param:       RxFifo0ITs：中断标志位
* @retval:     	void
* @details:    	HAL库的FDCAN中断回调函数
**/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
		if(hfdcan == &hfdcan1)
		{
			fdcan1_rx_callback();
		}
		if(hfdcan == &hfdcan2)
		{
			fdcan2_rx_callback();
		}
		if(hfdcan == &hfdcan3)
		{
			fdcan3_rx_callback();
		}
	}
}



