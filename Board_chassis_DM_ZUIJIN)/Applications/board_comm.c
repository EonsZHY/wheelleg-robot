
#include "board_comm.h"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
BoardComm board_comm;
/* Private function prototypes -----------------------------------------------*/

void BoardCommInit(BoardComm *_board_comm,FDCAN_HandleTypeDef* _phcan, uint16_t _id){
_board_comm->boardfdcan=_phcan;
	_board_comm->boardid=_id;
	_board_comm->rece_len=8;
	_board_comm->send_len=8;
	
	_board_comm->rece_.speed=0;
	_board_comm->rece_.w_speed=0;
	_board_comm->rece_.len=0;
	_board_comm->rece_.yaw_position=0;
	_board_comm->rece_.yaw_speed=0;

  _board_comm->rece_.jump_flag = 0;
	_board_comm->rece_.boardcomm_ready_flag=0;
	
	_board_comm->send_.yaw_real_position=0;
	_board_comm->send_.bullet_speed=0;
}

void BoardCommSend(BoardComm *_board_comm) {
	 _board_comm->senddata[0] = _board_comm->send_.yaw_real_position>>8;
   _board_comm->senddata[1] = _board_comm->send_.yaw_real_position;
   _board_comm->senddata[2] = 0;
   _board_comm->senddata[3] = 0;
   _board_comm->senddata[4] = _board_comm->send_.bullet_speed>>8;
   _board_comm->senddata[5] = _board_comm->send_.bullet_speed;
   _board_comm->senddata[6] = 0;
   _board_comm->senddata[7] = 0;
	 Can_Fun.fdcanx_send_data(_board_comm->boardfdcan,_board_comm->boardid,_board_comm->senddata, _board_comm->send_len);
}

void BoardCommReceive(BoardComm *_board_comm,FDCan_Export_Data_t RxMessage) {
		if( RxMessage.fdcan_RxHeader.Identifier==0x10f)
		{
		_board_comm->rece_.speed=-(int16_t)(RxMessage.FDCANx_Export_RxMessage[0]<<8|RxMessage.FDCANx_Export_RxMessage[1]);
		_board_comm->rece_.len=-(int16_t)(RxMessage.FDCANx_Export_RxMessage[2]<<8|RxMessage.FDCANx_Export_RxMessage[3]);
		_board_comm->rece_.w_speed=(int16_t)(RxMessage.FDCANx_Export_RxMessage[4]<<8|RxMessage.FDCANx_Export_RxMessage[5]);
		_board_comm->rece_.yaw_speed=(int16_t)(RxMessage.FDCANx_Export_RxMessage[6]<<8|RxMessage.FDCANx_Export_RxMessage[7]);
		_board_comm->rece_.boardcomm_ready_flag=1;
		}
				if( RxMessage.fdcan_RxHeader.Identifier==0x11f)
		{
		_board_comm->rece_.yaw_position=(int16_t)(RxMessage.FDCANx_Export_RxMessage[0]<<8|RxMessage.FDCANx_Export_RxMessage[1]);
		_board_comm->rece_.AutoAimFlag =0;
		_board_comm->rece_.jump_flag=0;
		_board_comm->rece_.boardcomm_ready_flag=1;
		}
}
	
	
	
	
	
	
	
	


