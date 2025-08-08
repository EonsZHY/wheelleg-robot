
#include "M6020_Motor.h"
#include "BSP_fdcan.h"
//ֱ��������Ӧ�ĵ���Ľṹ����������飬ֱ�۱��ں��ڵ��Թ۲�����ʹ�á�
M6020s_t M6020s_Yaw;                                    //IDΪ1
uint8_t M6020_Data_Tx[8];

void M6020_Enable(void);
void M6020_setVoltage(void);
void M6020_getInfo(FDCan_Export_Data_t RxMessage,M6020s_t *m6020);
void M6020_setTargetAngle(M6020s_t *M6020, int32_t angle);
void M6020_Reset(M6020s_t *m6020);

M6020_Fun_t M6020_Fun = M6020_FunGroundInit;
#undef M6020_FunGroundInit


/**
  * @brief  M6020pid����
  * @param  None
  * @retval None
  */
void M6020_Enable(void)
{	
	Position_PIDInit(&M6020s_Yaw.Aim_position_PID, 1300.0f, 70.0f, 900.0f, 300.0f, 30000, 10000);
	Position_PIDInit(&M6020s_Yaw.position_PID, 5.0f, 0.0f, 3.0f, 0.0f, 30000, 10000);
	Position_PIDInit(&M6020s_Yaw.velocity_PID, 50.0f, 0.0f, 1.5f, 0.0f, 30000, 10000);
}


/**
  * @brief  ����M6020�����ѹ��id��Ϊ1~4��
  * @param  uqx (x:1~4) ��Ӧid�ŵ���ĵ�ѹֵ����Χ -30000~0~30000
  * @retval None
  */

 void M6020_setVoltage(void)
 {

 	M6020_Data_Tx[0] = M6020s_Yaw.outCurrent >> 8;
 	M6020_Data_Tx[1] = M6020s_Yaw.outCurrent;
 	M6020_Data_Tx[2] = 0;
 	M6020_Data_Tx[3] = 0;
 	M6020_Data_Tx[4] = 0;
 	M6020_Data_Tx[5] = 0;
 	M6020_Data_Tx[6] = 0;
 	M6020_Data_Tx[7] = 0;
	Can_Fun.fdcanx_send_data(&hfdcan3,M6020_SENDID, M6020_Data_Tx, 8);
 }

/**
  * @brief  ��CAN�����л�ȡM6020�����Ϣ
  * @param  RxMessage 	CAN���Ľ��սṹ��
  * @retval None
  */

void M6020_getInfo(FDCan_Export_Data_t RxMessage,M6020s_t *m6020)
{


    //������ݣ����ݸ�ʽ���C620���˵����P33
   m6020->lastAngle =m6020->realAngle;
    m6020->realAngle = (uint16_t)(RxMessage.FDCANx_Export_RxMessage[0] << 8 | RxMessage.FDCANx_Export_RxMessage[1]);
    m6020->realSpeed = (int16_t)(RxMessage.FDCANx_Export_RxMessage[2] << 8 | RxMessage.FDCANx_Export_RxMessage[3]);
    m6020->realCurrent = (int16_t)(RxMessage.FDCANx_Export_RxMessage[4] << 8 | RxMessage.FDCANx_Export_RxMessage[5]);
    m6020->temperture = RxMessage.FDCANx_Export_RxMessage[6];

    if (m6020->realAngle - m6020->lastAngle < -6500)
    {
       m6020->turnCount++;
    }

    if (m6020->lastAngle -m6020->realAngle < -6500)
    {
        m6020->turnCount--;
    }

  m6020->totalAngle =m6020->realAngle + (8192 * m6020->turnCount);
    //֡��ͳ�ƣ����ݸ��±�־λ
    m6020->InfoUpdateFrame++;
   m6020->InfoUpdateFlag = 1;

}

/*
*@brief  �趨M6020�����Ŀ��Ƕ�
* @param  motorName 	������� @ref M6623Name_e
*			angle		����ֵ����Χ 0~8191 ��������0��8191�ᵼ�µ���񵴣�Ҫ�����޷�
* @retval None
* */
void M6020_setTargetAngle(M6020s_t *M6020, int32_t angle)
{
    M6020->targetAngle = angle;
}

/*************************************
* Method:    M6020_OverflowReset
* Returns:   void
* ˵�������˴˺����Խ��totalAngle ����������⡣
************************************/
void M6020_Reset(M6020s_t *m6020)
{
    //������ݣ����ݸ�ʽ���C620���˵����P33
    m6020->lastAngle = m6020->realAngle;
    m6020->totalAngle = m6020->realAngle;
    m6020->turnCount = 0;
}

