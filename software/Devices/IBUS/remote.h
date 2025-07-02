/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REMOTE_H_
#define __REMOTE_H_

#ifdef __cplusplus

#include "bsp_uart.h"
#define IBUS_USER_CHANNELS		10			// Use 6 channels
#define IBUS_LENGTH				0x20	// 32 bytes
#define IBUS_COMMAND40			0x40	// Command to set servo or motor speed is always 0x40
#define IBUS_MAX_CHANNLES		14

class Remote_t
{
   public:
   void Init(UART_HandleTypeDef *_phuart);
   UartInstance *premote_instance;
   void SbusToRc(uint8_t *_pdata);
   private:
    uint16_t channel[IBUS_USER_CHANNELS] = {0};
    uint16_t checksum_cal, checksum_ibus;
};

extern Remote_t remote;

#endif

#ifdef __cplusplus
extern "C" {
#endif
#include "usart.h"
#ifdef __cplusplus
}
#endif

#endif /* __REMOTE_H_ */
