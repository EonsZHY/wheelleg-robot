#ifndef _FLY_IBUS_H_
#define _FLY_IBUS_H_

#include "main.h"
#include "usart.h"

/* User configuration */
#define IBUS_UART				(&huart10)
#define IBUS_UART_INSTANCE		(USART10)
#define IBUS_USER_CHANNELS		10			// Use 6 channels
/* User configuration */

#define IBUS_LENGTH				0x20	// 32 bytes
#define IBUS_COMMAND40			0x40	// Command to set servo or motor speed is always 0x40
#define IBUS_MAX_CHANNLES		14

void IBUS_INIT();
void IBUS_READ_CHANNEL(uint8_t user_channels);

#endif /* FLY_IBUS_H_ */

