#ifndef INC_CORE_INIT_H_
#define INC_CORE_INIT_H_

#include <stdbool.h>

static void MX_USART1_UART_Init(void);
void 		init_uart (bool var);
void 		USART1_String_Parse(char* data);
void 		SystemClock_Config(void);
static void MX_GPIO_Init(void);
void 		init_GPIO (bool var);


#endif /* INC_CORE_INIT_H_ */
