#include "main.h"
#include "core_init.h"
#include <stdbool.h>

uint8_t data[100] = "zygelis";
bool val = true;

int main(void) {
  HAL_Init();
  SystemClock_Config();
  init_GPIO (val);
  init_uart (val);

  while (1) {
	  USART1_String_Parse(data);
	  HAL_Delay(1000);
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {

  }
}
