#include "main.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* -----DEFINES----- */
UART_HandleTypeDef huart1;
#define MAX_STRING_PARSE_SIZE 200

/* -----STRUCTS | ENUMS | UNIONS----- */
typedef enum {
	eStringIsTooLarge,
	/*Insert additional error conditions*/
} ErrorValues_t;

typedef struct {
	ErrorValues_t errval;
	char* ErrorStr;
} ErrorTable_t;

ErrorTable_t error_string_table[3] = {
	{eStringIsTooLarge	,	"STRING IS TOO LARGE1\r" },
	/*Insert additional values based on error conditions*/
};

/* -----FUNCTION PROTOTYPES----- */
static void MX_USART1_UART_Init(void);
void 		init_uart (bool var);
void 		USART1_String_Parse(char* data);
void 		SystemClock_Config(void);
static void MX_GPIO_Init(void);
void 		init_GPIO (bool var);

/* -----FUNCTIONS----- */
static void MX_USART1_UART_Init(void) {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);
}
//void SystemClock_Config(void) {
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
//    Error_Handler();
//  }
//
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
//    Error_Handler();
//  }
//}
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void init_uart (bool var) {
	if (var == true) {
		MX_USART1_UART_Init();
	}
}

void init_GPIO (bool var) {
	if (var == true) {
		MX_GPIO_Init();
	}
}

void USART1_String_Parse (char* data) {
	const char* carriage_return_append = "\r";
	char ptr[MAX_STRING_PARSE_SIZE];
	memset(ptr, NULL, strlen(ptr));
	if (strlen(data) > MAX_STRING_PARSE_SIZE) {
		HAL_UART_Transmit(&huart1, error_string_table[eStringIsTooLarge].ErrorStr, strlen(error_string_table[eStringIsTooLarge].ErrorStr), 15);
	} else {
		strncpy(ptr, data, strlen(data));
		strcat(ptr, carriage_return_append);
		HAL_UART_Transmit(&huart1, ptr, strlen(data)+strlen(carriage_return_append), 100);
	}
}
