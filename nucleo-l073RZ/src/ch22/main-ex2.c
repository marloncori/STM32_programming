/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include <cmsis_os.h>
#include <nucleo_hal_bsp.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart2;

void blinkThread(void const *argument);
void UARTThread(void const *argument);

volatile const int __attribute__((used)) uxTopUsedPriority = configMAX_PRIORITIES;

int main(void) {
  HAL_Init();

  Nucleo_BSP_Init();

  osThreadDef(blink, blinkThread, osPriorityNormal, 0, 100);
  osThreadCreate(osThread(blink), NULL);

  osThreadDef(uart, UARTThread, osPriorityAboveNormal, 0, 100);
  osThreadCreate(osThread(uart), NULL);

  osKernelStart();

  /* Infinite loop */
  while (1);
}

void blinkThread(void const *argument) {
  while(1) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(500);
  }
}

void UARTThread(void const *argument) {
  while(1) {
    HAL_UART_Transmit(&huart2, "UARTThread\r\n", strlen("UARTThread\r\n"), HAL_MAX_DELAY);
  }
}

#ifdef DEBUG

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName ) {
  asm("BKPT #0");
}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
