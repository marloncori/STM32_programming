/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include <cmsis_os.h>
#include <nucleo_hal_bsp.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart2;

void blinkThread(void const *argument);

int main(void) {
  osThreadId blinkTID;

  HAL_Init();

  Nucleo_BSP_Init();

  osThreadDef(blink, blinkThread, osPriorityNormal, 0, 100);
  blinkTID = osThreadCreate(osThread(blink), NULL);

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

#ifdef DEBUG

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName ) {
  asm("BKPT #0");
}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
