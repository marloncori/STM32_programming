/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include <nucleo_hal_bsp.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim6;
volatile uint8_t convCompleted = 0;

/* Private function prototypes -----------------------------------------------*/
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);

int main(void) {
  char msg[20];
  uint16_t rawValues[3];
  float temp;

  HAL_Init();
  Nucleo_BSP_Init();

  /* Initialize all configured peripherals */
  MX_TIM6_Init();
  MX_ADC1_Init();

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  HAL_TIM_Base_Start(&htim6);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)rawValues, 1);

  while(1) {
    while(!convCompleted);

     for(uint8_t i = 0; i < 1; i++) {
      temp = ((float)rawValues[i]) / 4095 * 3300;
      temp = ((temp - 670.0) / 1.70) + 130;

      sprintf(msg, "rawValue %d: %hu\r\n", i, rawValues[i]);
      HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

      sprintf(msg, "Temperature %d: %f\r\n",i,  temp);
      HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
    }
    convCompleted = 0;
  }
}

/* ADC1 init function */
void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig;

  /* Enable ADC peripheral */
  __HAL_RCC_ADC1_CLK_ENABLE();

  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  hadc1.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONV_T6_TRGO;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerFrequencyMode = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

void MX_TIM6_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  __HAL_RCC_TIM6_CLK_ENABLE();

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 31999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 3999;
  htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim6);

  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_IT_UPDATE);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim6, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) {
  if(hadc->Instance==ADC1)  {
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    /* Peripheral DMA init*/
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_adc1);

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc1);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  convCompleted = 1;
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
  asm("BKPT #0");
}


#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
