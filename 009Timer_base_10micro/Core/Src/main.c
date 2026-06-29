/*
 * main.c
 *
 *  Created on: Jun 22, 2026
 *      Author: SANJU
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClockConfig(uint8_t clock_freq);
void TIMER6_Init(void);
void GPIO_Init(void);
void Error_handler(void);


/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htimer6;

int main () {

    HAL_Init();
    SystemClockConfig(SYS_CLOCK_FREQ_50_MHZ);

    GPIO_Init();
    TIMER6_Init();
    //Lets start timer
    HAL_TIM_Base_Start_IT(&htimer6);

while(1);

return 0;
}


/**
  * @brief System Clock Configuration
  * @retval None
  */

void SystemClockConfig(void){


}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef ledgpio;
	ledgpio.Pin = GPIO_PIN_5;
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA,&ledgpio);
}


/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
void TIMER6_Init(void)
{
	htimer6.Instance = TIM6;
	htimer6.Init.Prescaler = 24;
	htimer6.Init.Period = 64000-1;
	if( HAL_TIM_Base_Init(&htimer6) != HAL_OK )
	{
		Error_handler();
	}
}


/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_handler(void)
{
	while(1);
}
