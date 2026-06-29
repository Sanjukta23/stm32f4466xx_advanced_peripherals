/*
 * it.c
 *
 *  Created on: Jun 22, 2026
 *      Author: SANJU
 */


#include "main.h"
extern UART_HandleTypeDef huart2;

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


