/*
 * it.c
 *
 *  Created on: Jun 30, 2026
 *      Author: SANJU
 */

#include "main.h"

void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
