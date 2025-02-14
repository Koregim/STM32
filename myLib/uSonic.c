/*
 * uSonic.c
 *
 *  Created on: Nov 15, 2024
 *      Author: user
 */
#include "main.h"
#include <stdio.h>

//#include "C:\Users\user\STM32Cube\Repository\STM32Cube_FW_F4_V1.28.1\Drivers\STM32F4xx_HAL_Driver\Inc\stm32f4xx_hal_i2c.h"
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

void microDelay(int us) {
//	int t1 = htim2.Instance->CNT;
//	while((htim2.Instance->CNT - t1 < us));
//	{
//		if(htim2.Instance->CNT - t1 > us)	break;
//
//	}
	htim2.Instance->CNT = 0;
	while((htim2.Instance->CNT < us));
}

void Trigger()
{
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, 0);
	microDelay(10);
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, 1);
	microDelay(10);
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, 0);
}
double Distance()
{
	int t0 = 0, t1, t2;
	htim2.Instance->CNT = 0;
	Trigger();
	while(!(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin)))
	{
		if(htim2.Instance->CNT > 30000) return -1;
	}
	t1 = htim2.Instance->CNT;
	//	{
	//		if(HAL_GPIO_ReadPin(echo_GPIO_Port, echo_Pin)) break;
	//	}

	while(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin))
	{
		if(htim2.Instance->CNT > 60000 + t1) return -1;
	}
	t2 = htim2.Instance->CNT;

	double dist = (t2 - t1) * 0.17;



	return dist;
}

