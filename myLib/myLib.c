/*
 * myLib.c
 *
 *  Created on: Nov 15, 2024
 *      Author: user
 */
#include "main.h"
#include <stdio.h>
//#include "C:\Users\user\STM32Cube\Repository\STM32Cube_FW_F4_V1.28.1\Drivers\STM32F4xx_HAL_Driver\Inc\stm32f4xx_hal_i2c.h"
extern UART_HandleTypeDef huart2;
//int *hi2c = NULL;
//I2C_HandleTypeDef *hi2c = NULL;

int __io_getchar(void)
{
	char ch;
	while(HAL_UART_Receive(&huart2, &ch, 1, 10) != HAL_OK);
	HAL_UART_Transmit(&huart2, &ch, 1, 10);	//Echo
	if(ch == '\r')
		HAL_UART_Transmit(&huart2, "\n", 1, 10);
	return ch;
}
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, &ch, 1, 10);
	return ch;
}

void StandBy()
{
	while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin));
}

void ProgramStart(char * str)
{
	//printf("\033[2J\033[0;0H");
	cls();
	Cursor(0, 0);
	printf("Program Name - %s\r\n", str);
	printf("Press Blue-Button(B1) to Start...\r\n");
	StandBy();
	setvbuf(stdin, NULL, _IONBF, 0);	//scanf buffer clear
}

void cls()
{
	printf("\033[2J");
}

void Cursor(int x, int y)
{
	char buf[20];
	sprintf(buf, "\033[%d;%dH", y, x);
	puts(buf);   		//or printf("%s", buf);
}

//void ToUpper(char *str)
//{
//	int cnt = 0;
//
//	while(cnt < strlen(str))
//	{
//		if(*(str+cnt) >= 'a' && *(str+cnt) <= 'z') *(str+cnt) -= 32;
//		cnt++;
//	}
//}
//
//void ToLower(char *str)
//{
//	int cnt = 0;
//
//	while(cnt < strlen(str))
//	{
//		if(*(str+cnt) >= 'A' && *(str+cnt) <= 'Z' ) *(str+cnt) += 32;
//		cnt++;
//	}
//}
void ToUpper (char *s)
{
	while(*s)
	{
		if(*s >= 'a' && *s <= 'z') *s &= 0xdf;
		s++;
	}
}
void ToLower (char *s)
{
	while(*s)
	{
		if(*s >= 'A' && *s <= 'Z') *s |= ~0xdf;
		s++;
	}
}
//void i2c_init(I2C_HandleTypeDef *p)

//void i2c_init(int *p)
//{
//	hi2c = p;
//}
//
//int i2c_scan()
//{
//	if(hi2c == NULL) return;
//	cls();
//	Cursor(0, 0);
//	for(int addr = 0; addr < 128; addr++)
//	{
//		if(HAL_I2C_IsDeviceReady(hi2c, addr, 1, 10) == HAL_OK)
//			printf("  %02x ", addr);
//		else
//			printf("  .  ");
//
//		if((addr % 16) == 15) printf("\r\n");
//	}
//}
