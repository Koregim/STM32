/*
 * lcd1602.c
 *
 *  Created on: Nov 21, 2024
 *      Author: user
 *      function : 1602 LCD device control
 */
#include "main.h"
#include "C:\Users\user\STM32Cube\Repository\STM32Cube_FW_F4_V1.28.1\Drivers\STM32F4xx_HAL_Driver\Inc\stm32f4xx_hal_i2c.h"
extern I2C_HandleTypeDef *hi2c;
I2C_HandleTypeDef *hi2c = NULL;
//int *hi2c;
//extern I2C_HandleTypeDef hi2c1;	// not recommanded
#define I2C_ADDR  0x4e	// 0x27 << 1 == 0x4e

void lcd_command (char cmd)		// cmd_bit : abcd_efgh
{
	char n1, n2, n3, n4, dd[4];
	n1 = cmd & 0xf0;			// n1 : abcd_0000, upper nibble == readWrite, Enable signal
	n2 = (cmd & 0x0f) << 4;		// n2 : efgh_0000, lower nibble == NoConnection, RS signal
	n3 = /*RW*/(1 << 3) | /*EN*/(1 << 2) | /*NC*/0 | /*RS*/ 0;
	n4 = /*RW*/(1 << 3) | /*EN*/ 0 		 | /*NC*/0 | /*RS*/ 0;
	dd[0] = n1 | /*0x0c*/ n3;
	dd[1] = n1 | /*0x08*/ n4;
	dd[2] = n2 | /*0x0c*/ n3;
	dd[3] = n2 | /*0x08*/ n4;

	HAL_I2C_Master_Transmit(hi2c, I2C_ADDR, dd, 4, 10);
}

void lcd_data (char ch)			// control signal(4bit) : ReadWrite (active low : write)/ Enable / NoConnection(HIGH) / RS(Resister Select)0 : Command, 1 : Data
{
	char n1, n2, n3, n4, dd[4];
	n1 = ch & 0xf0;				// n1 : abcd_0000, upper nibble == readWrite, Enable signal
	n2 = (ch & 0x0f) << 4;		// n2 : efgh_0000, lower nibble == NoConnection, RS signal
	n3 = /*RW*/(1 << 3) | /*EN*/(1 << 2) | /*NC*/0 | /*RS*/ (1 << 0);
	n4 = /*RW*/(1 << 3) | /*EN*/ 0 		 | /*NC*/0 | /*RS*/ (1 << 0);
	dd[0] = n1 | /*0x0d*/ n3;
	dd[1] = n1 | /*0x09*/ n4;
	dd[2] = n2 | /*0x0d*/ n3;
	dd[3] = n2 | /*0x09*/ n4;

	HAL_I2C_Master_Transmit(hi2c, I2C_ADDR, dd, 4, 10);
}

void lcd_init()
{
	HAL_Delay(1);
	lcd_command(0x01);	// screen clear
	HAL_Delay(10);
	lcd_command(0x02); 	// corsor home
	HAL_Delay(10);
	lcd_command(0x06);	//
	HAL_Delay(10);
	lcd_command(0x0f);
	HAL_Delay(30);
}

void lcd_print(char *str)
{
	while(*str) lcd_data(*str++);
}

void lcd_printEx(char *str, int ln)
{
//	if(ln == 0) lcd_command(0x80);
//	if(ln == 1) lcd_command(0xc0);
	ln ? lcd_command(0xc0) : lcd_command(0x80);
	lcd_print(str);
}
int ln2 = 0;
char sBuf[20];
void lcd_printEx2(char *str)
{
//	if(ln == 0) lcd_command(0x80);
//	if(ln == 1) lcd_command(0xc0);
//	if(ln2 == 0)
//	{
//		lcd_command(0x80);
//		ln2++;
//	}
//	else if(ln2 == 1)
//	{
//		lcd_command(0x80);
//		lcd_print(sBuf);
//		lcd_command(0xc0);
//		strcpy(sBuf, str);
//	}
	ln2++ ? lcd_command(0x80),HAL_Delay(100), lcd_print(sBuf), lcd_command(0xc0), strcpy(sBuf, str) : lcd_command(0x80);
	lcd_print(str);
}
void i2c_init(I2C_HandleTypeDef *p)
//void i2c_init(int *p)
{
	hi2c = p;
}

int i2c_scan()
{
	if(hi2c == NULL) return;
	for(int addr = 0; addr < 256; addr++)
	{
		if(HAL_I2C_IsDeviceReady(hi2c, addr, 1, 10) == HAL_OK)
			printf("  %02x ", addr);
		else
			printf("  .  ");

		if((addr % 16) == 15) printf("\r\n");
	}
}
