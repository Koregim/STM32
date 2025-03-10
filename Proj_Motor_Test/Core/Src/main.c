/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int mode = 0;
int ccrL , ccrR;
int duty;
int dutyL, dutyR;

volatile int Time_REF, Time_RT;
// volatile double dist;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{}

void Micro_Delay(int us)// us : micro-sec (10^-6)
{
	// Consider Overflow Case
	// int t1 = htim2.Instance->CNT;
	htim2.Instance->CNT = 0;
	while( htim2.Instance->CNT <  us );
}

void Trigger1()
{
	// *** Trigger Pulse Gen. *** //
	// HAL_GPIO_WritePin(Pin Group, Pin#, Pin State(H/L));
	HAL_GPIO_WritePin(Trig1_GPIO_Port, Trig1_Pin, 0); // Initial Reset
	Micro_Delay(10); // 10us Delay

	HAL_GPIO_WritePin(Trig1_GPIO_Port, Trig1_Pin, 1); // Trigger Pulse High
	Micro_Delay(10); // 10us Delay

	HAL_GPIO_WritePin(Trig1_GPIO_Port, Trig1_Pin, 0); // Reset
	// *** Trigger Pulse Gen. *** //
	// printf( "Trigger END \r\n" );
}
void Trigger2()
{
	// *** Trigger Pulse Gen. *** //
	// HAL_GPIO_WritePin(Pin Group, Pin#, Pin State(H/L));
	HAL_GPIO_WritePin(Trig2_GPIO_Port, Trig2_Pin, 0); // Initial Reset
	Micro_Delay(10); // 10us Delay

	HAL_GPIO_WritePin(Trig2_GPIO_Port, Trig2_Pin, 1); // Trigger Pulse High
	Micro_Delay(10); // 10us Delay

	HAL_GPIO_WritePin(Trig2_GPIO_Port, Trig2_Pin, 0); // Reset
	// *** Trigger Pulse Gen. *** //
	// printf( "Trigger END \r\n" );
}
void Trigger3()
{
	// *** Trigger Pulse Gen. *** //
	// HAL_GPIO_WritePin(Pin Group, Pin#, Pin State(H/L));
	HAL_GPIO_WritePin(Trig3_GPIO_Port, Trig3_Pin, 0); // Initial Reset
	Micro_Delay(10); // 10us Delay

	HAL_GPIO_WritePin(Trig3_GPIO_Port, Trig3_Pin, 1); // Trigger Pulse High
	Micro_Delay(10); // 10us Delay

	HAL_GPIO_WritePin(Trig3_GPIO_Port, Trig3_Pin, 0); // Reset
	// *** Trigger Pulse Gen. *** //
	// printf( "Trigger END \r\n" );
}

static double dist[3];
double* Distances()
 {
	 // static double dist[3];
	 uint32_t t0, t1, t2, t3, t4, t5, t6;
	 //
	 Trigger1();
	 // t0 = htim2.Instance->CNT;
	 // printf( "%d \r\n", HAL_GPIO_ReadPin(Echo1_GPIO_Port, Echo1_Pin) );
	 while( HAL_GPIO_ReadPin(Echo1_GPIO_Port, Echo1_Pin) == 0 )
	 {
		 // t1 = htim2.Instance->CNT;
		 // printf( "t1 : %d \r\n", t1 );
		if(htim2.Instance->CNT > t0 + 30000)
		{
		   printf( "Timeout : %d %d \r\n", t0, htim2.Instance->CNT );
		   return NULL;
		}
	 }
	 t1 = htim2.Instance->CNT;
	 //printf( "t1 : %d \r\n", t1 );
	 while(HAL_GPIO_ReadPin(Echo1_GPIO_Port, Echo1_Pin) == 1)
	 {
		if(htim2.Instance->CNT > t1 + 30000)
		{
			printf( "Timeout : %d \r\n", htim2.Instance->CNT );
			return NULL;
		}
	 }
	 t2 = htim2.Instance->CNT;
	 //printf( "t2 : %d \r\n", t2 );
	 //
	 Trigger2();
	 t0 = htim2.Instance->CNT;
	 while(HAL_GPIO_ReadPin(Echo2_GPIO_Port, Echo2_Pin) == 0)
	 {
		if(htim2.Instance->CNT > t0 + 30000)
		{
		   return NULL;
		}
	 }
	 t3 = htim2.Instance->CNT;
	 // printf( "t3 : %d \r\n", t3 );
	 while(HAL_GPIO_ReadPin(Echo2_GPIO_Port, Echo2_Pin) == 1)
	 {
		if(htim2.Instance->CNT > t3 + 30000)
		{
		   return NULL;
		}
	 }
	 t4 = htim2.Instance->CNT;
	 // printf( "t4 : %d \r\n", t4 );
	 //
	 Trigger3();
	 t0 = htim2.Instance->CNT;
	 while(HAL_GPIO_ReadPin(Echo3_GPIO_Port, Echo3_Pin) == 0)
	 {
		if(htim2.Instance->CNT > t0 + 30000)
		{
		   return NULL;
		}
	 }
	 t5 = htim2.Instance->CNT;
	 // printf( "t5 : %d \r\n", t5 );
	 while(HAL_GPIO_ReadPin(Echo3_GPIO_Port, Echo3_Pin) == 1)
	 {
		if(htim2.Instance->CNT > t5 + 30000)
		{
		   return NULL;
		}
	 }
	 t6 = htim2.Instance->CNT;
	 // printf( "t6 : %d \r\n", t6 );
	 //
	 dist[0] =(t2 - t1)*0.17;
	 dist[1] =(t4 - t3)*0.17;
	 dist[2] =(t6 - t5)*0.17;

	 // printf( "Distance END \r\n" );
	 return dist;
}

void Forward()
{
	  HAL_GPIO_WritePin(DIR_A1_GPIO_Port, DIR_A1_Pin, 1); // DIR_A1 = 'H'
	  HAL_GPIO_WritePin(DIR_A2_GPIO_Port, DIR_A2_Pin, 0); // DIR_A2 = 'L'

	  HAL_GPIO_WritePin(DIR_B1_GPIO_Port, DIR_B1_Pin, 1); // DIR_B1 = 'H'
	  HAL_GPIO_WritePin(DIR_B2_GPIO_Port, DIR_B2_Pin, 0); // DIR_B2 = 'L'

   	   dutyL = 35; dutyR = 35;
   	   ccrL = (htim3.Instance->ARR + 1)  * dutyL / 100;
   	   ccrR = (htim3.Instance->ARR + 1) * dutyR / 100;
   	   htim3.Instance->CCR2 = ccrL - 1;
   	   htim3.Instance->CCR1 = ccrR - 1;
}

void Left()
{
	  HAL_GPIO_WritePin(DIR_A1_GPIO_Port, DIR_A1_Pin, 1); // DIR_A1 = 'H'
	  HAL_GPIO_WritePin(DIR_A2_GPIO_Port, DIR_A2_Pin, 0); // DIR_A2 = 'L'

	  HAL_GPIO_WritePin(DIR_B1_GPIO_Port, DIR_B1_Pin, 1); // DIR_B1 = 'H'
	  HAL_GPIO_WritePin(DIR_B2_GPIO_Port, DIR_B2_Pin, 0); // DIR_B2 = 'L'

   dutyL = 5; dutyR = 40;
   ccrL = (htim3.Instance->ARR + 1)  * dutyL / 100;
   ccrR = (htim3.Instance->ARR + 1) * dutyR / 100;
   htim3.Instance->CCR2 = ccrL - 1;
   htim3.Instance->CCR1 = ccrR - 1;
}
void Right()
{
	  HAL_GPIO_WritePin(DIR_A1_GPIO_Port, DIR_A1_Pin, 1); // DIR_A1 = 'H'
	  HAL_GPIO_WritePin(DIR_A2_GPIO_Port, DIR_A2_Pin, 0); // DIR_A2 = 'L'

	  HAL_GPIO_WritePin(DIR_B1_GPIO_Port, DIR_B1_Pin, 1); // DIR_B1 = 'H'
	  HAL_GPIO_WritePin(DIR_B2_GPIO_Port, DIR_B2_Pin, 0); // DIR_B2 = 'L'

   dutyL = 40; dutyR = 5;
   ccrL = (htim3.Instance->ARR + 1)  * dutyL / 100;
   ccrR = (htim3.Instance->ARR + 1) * dutyR / 100;
   htim3.Instance->CCR2 = ccrL - 1;
   htim3.Instance->CCR1 = ccrR - 1;
}
void Reverse()
{
	  HAL_GPIO_WritePin(DIR_A1_GPIO_Port, DIR_A1_Pin, 0); // DIR_A1 = 'L'
	  HAL_GPIO_WritePin(DIR_A2_GPIO_Port, DIR_A2_Pin, 1); // DIR_A2 = 'H'

	  HAL_GPIO_WritePin(DIR_B1_GPIO_Port, DIR_B1_Pin, 0); // DIR_B1 = 'L'
	  HAL_GPIO_WritePin(DIR_B2_GPIO_Port, DIR_B2_Pin, 1); // DIR_B2 = 'H'

   dutyL = 40; dutyR = 40;
   ccrL = (htim3.Instance->ARR + 1)  * dutyL / 100;
   ccrR = (htim3.Instance->ARR + 1) * dutyR / 100;
   htim3.Instance->CCR2 = ccrL - 1;
   htim3.Instance->CCR1 = ccrR - 1;
}

void Stop()
{
	  HAL_GPIO_WritePin(DIR_A1_GPIO_Port, DIR_A1_Pin, 1); // DIR_A1 = 'H'
	  HAL_GPIO_WritePin(DIR_A2_GPIO_Port, DIR_A2_Pin, 0); // DIR_A2 = 'L'

	  HAL_GPIO_WritePin(DIR_B1_GPIO_Port, DIR_B1_Pin, 1); // DIR_B1 = 'H'
	  HAL_GPIO_WritePin(DIR_B2_GPIO_Port, DIR_B2_Pin, 0); // DIR_B2 = 'L'

   dutyL = 5; dutyR = 5;
   ccrL = (htim3.Instance->ARR + 1)  * dutyL / 100;
   ccrR = (htim3.Instance->ARR + 1) * dutyR / 100;
   htim3.Instance->CCR2 = ccrL - 1;
   htim3.Instance->CCR1 = ccrR - 1;
}

#define BUF_SIZE 100
char buf1[BUF_SIZE], buf2[BUF_SIZE]; // DMA Buffer
char dum1, dum2;
int head1 = 0, head2 = 0, tail1 = 0, tail2 = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart6)
	{
		buf1[tail1++] = dum1;
		HAL_UART_Transmit(&huart2, &dum1/*== buf1+t1-1*/, 1, 10);		// putty print
		if(dum1 == '\r')		// End of Line
		{
			CheckCMD(buf1);
			tail1 = 0;
		}
		HAL_UART_Receive_IT(&huart6, &dum1, 1);			// interrupt chain
	}
	else if(huart == &huart2)
	{
		buf2[tail2++] = dum2;
		HAL_UART_Transmit(&huart2, &dum2, 1, 10); // terminal echo
		if(dum2 == '\r')  // CR : 0x0d
		{
			HAL_UART_Transmit(&huart2, "\n", 1, 10); // terminal echo

			buf2[tail2++] = '\n'; // == HAL_UART_Transmit(&huart1, "\n", 1, 10);
			HAL_UART_Transmit(&huart6, buf2, tail2, 10);	// AT Command
//			HAL_UART_Transmit(&huart1, "\n", 1, 10);
			tail2 = 0;
		}
		HAL_UART_Receive_IT(&huart2, &dum2, 1);			// interrupt chain
	}
}

char * Trim(char *s) // s = "    xx x \t \r\n" ==> "xx x"
{
	int trim_head = 0, trim_tail = strlen(s) - 1;
	while(*(s + trim_head) == ' ' || *(s + trim_head) == '\r' || *(s + trim_head) == '\n' || *(s + trim_head) == '\t') trim_head++;
//	{
//		if(*(s + trim_head) == ' ' || *(s + trim_head) == '\r' || *(s + trim_head) == '\n' || *(s + trim_head) == '\t') trim_head++;
//		else break;
//	}
	while(*(s + trim_tail) == ' ' || *(s + trim_tail) == '\r' || *(s + trim_tail) == '\n' || *(s + trim_tail) == '\t') trim_tail--;
//	{
//		if(*(s + trim_tail) == ' ' || *(s + trim_tail) == '\r' || *(s + trim_tail) == '\n' || *(s + trim_tail) == '\t') trim_tail--;
//		else break;
//	}
	char *dest = (char *) malloc(trim_tail - trim_head + 1);
	strncpy(dest, (s + trim_head), (trim_tail - trim_head + 1));
	return dest;
}

void CheckCMD(char *bb)
{
	char *str = Trim(bb);
//	char * str = Trim(bb); //Trim : white space remove
//	Trim_EX(str,bb);
	ToUpper(str);
	if(strncmp(str, "MODE", 4) == 0)
	{
		str = Trim(str+4);
		if(str[0] == '1')
			mode = 1;
		else if(str[0] == '0')
			mode = 0;
	}
	else if(strncmp(str, "MOVE", 4) == 0 && mode == 0)
	{
		str = Trim(str+4);
		if(str[0] == '0')
			Stop();
		else if(str[0] == '1')
			Forward();
		else if(str[0] == '2')
			Reverse();
		else if(str[0] == '3')
			Left();
		else if(str[0] == '4')
			Right();
	}
}

#define NUM_MAX7219  4
#define MAX7219_REG_NOOP        0x00
#define MAX7219_REG_DIGIT0      0x01
#define MAX7219_REG_DIGIT1      0x02
#define MAX7219_REG_DIGIT2      0x03
#define MAX7219_REG_DIGIT3      0x04
#define MAX7219_REG_DIGIT4      0x05
#define MAX7219_REG_DIGIT5      0x06
#define MAX7219_REG_DIGIT6      0x07
#define MAX7219_REG_DIGIT7      0x08
#define MAX7219_REG_DECODEMODE  0x09
#define MAX7219_REG_INTENSITY   0x0A
#define MAX7219_REG_SCANLIMIT   0x0B
#define MAX7219_REG_SHUTDOWN    0x0C
#define MAX7219_REG_DISPLAYTEST 0x0F

void MAX7219_WriteReg(uint8_t reg, uint8_t data) {
    //
    uint8_t txData[2];
    txData[0] = reg;
    txData[1] = data;

    HAL_SPI_Transmit(&hspi2, txData, 2, HAL_MAX_DELAY);
}

void MAX7219_InitMatrix(void) {
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	//
	HAL_Delay(1);
	// SHUTDOWN
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	for( int chip=0; chip < NUM_MAX7219; chip++ )
	{
		MAX7219_WriteReg(MAX7219_REG_SHUTDOWN, 0x01);
	}
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	// DISPLAYTEST
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	for( int chip=0; chip < NUM_MAX7219; chip++ )
	{
		MAX7219_WriteReg(MAX7219_REG_DISPLAYTEST, 0x00);
	}
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	// SCANLIMIT
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	for( int chip=0; chip < NUM_MAX7219; chip++ )
	{
		MAX7219_WriteReg(MAX7219_REG_SCANLIMIT, 0x07);
	}
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	// DECODEMODE
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	for( int chip=0; chip < NUM_MAX7219; chip++ )
	{
		MAX7219_WriteReg(MAX7219_REG_DECODEMODE, 0x00);
	}
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	// INTENSITY
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	for( int chip=0; chip < NUM_MAX7219; chip++ )
	{
		MAX7219_WriteReg(MAX7219_REG_INTENSITY, 0x07);
	}
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);

	for (int row = 0; row < 8; row++)
	{
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		for(int chip=0; chip < NUM_MAX7219; chip++)
		{
			MAX7219_WriteReg(MAX7219_REG_DIGIT0 + row, 0);
		}
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
	}
}

#define DFPLAYER_NEXT          0x01
#define DFPLAYER_PREV          0x02
#define DFPLAYER_PLAY          0x0D
#define DFPLAYER_PAUSE         0x0E
#define DFPLAYER_VOLUME        0x06
#define DFPLAYER_SELECT_TRACK  0x03
#define DFPLAYER_EQ            0x07
#define DFPLAYER_RESET         0x0C

void DFPlayer_Send_Cmd(uint8_t cmd, uint16_t param) {
    uint8_t buffer[10];
    uint16_t checksum;

    buffer[0] = 0x7E;   // Start byte
    buffer[1] = 0xFF;   // Version
    buffer[2] = 0x06;   // Length
    buffer[3] = cmd;    // Command
    buffer[4] = 0x00;   // Feedback (0: No feedback, 1: Feedback)
    buffer[5] = (uint8_t)(param >> 8);  // Parameter (High byte)
    buffer[6] = (uint8_t)(param & 0xFF); // Parameter (Low byte)

    // Checksum
    checksum = 0;
    for (int i = 1; i < 7; i++) {
        checksum += buffer[i];
    }
    checksum = 0 - checksum;
    buffer[7] = (uint8_t)(checksum >> 8);   // Checksum (High byte)
    buffer[8] = (uint8_t)(checksum & 0xFF);  // Checksum (Low byte)
    buffer[9] = 0xEF;   // End byte


    HAL_UART_Transmit(&huart1, buffer, 10, 100); // UART
    HAL_Delay(20); //DFPlayer
}

//
void DFPlayer_Play_Track(uint16_t track_number) {
     DFPlayer_Send_Cmd(DFPLAYER_SELECT_TRACK, track_number);
}

// (0 ~ 30)
void DFPlayer_Set_Volume(uint8_t volume) {
    DFPlayer_Send_Cmd(DFPLAYER_VOLUME, volume);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  MAX7219_InitMatrix(); // Initialization
  ProgramStart( "Project : Motor Control" );

  // *** Motor Default Setting *** //
  // Rotation Direction :  (Default) Both Clockwise
  HAL_GPIO_WritePin(DIR_A1_GPIO_Port, DIR_A1_Pin, 1); // DIR_A1 = 'H'
  HAL_GPIO_WritePin(DIR_A2_GPIO_Port, DIR_A2_Pin, 0); // DIR_A2 = 'L'

  HAL_GPIO_WritePin(DIR_B1_GPIO_Port, DIR_B1_Pin, 1); // DIR_B1 = 'H'
  HAL_GPIO_WritePin(DIR_B2_GPIO_Port, DIR_B2_Pin, 0); // DIR_B2 = 'L'

  // Rotation Speed :  (Default) Both Duty 30%
  duty = 40;
  ccrL = (htim3.Instance->ARR + 1)  * duty / 100;
  ccrR = (htim3.Instance->ARR + 1) * duty / 100;
  htim3.Instance->CCR2 = ccrL - 1;
  htim3.Instance->CCR1 = ccrR - 1;
  //

  // TIM Start
  // HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1); // Start TIM11 CH1 for Trigger PWM Signal
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Start TIM3 CH2 for ENA PWM Signal
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Start TIM3 CH1 for ENB PWM Signal
  HAL_UART_Receive_IT(&huart6, &dum1, 1);
  HAL_UART_Receive_IT(&huart2, &dum2, 1);


  uint8_t matrixData[4][8] = {0};

  	//
	matrixData[0][0] = 0b10001011;
	matrixData[0][1] = 0b10001010;
	matrixData[0][2] = 0b10001010;
	matrixData[0][3] = 0b11111011;
	matrixData[0][4] = 0b10001010;
	matrixData[0][5] = 0b10001010;
	matrixData[0][6] = 0b10001011;
	matrixData[0][7] = 0b00000000;

	matrixData[1][0] = 0b11101000;
	matrixData[1][1] = 0b00001000;
	matrixData[1][2] = 0b00001000;
	matrixData[1][3] = 0b11001000;
	matrixData[1][4] = 0b00001000;
	matrixData[1][5] = 0b00001000;
	matrixData[1][6] = 0b11101111;
	matrixData[1][7] = 0b00000000;

	matrixData[2][0] = 0b00100000;
	matrixData[2][1] = 0b00100000;
	matrixData[2][2] = 0b00100000;
	matrixData[2][3] = 0b00100000;
	matrixData[2][4] = 0b00100000;
	matrixData[2][5] = 0b00100000;
	matrixData[2][6] = 0b10111110;
	matrixData[2][7] = 0b00000000;

	matrixData[3][0] = 0b01110000;
	matrixData[3][1] = 0b10001000;
	matrixData[3][2] = 0b10001000;
	matrixData[3][3] = 0b10001000;
	matrixData[3][4] = 0b10001000;
	matrixData[3][5] = 0b10001000;
	matrixData[3][6] = 0b01110000;
	matrixData[3][7] = 0b00000000;
	//

	for (int row = 0; row < 8; row++)
	{
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		for(int chip=0; chip < NUM_MAX7219; chip++)
		{
			MAX7219_WriteReg(MAX7219_REG_DIGIT0 + row, matrixData[chip][row]);
		}
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
	}

	// HAL_Delay(200);
	DFPlayer_Send_Cmd(DFPLAYER_RESET, 0);
	HAL_Delay(1000);
	DFPlayer_Set_Volume(20); // (0~30)
	HAL_Delay(100);
	DFPlayer_Play_Track(1);
	HAL_Delay(400);
	DFPlayer_Send_Cmd(0x11, 1); // repeat
	HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // [1] Ultra-sonic Sensor


	  // [2] Distance Check & Motor Control
	  // static double dist[3];
	  // dist[0] : Left
	  // dist[1] : Forward
	  // dist[2] : Right
	  if(mode == 1)
	  {
		  Distances();
		  //
		  if ( dist[0] < 100 ) // Near to Left
			  {
				  Right();
				  printf("Left \r\n");
			  }
		  else if ( dist[2] < 100 ) // Near to Right
			  {
				  Left();
				  printf("Right \r\n");
			  }
		  //
		  else if ( dist[0] > 350 ) // Far from Left
			  {
				  Left();
				  printf("Left \r\n");
			  }
		  else if ( dist[1] > 350 ) // No Obstacle at Front
			  {
				  Forward();
				  printf("Forward \r\n");
			  }
		  else if ( dist[2] > 350 ) // Far from Right
			  {
				  Right();
				  printf("Right \r\n");
			  }
		  //
		  else
			  Reverse();
	  }

	  /*
	  if ( dist[0] < 350 ) { Right(); printf("Left \r\n"); }
	  else if ( dist[1] < 350 ) { Reverse(); printf("Forward \r\n"); }
	  else if ( dist[2] < 350 ) { Left(); printf("Right \r\n"); }
	  else Forward();
	  */
	  printf( "Dist : %f  %f %f\r\n",dist[0], dist[1], dist[2] );


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Trig2_Pin|DIR_B2_Pin|Trig1_Pin
                          |DIR_A1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIR_B1_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR_A2_Pin|DIR_A1B8_Pin|Trig3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Trig2_Pin DIR_B2_Pin Trig1_Pin
                           DIR_A1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Trig2_Pin|DIR_B2_Pin|Trig1_Pin
                          |DIR_A1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo3_Pin */
  GPIO_InitStruct.Pin = Echo3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo1_Pin */
  GPIO_InitStruct.Pin = Echo1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_B1_Pin CS_Pin */
  GPIO_InitStruct.Pin = DIR_B1_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_A2_Pin DIR_A1B8_Pin Trig3_Pin */
  GPIO_InitStruct.Pin = DIR_A2_Pin|DIR_A1B8_Pin|Trig3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo2_Pin */
  GPIO_InitStruct.Pin = Echo2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
