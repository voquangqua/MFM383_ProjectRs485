/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "math.h"
#include "stdio.h"
#include "stm32f4xx_it.h"
#include "stdint.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



#define METER_MAX 255




typedef enum
{
    MFM383A_ID_1 = 1,
    MFM383A_ID_2 = 2

}MFM383A_SlaveID_t;

typedef enum
{
    REG_V1N       = 0,
    REG_V2N       = 2,
    REG_V3N       = 4,
    REG_AVG_VLN   = 6,

    REG_V12       = 8,
    REG_V23       = 10,
    REG_V31       = 12,
    REG_AVG_VLL   = 14,

    REG_I1        = 16,
    REG_I2        = 18,
    REG_I3        = 20,
    REG_AVG_I     = 22,

    REG_KW1       = 24,
    REG_KW2       = 26,
    REG_KW3       = 28,

    REG_KVA1      = 30,
    REG_KVA2      = 32,
    REG_KVA3      = 34,

    REG_KVAR1     = 36,
    REG_KVAR2     = 38,
    REG_KVAR3     = 40,

    REG_TOTAL_KW  = 42,
    REG_TOTAL_KVA = 44,
    REG_TOTAL_KVAR= 46,

    REG_PF1       = 48,
    REG_PF2       = 50,
    REG_PF3       = 52,
    REG_AVG_PF    = 54,

    REG_FREQ      = 56,
    REG_KWH       = 58

}MFM383A_Register_t;


typedef struct
{
    float V1N;
    float V2N;
    float V3N;
    float AvgVLN;

    float V12;
    float V23;
    float V31;
    float AvgVLL;

    float I1;
    float I2;
    float I3;
    float AvgCurrent;

    float kW1;
    float kW2;
    float kW3;

    float kVA1;
    float kVA2;
    float kVA3;

    float kVAR1;
    float kVAR2;
    float kVAR3;

    float TotalkW;
    float TotalkVA;
    float TotalkVAR;

    float PF1;
    float PF2;
    float PF3;
    float AvgPF;

    float Frequency;
    float kWh;

}MFM383A_Data_t;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

__IO ITStatus Uart2Ready = RESET;
volatile uint8_t aRxBuffer[1];
uint8_t rx_nfn383[512];
uint16_t  u8_len_rx_mfm383;
uint8_t  u8_timeout_uart;
volatile uint8_t aTx_Buffer[64];
volatile uint16_t dem_t =0;
volatile uint16_t dem_t10 =0;
volatile uint8_t SenFlag_Modbud =0;

MFM383A_Data_t g_meter[METER_MAX];

float v_1n;


typedef enum
{
	Receiver =0,
	Transmiter=1
}RS485State;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    unsigned int i; /* will index into CRC lookup */

    /* pass through message buffer */
    while (buffer_length--) {
        i = crc_lo ^ *buffer++; /* calculate the CRC  */
        crc_lo = crc_hi ^ table_crc_hi[i];
        crc_hi = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}

void SetRS485Mode(RS485State state)
{
	switch (state)
	{
	 case Receiver: HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET); break;
	 case Transmiter: HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET); break;
	 default:  break;
	}
}
void Ledstatus_On (uint16_t led_pin)
{
  HAL_GPIO_WritePin(GPIOD, led_pin, RESET); 
}

void Ledstatus_Off (uint16_t led_pin)
{
  HAL_GPIO_WritePin(GPIOD, led_pin, SET); 
}

void RS485SendData()
{
	uint8_t data[8]={0};
	data[0]=(uint8_t)MFM383A_ID_1;// slave ID
	data[1]=0x04;
	data[2]=0x00;
	data[3]=0x00;
	data[4]=0x00;
  data[5]=0x3C;
	uint16_t tmp_crc=crc16(data,6);
	data[6]=tmp_crc&0xFF;
	data[7]=(tmp_crc>>8)&0xFF;

	// SetRS485Mode(Transmiter);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);
	// UART1_TX(data, 8);
  HAL_UART_Transmit(&huart2, data, 8, 100);
  // đợi gửi xong hoàn toàn
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);

	// SetRS485Mode(Receiver);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET);

  Ledstatus_On(GPIO_PIN_11);
  Ledstatus_Off(GPIO_PIN_9);
  Ledstatus_Off(GPIO_PIN_10);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
if(htim->Instance == TIM2){
	dem_t = dem_t +1;
	if(dem_t>=20){
		dem_t10 = dem_t10+1;
    if(dem_t10>=2){
      // modbus_tx_data();
      // Slave ID : 0x01
      // Function code : 0x04 // read register

      dem_t10 = 0;
      if(SenFlag_Modbud == 0){
        SenFlag_Modbud = 1;
      }

      // RS485SendData(0x01, 0x04, 0x00, 64 );
    }
		dem_t = 0;
	}
}
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);

  HAL_UART_Receive_IT(&huart2, aRxBuffer, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    if(SenFlag_Modbud==1){

	      memset(rx_nfn383, 0, sizeof(rx_nfn383));
	    	    // On Rx
	      HAL_UART_Receive_IT(&huart2, aRxBuffer, 1);
	      RS485SendData();
	      SenFlag_Modbud = 0;
	    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 33600;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 39;
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
  huart2.Init.BaudRate = 9600;
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
	 if(HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer,1) != HAL_OK)
	 	{

	 		Error_Handler();

	 	}
  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD9 PD10 PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
float modbusPyloadToFloat(uint8_t *payload)
{
    uint8_t temp[4];

    temp[0] = payload[3];
    temp[1] = payload[2];
    temp[2] = payload[1];
    temp[3] = payload[0];

    float value;
    memcpy(&value, temp, 4);
    return value;
}

// void handle_data_uart_mfm3883()
// {
// 	v_1n = modbusPyloadToFloat((uint8_t *)&rx_nfn383[0+3]);
// }

void handle_data_uart_mfm3883(void)
{
    // MFM383A_SlaveID_t slaveID = MFM383A_ID_2;
    MFM383A_SlaveID_t slaveID = MFM383A_ID_1;
    
    // uint8_t slaveID = rx_nfn383[0];

    if(slaveID == 0 || slaveID > METER_MAX)
        return;

    MFM383A_Data_t *meter = &g_meter[slaveID];

    meter->V1N        = modbusPyloadToFloat(&rx_nfn383[3]);
    meter->V2N        = modbusPyloadToFloat(&rx_nfn383[7]);
    meter->V3N        = modbusPyloadToFloat(&rx_nfn383[11]);
    meter->AvgVLN     = modbusPyloadToFloat(&rx_nfn383[15]);

    meter->V12        = modbusPyloadToFloat(&rx_nfn383[19]);
    meter->V23        = modbusPyloadToFloat(&rx_nfn383[23]);
    meter->V31        = modbusPyloadToFloat(&rx_nfn383[27]);
    meter->AvgVLL     = modbusPyloadToFloat(&rx_nfn383[31]);

    meter->I1         = modbusPyloadToFloat(&rx_nfn383[35]);
    meter->I2         = modbusPyloadToFloat(&rx_nfn383[39]);
    meter->I3         = modbusPyloadToFloat(&rx_nfn383[43]);
    meter->AvgCurrent = modbusPyloadToFloat(&rx_nfn383[47]);

    meter->kW1        = modbusPyloadToFloat(&rx_nfn383[51]);
    meter->kW2        = modbusPyloadToFloat(&rx_nfn383[55]);
    meter->kW3        = modbusPyloadToFloat(&rx_nfn383[59]);

    meter->kVA1       = modbusPyloadToFloat(&rx_nfn383[63]);
    meter->kVA2       = modbusPyloadToFloat(&rx_nfn383[67]);
    meter->kVA3       = modbusPyloadToFloat(&rx_nfn383[71]);

    meter->kVAR1      = modbusPyloadToFloat(&rx_nfn383[75]);
    meter->kVAR2      = modbusPyloadToFloat(&rx_nfn383[79]);
    meter->kVAR3      = modbusPyloadToFloat(&rx_nfn383[83]);

    meter->TotalkW    = modbusPyloadToFloat(&rx_nfn383[87]);
    meter->TotalkVA   = modbusPyloadToFloat(&rx_nfn383[91]);
    meter->TotalkVAR  = modbusPyloadToFloat(&rx_nfn383[95]);

    meter->PF1        = modbusPyloadToFloat(&rx_nfn383[99]);
    meter->PF2        = modbusPyloadToFloat(&rx_nfn383[103]);
    meter->PF3        = modbusPyloadToFloat(&rx_nfn383[107]);
    meter->AvgPF      = modbusPyloadToFloat(&rx_nfn383[111]);

    meter->Frequency  = modbusPyloadToFloat(&rx_nfn383[115]);
    meter->kWh        = modbusPyloadToFloat(&rx_nfn383[119]);

  Ledstatus_On(GPIO_PIN_9);
  Ledstatus_Off(GPIO_PIN_11);
  Ledstatus_On(GPIO_PIN_10);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (u8_timeout_uart == 0)
        {
            u8_len_rx_mfm383 = 0;
//            memset(rx_nfn383, 0, sizeof(rx_nfn383));
        }

//        if (u8_len_rx_mfm383 < 128)
//        {
            rx_nfn383[u8_len_rx_mfm383++] = aRxBuffer[0];
//        }
//            if (u8_len_rx_mfm383 == 9){
//
//            }

        u8_timeout_uart = 100;

        HAL_UART_Receive_IT(&huart2, aRxBuffer, 1);
    }
}

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  if((u8_timeout_uart>0)&&(--u8_timeout_uart==0))
  {
	  handle_data_uart_mfm3883();
  }

  /* USER CODE END SysTick_IRQn 1 */
}

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
