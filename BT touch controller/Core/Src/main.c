/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcdtp.h"
#include "xpt2046.h"
#include <stdio.h>
#include <string.h>





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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
uint32_t adc_value[2];  // Raw ADC values
int16_t cal_value[2];   // Calibrated values (-3 to +3)
char buff[32];          // Buffer for string formatting


uint8_t Data[3];																				//BT


// Calibration constants
#define ADC_SAMPLES 8   // Number of samples for averaging
#define DEAD_ZONE 50    // Dead zone for center position

#define X_CENTER 2029
#define X_MIN    0001
#define X_MAX    4090
#define Y_CENTER 2008
#define Y_MIN    0
#define Y_MAX    4040



    int16_t joystick_x;     // X value
    int16_t joystick_y;     // Y value

/* Function to map ADC value to -5 to +5 scale with dead zone */
int16_t calibrate_axis(uint32_t raw, uint32_t center, uint32_t min_val, uint32_t max_val) {
    if (raw > (center - DEAD_ZONE) && raw < (center + DEAD_ZONE)) {
        return 0;  // Within dead zone, return center position
    }
    
    if (raw < center) {
        // Map values below center to -5 to 0
        if (raw < min_val) raw = min_val;
        return (int16_t)(-5.0f * (center - raw) / (center - min_val));
    } else {
        // Map values above center to 0 to +5
        if (raw > max_val) raw = max_val;
        return (int16_t)(5.0f * (raw - center) / (max_val - center));
    }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UARTEx_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if(huart->Instance==USART2)
  {

    HAL_UART_Transmit(&huart2, Data, Size, 1000);

  }
  HAL_UARTEx_ReceiveToIdle_IT(&huart2,Data,3);
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
  MX_FSMC_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	macXPT2046_CS_DISABLE();
	
	LCD_INIT();

   // Initialize and calibrate both ADCs
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);

  // Configure ADC1 for Channel 12
  hadc1.Init.ScanConvMode = DISABLE;  // Single channel mode
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc1);

  // Configure ADC2 for Channel 13
  hadc2.Init.ScanConvMode = DISABLE;  // Single channel mode
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc2);

  // Configure Channel 12 on ADC1
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  // Configure Channel 13 on ADC2
  sConfig.Channel = ADC_CHANNEL_13;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  // Start both ADCs
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  LCD_DrawString(100, 35, "X       Y");
  LCD_DrawString(10, 60, "Joystick:");




  HAL_UARTEx_ReceiveToIdle_IT(&huart2,Data,3); 			  									// BT





	LCD_Clear (50, 80, 140, 70, RED);
	LCD_DrawString(68, 100, "TOUCHPAD Init");
	HAL_Delay(2000);

	while( ! XPT2046_Touch_Calibrate () );   

	LCD_Clear ( 0, 0, 240, 320, GREY );
	// LCD_Clear ( 90,  230,  60, 60, BLUE );

  // Draw 4 buttons at the bottom
  LCD_Clear(20, 232, 50, 50, BLUE);    // Button 1
  LCD_Clear(75, 232, 50, 50, GREEN);   // Button 2
  LCD_Clear(130, 232, 50, 50, YELLOW); // Button 3
  LCD_Clear(185, 232, 50, 50, RED);    // Button 4 (reset)

  // Optionally, add labels
  LCD_DrawString(35, 255, "B1");
  LCD_DrawString(90, 255, "B2");
  LCD_DrawString(145, 255, "B3");
  LCD_DrawString(200, 255, "RST");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
  while (1)
  {
    if ( ucXPT2046_TouchFlag == 1 )	         
    {
			Check_touchkey();			
      ucXPT2046_TouchFlag = 0;		            
    }					
		HAL_Delay(50);		
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint32_t x_sum = 0;
    uint32_t y_sum = 0;

    // Take multiple samples and average them
    for(int i = 0; i < ADC_SAMPLES; i++) {
        // Read X axis (ADC1 Channel 12)
        HAL_ADC_PollForConversion(&hadc1, 100);
        x_sum += HAL_ADC_GetValue(&hadc1);

        // Read Y axis (ADC2 Channel 13)
        HAL_ADC_PollForConversion(&hadc2, 100);
        y_sum += HAL_ADC_GetValue(&hadc2);
        
        HAL_Delay(1);
    }

    // Calculate averages
    adc_value[0] = x_sum / ADC_SAMPLES;
    adc_value[1] = y_sum / ADC_SAMPLES;

    // Calibrate values to -10 to +10 scale
    cal_value[0] = calibrate_axis(adc_value[0], X_CENTER, X_MIN, X_MAX);
    cal_value[1] = calibrate_axis(adc_value[1], Y_CENTER, Y_MIN, Y_MAX);

    // Display raw X value
    sprintf(buff, "%04lu", adc_value[0]);
    LCD_DrawString(100, 80, buff);

    // Display raw Y value
    sprintf(buff, "%04lu", adc_value[1]);
    LCD_DrawString(160, 80, buff);

    // Display calibrated X value
    sprintf(buff, "%3d", cal_value[0]);
    LCD_DrawString(100, 100, buff);

    // Display calibrated Y value
    sprintf(buff, "%3d", cal_value[1]);
    LCD_DrawString(160, 100, buff);

    // Read B1 button state (pressed = 0, not pressed = 1)
    uint8_t pressed = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET);

    if (pressed) {
        // Remap for B9 pressed
        switch (cal_value[0]) {
            case -5: Data[0] = 35; break;
            case -4: Data[0] = 34; break;
            case -3: Data[0] = 33; break;
            case -2: Data[0] = 32; break;
            case -1: Data[0] = 31; break;
            case 0:  Data[0] = 20; break;
            case 1:  Data[0] = 21; break;
            case 2:  Data[0] = 22; break;
            case 3:  Data[0] = 23; break;
            case 4:  Data[0] = 24; break;
            case 5:  Data[0] = 25; break;
            default: Data[0] = 0; break; // fallback
        }
        switch (cal_value[1]) {
            case -5: Data[1] = 35; break;
            case -4: Data[1] = 34; break;
            case -3: Data[1] = 33; break;
            case -2: Data[1] = 32; break;
            case -1: Data[1] = 31; break;
            case 0:  Data[1] = 20; break;
            case 1:  Data[1] = 21; break;
            case 2:  Data[1] = 22; break;
            case 3:  Data[1] = 23; break;
            case 4:  Data[1] = 24; break;
            case 5:  Data[1] = 25; break;
            default: Data[1] = 0; break; // fallback
        }
    } else {
        // Original mapping for B9 not pressed
        switch (cal_value[0]) {
            case -5: Data[0] = 15; break;
            case -4: Data[0] = 14; break;
            case -3: Data[0] = 13; break;
            case -2: Data[0] = 12; break;
            case -1: Data[0] = 11; break;
            default: Data[0] = (uint8_t)cal_value[0]; break;  // 0 to 5
        }
        switch (cal_value[1]) {
            case -5: Data[1] = 15; break;
            case -4: Data[1] = 14; break;
            case -3: Data[1] = 13; break;
            case -2: Data[1] = 12; break;
            case -1: Data[1] = 11; break;
            default: Data[1] = (uint8_t)cal_value[1]; break;  // 0 to 5
        }
    }

    HAL_UART_Transmit(&huart2, Data, 3, 100);

    sprintf(buff, "Raw: %3d %3d %3d", Data[0], Data[1], Data[2]);
    LCD_DrawString(10, 120, buff);






   // read the register to see if the nrf24l01 is connected to the microcontroller
    /*
    uint8_t status_reg = nrf24_ReadReg(STATUS);
    sprintf(buff, "ST:%02X", status_reg);
    LCD_DrawString(100, 140, buff);
    */



    // Start next conversions
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
    
    HAL_Delay(50);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

void Check_touchkey(void)
{
    strType_XPT2046_Coordinate strDisplayCoordinate;

    if (XPT2046_Get_TouchedPoint(&strDisplayCoordinate, &strXPT2046_TouchPara))
    {
        // Define 4 button areas horizontally in the region y: 232-282
        if ((strDisplayCoordinate.y > 232) && (strDisplayCoordinate.y < 282))
        {
            if ((strDisplayCoordinate.x > 20) && (strDisplayCoordinate.x < 70))
            {
                // Button 1
                Data[2] = 1;
            }
            else if ((strDisplayCoordinate.x > 75) && (strDisplayCoordinate.x < 125))
            {
                // Button 2
                Data[2] = 2;
            }
            else if ((strDisplayCoordinate.x > 130) && (strDisplayCoordinate.x < 180))
            {
                // Button 3
                Data[2] = 3;
            }
            else if ((strDisplayCoordinate.x > 185) && (strDisplayCoordinate.x < 235))
            {
                // Button 4 (reset)
                Data[2] = 0;
            }
        }
    }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
