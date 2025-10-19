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
#include "lcd.h"
#include "bsp_ov7725.h"
#include "bsp_sccb.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Stream the image in blocks
#define PHOTO_WIDTH  320
#define PHOTO_HEIGHT 240
#define PHOTO_SIZE   (PHOTO_WIDTH * PHOTO_HEIGHT)
#define BLOCK_SIZE   512

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
int16_t cal_value[2];   // Calibrated values (-3 to +3)
char buff[32];          // Buffer for string formatting


uint8_t Data[3];																				//BT

uint8_t joybut_pressed = 0;         //b9 pressed?



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
volatile uint8_t Ov7725_vsync ;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
void HAL_UARTEx_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if(huart->Instance==USART3)
  {


	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);    // Green LED on (active low)
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);  // Red LED off (active low)


      sprintf(buff, "RX OK %d", Size);
      LCD_DrawString(10, 140, (uint8_t *)buff);
      sprintf(buff, "D0:%3d D1:%3d D2:%3d", Data[0], Data[1], Data[2]);
      LCD_DrawString(10, 160, (uint8_t *)buff);

      // Start next reception
      HAL_UARTEx_ReceiveToIdle_IT(&huart3, Data, 3);
  }
}
*/

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART3)
  {


      // Restart reception
      HAL_UARTEx_ReceiveToIdle_IT(&huart3, Data, 3);
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
  MX_FSMC_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	LCD_INIT();
	


  LCD_DrawString(100, 35, "X       Y");
  LCD_DrawString(10, 60, "Joystick:");


  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	while(Ov7725_Init() != SUCCESS);
	Ov7725_vsync = 0;
	
  while (1)
  {

  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
  // Simple polling receive
  if (HAL_UART_Receive(&huart3, Data, 3, 100) == HAL_OK)
  {
      // Successfully received data
      sprintf(buff, "RX OK");
      LCD_DrawString(10, 140, (uint8_t *)buff);
      sprintf(buff, "D0:%3d D1:%3d D2:%3d", Data[0], Data[1], Data[2]);
      LCD_DrawString(10, 160, (uint8_t *)buff);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);    // Green LED on (active low)
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);  // Red LED off (active low)



  }


  // Default: not pressed
  joybut_pressed = 0;

  // Decode Data[0]
  switch (Data[0]) {
      case 35: cal_value[0] = -5; joybut_pressed = 1; break;
      case 34: cal_value[0] = -4; joybut_pressed = 1; break;
      case 33: cal_value[0] = -3; joybut_pressed = 1; break;
      case 32: cal_value[0] = -2; joybut_pressed = 1; break;
      case 31: cal_value[0] = -1; joybut_pressed = 1; break;
      case 20: cal_value[0] = 0;  joybut_pressed = 1; break;
      case 21: cal_value[0] = 1;  joybut_pressed = 1; break;
      case 22: cal_value[0] = 2;  joybut_pressed = 1; break;
      case 23: cal_value[0] = 3;  joybut_pressed = 1; break;
      case 24: cal_value[0] = 4;  joybut_pressed = 1; break;
      case 25: cal_value[0] = 5;  joybut_pressed = 1; break;
      case 15: cal_value[0] = -5; joybut_pressed = 0; break;
      case 14: cal_value[0] = -4; joybut_pressed = 0; break;
      case 13: cal_value[0] = -3; joybut_pressed = 0; break;
      case 12: cal_value[0] = -2; joybut_pressed = 0; break;
      case 11: cal_value[0] = -1; joybut_pressed = 0; break;
      case 0:  cal_value[0] = 0;  joybut_pressed = 0; break;
      case 1:  cal_value[0] = 1;  joybut_pressed = 0; break;
      case 2:  cal_value[0] = 2;  joybut_pressed = 0; break;
      case 3:  cal_value[0] = 3;  joybut_pressed = 0; break;
      case 4:  cal_value[0] = 4;  joybut_pressed = 0; break;
      case 5:  cal_value[0] = 5;  joybut_pressed = 0; break;
      default: cal_value[0] = 0;  joybut_pressed = 0; break;
  }

  // Decode Data[1]
  switch (Data[1]) {
      case 35: cal_value[1] = -5; joybut_pressed = 1; break;
      case 34: cal_value[1] = -4; joybut_pressed = 1; break;
      case 33: cal_value[1] = -3; joybut_pressed = 1; break;
      case 32: cal_value[1] = -2; joybut_pressed = 1; break;
      case 31: cal_value[1] = -1; joybut_pressed = 1; break;
      case 20: cal_value[1] = 0;  joybut_pressed = 1; break;
      case 21: cal_value[1] = 1;  joybut_pressed = 1; break;
      case 22: cal_value[1] = 2;  joybut_pressed = 1; break;
      case 23: cal_value[1] = 3;  joybut_pressed = 1; break;
      case 24: cal_value[1] = 4;  joybut_pressed = 1; break;
      case 25: cal_value[1] = 5;  joybut_pressed = 1; break;
      case 15: cal_value[1] = -5; joybut_pressed = 0; break;
      case 14: cal_value[1] = -4; joybut_pressed = 0; break;
      case 13: cal_value[1] = -3; joybut_pressed = 0; break;
      case 12: cal_value[1] = -2; joybut_pressed = 0; break;
      case 11: cal_value[1] = -1; joybut_pressed = 0; break;
      case 0:  cal_value[1] = 0;  joybut_pressed = 0; break;
      case 1:  cal_value[1] = 1;  joybut_pressed = 0; break;
      case 2:  cal_value[1] = 2;  joybut_pressed = 0; break;
      case 3:  cal_value[1] = 3;  joybut_pressed = 0; break;
      case 4:  cal_value[1] = 4;  joybut_pressed = 0; break;
      case 5:  cal_value[1] = 5;  joybut_pressed = 0; break;
      default: cal_value[1] = 0;  joybut_pressed = 0; break;
  }

  // Display calibrated values
  sprintf(buff, "%3d", cal_value[0]);
  LCD_DrawString(100, 60, (uint8_t *)buff);

  sprintf(buff, "%3d", cal_value[1]);
  LCD_DrawString(160, 60, (uint8_t *)buff);

  sprintf(buff, "B:%3d", Data[2]);
  LCD_DrawString(20, 120, (uint8_t *)buff);

  sprintf(buff, "joybut:%3d", joybut_pressed);
  LCD_DrawString(20, 90, (uint8_t *)buff);

  HAL_Delay(50);


  if(Data[2] == 3){
    if (Ov7725_vsync == 2)
	{
	  FIFO_PREPARE;
	  ImagDisp();			
	  Ov7725_vsync = 0;
	}
  }


  //Handle_USART1_Request();                                                                  // to f4
    send_normal_data();
    HAL_Delay(100); // Send normal data every 100ms (adjust as needed)

    if (Data[2] == 3) 
    {
      send_photo();
      HAL_Delay(100);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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


                                                                                                               //to F4
/*
void Handle_USART1_Request(void)
{
  uint8_t request;
  if (HAL_UART_Receive(&huart1, &request, 1, 10) == HAL_OK)
  {
    if (request == 0xA5) // Normal data request
    {
      uint8_t tx_buf[5];
      tx_buf[0] = (uint8_t)cal_value[0];
      tx_buf[1] = (uint8_t)cal_value[1];
      tx_buf[2] = Data[2];
      tx_buf[3] = (uint8_t)joybut_pressed;
      tx_buf[4] = 0x5A; // stop bit
      HAL_UART_Transmit(&huart1, tx_buf, 5, 100);
    }
    else if (request == 0xB5) // Photo request
    {
      if (Data[2] == 3)
      {
        // Send a header first
        uint8_t header[2] = {0xB5, 0xAA};
        HAL_UART_Transmit(&huart1, header, 2, 100);

        FIFO_PREPARE;


        uint8_t block[BLOCK_SIZE];
        uint32_t sent = 0;

        while (sent < PHOTO_SIZE) {
          uint32_t to_send = ((PHOTO_SIZE - sent) > BLOCK_SIZE) ? BLOCK_SIZE : (PHOTO_SIZE - sent);
          for (uint32_t i = 0; i < to_send; i++) {
            uint16_t Camera_Data;
            READ_FIFO_PIXEL(Camera_Data);
            block[i] = (uint8_t)(Camera_Data & 0xFF);
          }
          HAL_UART_Transmit(&huart1, block, to_send, 1000);
          sent += to_send;
        }

        // Send a footer
        uint8_t footer[2] = {0xAA, 0xB5};
        HAL_UART_Transmit(&huart1, footer, 2, 100);
      }
    }
  }
}
*/

void send_normal_data(void)
{
    uint8_t tx_buf[4];
    tx_buf[0] = (uint8_t)cal_value[0];
    tx_buf[1] = (uint8_t)cal_value[1];
    tx_buf[2] = Data[2];
    tx_buf[3] = (uint8_t)joybut_pressed;
    //tx_buf[4] = 0x5A; // stop ender
    HAL_UART_Transmit(&huart1, tx_buf, 4, 100);		//5
}

void send_photo(void)
{
    // Send a header first
    uint8_t header[2] = {0xB5, 0xAA};
    HAL_UART_Transmit(&huart1, header, 2, 100);

    FIFO_PREPARE;

    uint8_t block[BLOCK_SIZE];
    uint32_t sent = 0;

    while (sent < PHOTO_SIZE) {
        uint32_t to_send = ((PHOTO_SIZE - sent) > BLOCK_SIZE) ? BLOCK_SIZE : (PHOTO_SIZE - sent);
        for (uint32_t i = 0; i < to_send; i++) {
            uint16_t Camera_Data;
            READ_FIFO_PIXEL(Camera_Data);
            block[i] = (uint8_t)(Camera_Data & 0xFF);
        }
        HAL_UART_Transmit(&huart1, block, to_send, 1000);
        sent += to_send;
    }

    // Send a footer
    uint8_t footer[2] = {0xAA, 0xB5};
    HAL_UART_Transmit(&huart1, footer, 2, 100);
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
