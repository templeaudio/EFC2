/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
#include "usbd_conf.h"
#include "usbd_desc.h"
#include "stm32l4xx_hal.h"
#include "usbd_core.h"
//#include "usbd_desc.h"
//#include "usbd_hid.h"
//#include "stm32l4xx_nucleo_32.h"
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef struct
{
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE1;
	uint8_t KEYCODE2;
	uint8_t KEYCODE3;
	uint8_t KEYCODE4;
	uint8_t KEYCODE5;
	uint8_t KEYCODE6;
} keyboardHID;

keyboardHID keyboardhid = {0,0,0,0,0,0,0,0};
uint8_t kbd1[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t kbdclear[8] = {0,0,0,0,0,0,0,0};
USBD_HandleTypeDef USBD_Device;
__IO uint32_t detect = 0;
extern PCD_HandleTypeDef hpcd;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* Init Device Library */
  USBD_Init(&USBD_Device, &FS_Desc, 0);

  /* Register the HID class */
  USBD_RegisterClass(&USBD_Device, &USBD_HID);

  /* Start Device Process */
  USBD_Start(&USBD_Device);

  //uint8_t ctrl_button = 0;
  //uint8_t shift_button = 0;
  //uint8_t alt_button = 0;
  //uint8_t super_button = 0;
  //uint8_t space_button = 0;

  uint8_t ctrl_state = 0;
  uint8_t shift_state = 0;
  uint8_t alt_state = 0;
  uint8_t super_state = 0;

  uint8_t ctrl_state_prev = 0;
  uint8_t shift_state_prev = 0;
  uint8_t alt_state_prev = 0;
  uint8_t super_state_prev = 0;

  uint32_t ctrl_last_pressed = 0;
  uint32_t shift_last_pressed = 0;
  uint32_t alt_last_pressed = 0;
  uint32_t super_last_pressed = 0;
  uint32_t latching_last_pressed = 0;
  //uint8_t space_state = 0;

  uint8_t kbd_buffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint32_t last_USB_send = 0;
  uint8_t latching_state = 1;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  	  {
	  ctrl_state_prev = ctrl_state;
	  shift_state_prev = shift_state;
	  alt_state_prev = alt_state;
	  super_state_prev = super_state;
	  kbd_buffer[2] = 0x0;

	  //--------------
	  //Latching Button
	  //--------------

	  //A6

	  if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == 0) && (HAL_GetTick() - latching_last_pressed > 150))
	  {

		  latching_state = !latching_state;
		  latching_last_pressed = HAL_GetTick();
		  ctrl_state = 0;
		  shift_state = 0;
		  alt_state = 0;
		  super_state = 0;


	  }

	  //--------------
	  //Control Button
	  //--------------

	  //D4

	  if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0) && (HAL_GetTick() - ctrl_last_pressed > 150))
	  {

		  ctrl_state = !ctrl_state;
		  ctrl_last_pressed = HAL_GetTick();
		  if (latching_state == 0)
		  {
			  kbd_buffer[0] = 0x01;
			  USBD_HID_SendReport(&USBD_Device, kbd_buffer, 8);
			  HAL_Delay(50); //debounce
			  while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0); //wait for button to be released
			  kbd_buffer[0] = 0x00;
			  USBD_HID_SendReport(&USBD_Device, kbd_buffer, 8);
			  HAL_Delay(50); //debounce

			  ctrl_state = !ctrl_state; //undo
		  }

	  }

	  //--------------
	  //Shift Button
	  //--------------

	  //D9

	  if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 0) && (HAL_GetTick() - shift_last_pressed > 150))
	  {

		  shift_state = !shift_state;
		  shift_last_pressed = HAL_GetTick();
		  if (latching_state == 0)
		  {
			  kbd_buffer[0] = 0x02;
			  USBD_HID_SendReport(&USBD_Device, kbd_buffer, 8);
			  HAL_Delay(50); //debounce
			  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 0); //wait for button to be released
			  kbd_buffer[0] = 0x00;
			  USBD_HID_SendReport(&USBD_Device, kbd_buffer, 8);
			  HAL_Delay(50); //debounce

			  shift_state = !shift_state; //undo
		  }

	  }

	  //--------------
	  //Alt Button
	  //--------------

	  //D6

	  if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0) && (HAL_GetTick() - alt_last_pressed > 150))
	  {

		  alt_state = !alt_state;
		  alt_last_pressed = HAL_GetTick();
		  if (latching_state == 0)
		  {
			  kbd_buffer[0] = 0x04;
			  USBD_HID_SendReport(&USBD_Device, kbd_buffer, 8);
			  HAL_Delay(50); //debounce
			  while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0); //wait for button to be released
			  kbd_buffer[0] = 0x00;
			  USBD_HID_SendReport(&USBD_Device, kbd_buffer, 8);
			  HAL_Delay(50); //debounce

			  alt_state = !alt_state; //undo
		  }

	  }

	  //--------------
	  //Super Button
	  //--------------

	  //A1

	  if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 0) && (HAL_GetTick() - super_last_pressed > 150))
	  {

		  super_state = !super_state;
		  super_last_pressed = HAL_GetTick();
		  if (latching_state == 0)
		  {
			  kbd_buffer[0] = 0x08;
			  USBD_HID_SendReport(&USBD_Device, kbd_buffer, 8);
			  HAL_Delay(50); //debounce
			  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 0); //wait for button to be released
			  kbd_buffer[0] = 0x00;
			  USBD_HID_SendReport(&USBD_Device, kbd_buffer, 8);
			  HAL_Delay(50); //debounce

			  super_state = !super_state; //undo
		  }

	  }

	  //--------------
	  //Space Button
	  //--------------

	  //A5

	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == 0)
	  {
		  kbd_buffer[2] = 0x2c;
		  USBD_HID_SendReport(&USBD_Device, kbd_buffer, 8);
		  HAL_Delay(50); //debounce
		  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == 0); //wait for button to be released
		  kbd_buffer[2] = 0x00;
		  USBD_HID_SendReport(&USBD_Device, kbd_buffer, 8);
		  HAL_Delay(50); //debounce

	  }

	  if ((ctrl_state != ctrl_state_prev) || (shift_state != shift_state_prev) || (alt_state != alt_state_prev) || (super_state != super_state_prev))
	  {
			kbd_buffer[0] = ctrl_state + shift_state*2 + alt_state*4 + super_state*8;

			if((HAL_GetTick() - last_USB_send) > 10)
			{
				USBD_HID_SendReport(&USBD_Device, kbd_buffer, 8);
				last_USB_send = HAL_GetTick();
				//if(latching_state == 0)
				//{
				//	HAL_Delay(50);
				//	kbd_buffer[0] = 0x00;
				//  USBD_HID_SendReport(&USBD_Device, kbd_buffer, 8);
				//  HAL_Delay(50);
				//}
			}
	  }
		  //send(ctrl_state, shift_state, alt_state, super_state, ctrl_state_prev, shift_state_prev, alt_state_prev, super_state_prev);
	} //end of while


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA3 PA4 PA5
                           PA6 PA7 PA8 PA9
                           PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB4 PB5 PB6
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

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
