/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN CODE */ //LM75A SETUP
#define LM75A_ADDR 0X4C
#define PLUS 1
#define MINUS 0
#define DIGIT_PLUS 10
#define DIGIT_MINUS 11
#define DECIMAL_FIVE 1
#define DECIMAL_ZERO 0
#define DOT 1
#define NOT_DOT 0
#define I2C_WRITE 0
#define I2C_READ 1
#define LM75A_CONFIGURATTION_REG_ADDR 0X01
#define LM75A_TEMPERATURE_REG_ADDR 0X00

/* USER CODE BEGIN 0 */
char number[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x27, 0x7f, 0x6f, 0X00, 0X40};
uint8_t i2c1_buffer[2]; // write and read buffer
int temperature;        // temp memory
uint32_t count = 0;
void fnd_display(int num, int position, int time_ms, int dot_flag)
{
	int i;
	switch (position) {
	case 3:
		HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, SET);
		HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin, RESET);
		HAL_GPIO_WritePin(DIG3_GPIO_Port, DIG3_Pin, RESET);
		HAL_GPIO_WritePin(DIG4_GPIO_Port, DIG4_Pin, RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, RESET);
		HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin, SET);
		HAL_GPIO_WritePin(DIG3_GPIO_Port, DIG3_Pin, RESET);
		HAL_GPIO_WritePin(DIG4_GPIO_Port, DIG4_Pin, RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, RESET);
		HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin, RESET);
		HAL_GPIO_WritePin(DIG3_GPIO_Port, DIG3_Pin, SET);
		HAL_GPIO_WritePin(DIG4_GPIO_Port, DIG4_Pin, RESET);
		break;
	case 0:
		HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, RESET);
		HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin, RESET);
		HAL_GPIO_WritePin(DIG3_GPIO_Port, DIG3_Pin, RESET);
		HAL_GPIO_WritePin(DIG4_GPIO_Port, DIG4_Pin, SET);
		break;
	}
  	for (i=0; i<12; i++)
	{
		if((number[num] & (1<<i))!=0)
		{
			switch (i)
			{
			case 0 : HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET); break;
			case 1 : HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET); break;
			case 2 : HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET); break;
			case 3 : HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET); break;
			case 4 : HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET); break;
			case 5 : HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET); break;
			case 6 : HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET); break;
			case 7 : HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET); break;
			}
		}else
			{
				switch(i)
				{
				case 0 : HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET); break;
				case 1 : HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET); break;
				case 2 : HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET); break;
				case 3 : HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET); break;
				case 4 : HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET); break;
				case 5 : HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET); break;
				case 6 : HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET); break;
				case 7 : HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_RESET); break;
				}
			}
		}
    if(dot_flag ==DOT)
    HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
	HAL_Delay(time_ms);
	}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t curtick = 0, prevTick = 0;
  uint32_t num3, num2, num1, num0;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  *i2c1_buffer = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, ((LM75A_ADDR << 1) | I2C_WRITE), LM75A_CONFIGURATTION_REG_ADDR, 8, i2c1_buffer, 1, 100);
  HAL_I2C_Mem_Read(&hi2c1, ((LM75A_ADDR << 1) | I2C_WRITE), LM75A_TEMPERATURE_REG_ADDR, 8, i2c1_buffer, 2, 100);

  void display_temperature(uint8_t *buffer);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(HAL_I2C_Master_Receive(&hi2c1,((LM75A_ADDR<<1)|I2C_WRITE), i2c1_buffer, 2, 100) == HAL_OK)
          display_temperature(i2c1_buffer);
        
    /* USER CODE END WHILE */
    if (((curtick = HAL_GetTick()) - prevTick) >= 1000)
    {
      prevTick = curtick;
      if (count == 0)
      {
        if (count != 0)
        {
        	count--;
        	count = 0;
        }
      }
      else
    	  count--;
    }
    num3 = count / 1000;         // 1000
    num2 = (count % 1000) / 100; // 100
    num1 = (count % 100) / 10;   // 10
    num0 = count % 10;           // 1
  }
  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void display_temperature(uint8_t *buffer)
{
    int temp, sign;
    uint8_t decimal;
    temp = (*buffer << 8) | *(buffer + 1);
    if ((temp & (1 << 15)))
    {
        sign = MINUS;
        temp = (~temp & 0x7FE0) + 0x20;
    }
    else
    {
        sign = PLUS;
    }

    decimal = (temp & (1 << 7)) ? DECIMAL_ZERO : DECIMAL_FIVE; // 소수 부분 설정

    temp >>= 7; // 7 비트만큼 오른쪽으로 시프트하여 정수부분만 남기기

    // 정수부분과 소수부분 나누기
    int integer_part = temp >> 1;
    int fractional_part = (temp & 0x01) * 5; // 소수 부분은 마지막 비트에 있으므로 1 비트만큼의 값을 곱하여 계산

    // 온도를 표시
    if (sign == PLUS)
        fnd_display(DIGIT_PLUS, 3, 4, NOT_DOT);
    else
        fnd_display(DIGIT_MINUS, 3, 4, NOT_DOT);

    fnd_display(integer_part / 10, 2, 4, NOT_DOT); // 십의 자리 표시
    fnd_display(integer_part % 10, 1, 4, DOT);     // 일의 자리 표시

    // 소수 부분을 표시
    fnd_display(fractional_part, 0, 4, NOT_DOT);
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, DIG1_Pin|DIG2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DIG4_Pin|DIG3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, A_Pin|B_Pin|C_Pin|D_Pin
                          |E_Pin|F_Pin|G_Pin|DP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIG1_Pin DIG2_Pin */
  GPIO_InitStruct.Pin = DIG1_Pin|DIG2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin DIG4_Pin DIG3_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|DIG4_Pin|DIG3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : A_Pin B_Pin C_Pin D_Pin
                           E_Pin F_Pin G_Pin DP_Pin */
  GPIO_InitStruct.Pin = A_Pin|B_Pin|C_Pin|D_Pin
                          |E_Pin|F_Pin|G_Pin|DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
