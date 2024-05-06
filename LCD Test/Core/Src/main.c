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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//if witing to instruction register r2 = 0
#define clear_lcd 0x01 //clear display
#define ret_home 0x02  //set ddram addres to 00
#define entry_mode_set 0x06 //set the cursor to increment
#define display_and_cursor_on_off 0x0F  //set diplay on, cursor showing ,and blinking
#define function_set 0x2A // 4 bit mode, 2 line display, 5x8 font - execute first, to ensire executing in 4 bit mode,

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t set_enable = 1 ;
uint32_t previous_time = 0;


    // Toggle enable (E) signal - non blocking delay code
//    if(HAL_GetTick() - previous_time >= 1 && set_enable ==1 ){
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // E = 1
//        set_enable = 0;
//        previous_time = HAL_GetTick() ;
//    }
//    else if(HAL_GetTick() - previous_time >=2 && set_enable ==0){
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // E = 0
//        set_enable = 1;
//        previous_time = HAL_GetTick() ;
//    }


/**
 * This function writes data to the lcd data register
 * The initial mistake made was I was writing bits in the inverse location
 */
void lcd_data(uint8_t dataP){

	// Write the nibble (4 bits) to DB4-DB7 pins - HIGH NIBBLE
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (dataP >> 4) & 0x01); // DB4
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, (dataP >> 5) & 0x01); // DB5
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, (dataP >> 6) & 0x01); // DB6
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, (dataP>>7) & 0x01);         // DB7

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // E = 1

	// Set RS and RW appropriately
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // RS = 0 (for instruction)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // RW = 0 (for write)

	HAL_Delay(1) ;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // E = 0

	// Write the nibble (4 bits) to DB4-DB7 pins - LOWER NIBBLE
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (dataP >> 0) & 0x01); // DB4
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, (dataP >> 1) & 0x01); // DB5
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, (dataP >> 2) & 0x01); // DB6
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, (dataP>>3) & 0x01);         // DB7

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // E = 1

	// Set RS and RW appropriately
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // RS = 0 (for instruction)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // RW = 0 (for write)

	HAL_Delay(1) ;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // E = 0

}

/**
 * This function write data to the instruction register
 * It is to be used for configuring the lcd
 * The initial mistake made was I was writing bits in the inverse location
 */
void lcd_cmd(uint8_t dataP){
	// Write the nibble (4 bits) to DB4-DB7 pins - HIGH NIBBLE
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (dataP >> 4) & 0x01); // DB4
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, (dataP >> 5) & 0x01); // DB5
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, (dataP >> 6) & 0x01); // DB6
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, (dataP>>7) & 0x01);    // DB7

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // E = 1

	// Set RS and RW appropriately
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // RS = 0 (for instruction)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // RW = 0 (for write)

	HAL_Delay(1) ;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // E = 0

	// Write the nibble (4 bits) to DB4-DB7 pins - LOWER NIBBLE
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (dataP >> 0) & 0x01); // DB4
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, (dataP >> 1) & 0x01); // DB5
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, (dataP >> 2) & 0x01); // DB6
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, (dataP>>3) & 0x01);    // DB7

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // E = 1

	// Set RS and RW appropriately
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // RS = 0 (for instruction)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // RW = 0 (for write)

	HAL_Delay(1) ;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // E = 0

}

/**
 * Function takes as input the row and columns number
 * 16x2 lcd display used , thus first row = 0 or second row =1
 * columns take on values from 0-15 , with column 0 being the first column
 * The address for first row is 0x00 and second row is 0x40
 * 0x80 - refers to the register value on the lcd
 */

void set_cursor_position(uint8_t row, uint8_t col){
	//postion on first row - taking modulus ensures that we write in either first or second row
	if(row % 2 == 0){
		lcd_cmd(0x80 | col) ;
	}

	//second row
	else if(row % 2 == 1){
		lcd_cmd(0x80 | 0x40 | col) ;
	}
}

/**
 * This function writes data to the lcd
 * input string of length<16, otherwise lcd overflows, but 17th letter and beyond wont be seen
 */

void write_data(char *string){

	//There can be a maximum of 16 characters on any row on the lcd
	for(int i = 0 ; i<16 ; i++){
		//if end of string reach leave function
		if( *string == '\0'){
			break ;
		}
		//if end of string not reached, print value to lcd
		else {
			lcd_data(*string) ;
			string++ ; //move on to the next character

		}
	}

}

/**
 * To interface with the lcd it has to undergo an initialisation sequence(pg. 46 of the HD447 LCD controller ... datasheet)
 *
 */
void lcd_init(){
	lcd_cmd(0x33) ;
	lcd_cmd(0x32) ;
	lcd_cmd(0x28) ;

	lcd_cmd(0x01);  //clear display
	lcd_cmd(0x0C) ;
	lcd_cmd(0x06);

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(20) ; //based on data sheet at least 15ms delay
  lcd_init() ;

  //display on cursor - reposition
  lcd_cmd(0x0F);

  //reposition cursor
  set_cursor_position(0, 1);

  //write data
  char name[] = "Alhamdulillah!" ;
  write_data(name);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA11 PA12 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
