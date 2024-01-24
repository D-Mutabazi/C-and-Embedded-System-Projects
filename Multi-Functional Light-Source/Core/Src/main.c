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
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define adc_buf_len 4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern uint8_t middle_button_pressed ;
extern  uint8_t right_button_pressed ;
extern uint8_t left_button_pressed ;
int button_count= 0 ;

uint8_t start_up = 1 ;
uint8_t em_count = 0 ;
uint8_t em_default = 1 ;

uint16_t adc_buf[adc_buf_len] ;  // stores adc values
uint16_t raw_adc_dma_val = 0 ;   // unscaled adc value
char adc_val[10] ;               // buffer to store val
int sum  = 0 ;
uint8_t adc_val_capture = 1 ;    // we capture adc current value if set
uint16_t adc_val_snapshot ;      // previous snapshot of adc value
uint8_t adc_conv_complete = 0 ;  // when buffer filled DMA interrrupt triggered and buffer ready for processin
double adc_scale_up =1.0314861460957178841309823677582 ;
uint16_t scaled_adc_val ;
uint8_t update_led_via_ADC = 0 ;  // WHITE led to be updated via ADC

uint8_t LED_ON = 0 ;   // white LED set on of off
uint16_t LED_intensity = 1 ;

/**Emergency mode*/
// STROBE
uint16_t strobe_delay =  300 ; // UNIT = ms f = 0.9765HZ (default on time)
uint32_t strobe_ticks = 0 ; // strobe delay count
uint8_t led_strobe_on = 0 ; // flag to alternate between on/off in strobe
uint32_t timePassed = 0 ;
//SOS morse
char letter[]={'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S'
					,'T','U','V','W','X','Y','Z', '1','2','3','4','5','6','7','8','9','0'} ;

char *morse_symbol[]={". -","- . . .","- . - .","- . ." ,"." ,". . - .","- - .",". . . .",". .",". - - -",
				"- . -",". - . ." ,"- -" ,"- ." ,"- - -" ,". - - .","- - . -",". - .",". . .","-",
				". . -",". . . -",". - -","- . . -","- . - -","- - . .", ". - - - -",". . - - -",
				". . . - -",". . . . -",". . . . .","- . . . .","- - . . .","- - - . .",
				"- - - - .","- - - - -"};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void MF_mode_LED() ;
void ME_mode_LED();
void MM_mode_LED();

void adc_dma_val_processing() ;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t studentNum[13]="#:23765518:$\n" ;
uint8_t recvd_char[1];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	//transmit recvd characer
	HAL_UART_Transmit(&huart2, recvd_char, 1, 50) ;

	// recieve character
	HAL_UART_Receive_IT(&huart2, recvd_char, 1);
}

/**
 * scale up transmit ADC values
 */

void adc_dma_val_processing(){

	if(adc_conv_complete == 1){

		adc_conv_complete =0  ;
		sum = 0 ;
		for(int i = 0 ; i < adc_buf_len ; i++){
			sum += adc_buf[i] ;
		}
		raw_adc_dma_val =(uint16_t)(sum/adc_buf_len) ;

		scaled_adc_val = (uint16_t)raw_adc_dma_val*adc_scale_up ; //adc scaled to max =4095

		//capture previous adc cal
		if(adc_val_capture == 1){
			adc_val_snapshot = scaled_adc_val ;
			adc_val_capture = 0 ;
		}
	}

	// if ADC movement significant update LED intensity
	if(abs(scaled_adc_val - adc_val_snapshot) >15){
	  update_led_via_ADC = 1 ;
	}

//	 WHITE LED intensity
	LED_intensity =(float)(scaled_adc_val)*(512.0/4095.0)  ;


}


void system_state_update(){
	 if(left_button_pressed ==1 ){
		 button_count++ ;
		 if(button_count > 2){
			 button_count = 0 ;
		 }
		 update_led_via_ADC =  0 ; // don't read ADC by default in next state
		 // snapshot of ADC taken in next state
		 if(adc_conv_complete == 1){
			 adc_val_capture =1  ; // capture ADC value
		 }

		 // LED off at each new state
		 htim2.Instance->CCR1 = 0;
		 left_button_pressed = 0 ;
	 }
}

/**
 * Updates system state after right button pressed in emergency mode
 */
void right_button_state_update(){
	if(button_count == 1){
		if(right_button_pressed){
			right_button_pressed = 0 ;

			update_led_via_ADC = 0 ; // dont read adc by default in next state

			 em_count++ ;

			 if(em_count>2){
				 em_count = 0;
			 }
		}
	}else if( button_count != 1 && right_button_pressed){
		right_button_pressed = 0; //do not read right button presses triggered
								  // in other states except emergency mode
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(200);
  HAL_UART_Transmit(&huart2, studentNum, 13, 150); //transmit student number

  HAL_UART_Receive_IT(&huart2, recvd_char, 1); //recv character input

  //Startup ADC
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, adc_buf_len) ;

  // TIM2_CH1 start PWM
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1) ;

  strobe_ticks  = HAL_GetTick() ;

  //
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // left button press to update system state (MF -> ME -> MM)
	  system_state_update() ;
	  //run adc and capture previous snapshot of ADC value and adc movement processing
	  adc_dma_val_processing();


	 // system state
	 if(button_count == 0 || start_up == 1){
		 start_up = 0 ; //for default MF state

		 MF_mode_LED() ; // sets the corresponding mode LED
		 em_count=0;     // reset the emergency mode count
		 em_default = 1; // to re-enter the EM state

		 // Middle button press -> LED ON / OFF
		 if(middle_button_pressed == 1){
			 LED_ON = !LED_ON ;  // turns the LED on of off

			 if(LED_ON == 1){
				 htim2.Instance->CCR1 = 1 ; // LED ON
			 }else if(LED_ON ==0){
				 htim2.Instance->CCR1 = 0 ; //LED OFFS
			 }
			 middle_button_pressed = 0 ;
		 }

		 // if LED_ON and SLIDER MOVED -> updated LED intensity
		if(LED_ON == 1 && update_led_via_ADC == 1){
		  htim2.Instance->CCR1 =  LED_intensity ; // vary the duty cycle of the LED [1:512]
		  sprintf(adc_val, "%d\n", scaled_adc_val) ;
		  HAL_UART_Transmit_IT(&huart2, (uint8_t*)adc_val, strlen(adc_val)) ;
		}

	 }else if(button_count == 1){// right button system state updated
		 ME_mode_LED() ; // sets the corresponding modes LED


		 if(update_led_via_ADC == 1){
			 sprintf(adc_val, "%d\n", scaled_adc_val) ;
			 HAL_UART_Transmit_IT(&huart2, (uint8_t*)adc_val, strlen(adc_val)) ;
		 }
	 }else{
		 if(button_count == 2){

			 em_count=0; // reset the emergency mode state
			 em_default = 1; // to re-enter EM state

			 MM_mode_LED() ; //sets the corresponding modes LED
		 }
	 }

//	 strobe_ticks = HAL_GetTick() ;
	 // right button state update
	 right_button_state_update() ;
	 //EMERGENCY MODES
	  if(button_count ==1 ){

		 if(em_count == 0 || em_default ==1){ //strobe wit default intensity
			 em_default = 0 ; //default state reached
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);


			 if(LED_ON){//LED_on =?
				 // default delay 512ms
				 timePassed =HAL_GetTick() - strobe_ticks ;
				 // time passed >512
				 if( timePassed >= strobe_delay && led_strobe_on == 0){
					 led_strobe_on =1 ;
					 htim2.Instance->CCR1 = 0 ;
				 }
				 // time Passed > 1024
				 if(timePassed >= 1024 && led_strobe_on == 1){
					 strobe_ticks =  HAL_GetTick() ; // update current time
					 led_strobe_on = 0 ;

					 htim2.Instance->CCR1 =256;
				 }
			 }
		 }
		 else if(em_count ==1){ // SOS MOSRE

			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		 }
		 else{
			 if(em_count == 2){ // CUSTOM MORSE
				 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		 }
		}
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 36;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 512;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_D3_Pin|LED_D4_Pin|LED_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_D3_Pin LED_D4_Pin LED_D5_Pin */
  GPIO_InitStruct.Pin = LED_D3_Pin|LED_D4_Pin|LED_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_D2_Pin */
  GPIO_InitStruct.Pin = LED_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_D2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
// FUNCTIONS
void MF_mode_LED(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}

void ME_mode_LED(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

void MM_mode_LED(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}


// adc buffer filled by dma circular sampling
// data should not be processed in the interrupt, it makes rest of the
// progam inaccessible
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	adc_conv_complete = 1 ;

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
