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
#include "string.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LEN 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t studentNum[13] = "&_23765518_*\n" ;

char char_rcvd[1] = " " ;

extern uint8_t g_left_button_pressed  ;
extern uint8_t g_right_button_pressed ;
extern uint8_t g_top_button_pressed ;
extern uint8_t g_bottom_button_pressed ;
extern uint8_t g_middle_button_pressed ;


//LM235 - TEMP Sens
uint16_t g_raw = 0;
char g_msg[10];
double g_temp = 0 ;
double g_vin = 0 ;
uint16_t g_temp_in_deg = 0 ;
char g_temperature[3]= {} ;

//LMTO1 senso
extern uint32_t pulse_count ;
uint8_t Tsp_temp_ready = 0 ;
uint32_t g_pulse_window_period =  0 ;
uint16_t g_num_pulses = 0;
double g_TO1_temp = 0;
uint16_t g_temp_in_deg_dig_sens = 0;

// SYSTEM state machine variables
char g_system_config[17] = {} ;
uint8_t g_byte_count = 0 ;
uint8_t g_config_command_rcvd = 0;    // check for when config recvd

// EN measure
uint8_t g_EN_measure= 0;
uint32_t g_time_passed = 0 ;
uint8_t g_LED_D3_ON  =0 ;   // LED D3 state initially off
uint8_t g_EN_config_command_rcvd = 0;

// EN measure
uint8_t g_SP_measure= 0;
uint8_t g_LED_D2_ON  =0 ;   // LED D2 state initially off
uint8_t g_SP_config_command_rcvd = 0;


char system_state_transmit[17] = {} ;
uint8_t g_transmit_system_state = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

uint16_t get_adc_value_and_celsius_temperature() ;
void store_temp_in_string(uint16_t temperature, char temp[], int len) ;
void system_state_update() ;
void flash_led_d3() ;
void flash_led_d2() ;
void store_system_state_in_buffer(char analog_temp[], char dig_temp[], char system_state[], uint8_t len_of_sys_arr );

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	g_system_config[g_byte_count] = char_rcvd[0] ;
	g_byte_count++ ;

	if(char_rcvd[0] == '\n'){
		if(g_byte_count == 7){
			g_config_command_rcvd = 1;
			// check for SP or EN command recvd
		}
		else{
			// remove for next DEMO
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)"Invalid command sent\n", 21);
			// DO NOTHING: NO STATE UPDATE IN THE CASE OF COMMAND NOT IN THE APPROPRIATE ORDER
		}

		g_byte_count =0 ;
	}

	//re-prime receiver
	HAL_UART_Receive_IT(&huart2, (uint8_t*)char_rcvd, 1) ;

}

/**
 * function starts the adc, waits for conversion
 * Then converts value to degrees
 */

uint16_t get_adc_value_and_celsius_temperature(){

	HAL_ADC_Start(&hadc1) ;
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	g_raw  = HAL_ADC_GetValue(&hadc1) ;

	g_vin = g_raw*(3.3/4095.0) ; // input voltage
	g_temp = g_vin*100 - 273.15 ; // cast to 16 bit uint

	return g_temp ;
}

void store_temp_in_string(uint16_t temperature, char temp[], int len){

	for(int i= 0 ; i < len ; i++){
		switch(i){
			case 0:
				temp[0]  = (temperature/100) + 48 ;

				break;
			case 1:
				temp[1] = (temperature - (temperature/100)*100 )/10 + 48 ;

				break;
			case 2:
				temp[2] = (temperature - ((temperature/10)*10) ) + 48 ;

				break;
			default:
				break;

		}
	}
}

/**
 * This function will update the system state based on the received UART command
 * or top button press
 */
void system_state_update(){

	//CHECK FOR TYPE OF MEASUREMENT
	if( g_config_command_rcvd == 1){
		g_config_command_rcvd = 0;

		if( g_system_config[2]=='E' && g_system_config[3] == 'N'){
			// EN measure comand
			g_EN_config_command_rcvd =1 ;
			//stop SP measure
			g_SP_config_command_rcvd =0 ;

		}

		else if(g_system_config[2]=='S' && g_system_config[3] == 'P'){
			// SP command
			g_SP_config_command_rcvd =1 ;
			//stop EN command
			g_EN_config_command_rcvd =0;

		}

		else{
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)"Invalid Command\n", 16);
		}
	}

	// Environment Measure - And not measuring SP
	if(g_top_button_pressed  == 1 && g_EN_config_command_rcvd ==0 && g_SP_config_command_rcvd ==0 && (g_SP_measure == 0 || g_SP_measure ==2)){
		g_top_button_pressed = 0;

		g_EN_measure++  ;

		if(g_EN_measure >2 ){
			g_EN_measure = 1;
		}
	}
	else if(g_top_button_pressed ==0  && g_EN_config_command_rcvd == 1 && g_SP_config_command_rcvd ==0  && (g_SP_measure == 0 || g_SP_measure ==2)){
		g_EN_config_command_rcvd = 0;
		if(g_system_config[0]== '&' && g_system_config[1 ]== '_' && g_system_config[2]=='E' && g_system_config[3] == 'N' &&g_system_config[4] =='_'&& g_system_config[5] =='*' &&  g_system_config[6] =='\n' ){
			if(g_EN_measure == 0){
				g_EN_measure = 1;
			}
			else if(g_EN_measure == 1){
				g_EN_measure = 2;

			}
			else{
				if(g_EN_measure ==2){
					g_EN_measure = 1;
				}
			}
		}
		//else block to not update g_EN_measure if incorrent command revcd
		else{
			g_EN_measure =  g_EN_measure ;
		}
	}



	//SP Measure
	if(g_bottom_button_pressed  == 1  && g_EN_config_command_rcvd == 0 && g_SP_config_command_rcvd ==0  && (g_EN_measure == 0 || g_EN_measure ==2)){
			g_bottom_button_pressed = 0;

			g_SP_measure++  ;

			if(g_SP_measure >2 ){
				g_SP_measure = 1;
			}
		}
		else if(g_bottom_button_pressed ==0  && g_EN_config_command_rcvd == 0 && g_SP_config_command_rcvd ==1 && (g_EN_measure == 0 || g_EN_measure ==2)){
			g_SP_config_command_rcvd = 0;
			if(g_system_config[0]== '&' && g_system_config[1 ]== '_' && g_system_config[2]=='S' && g_system_config[3] == 'P' &&g_system_config[4] =='_'&& g_system_config[5] =='*' &&  g_system_config[6] =='\n' ){
				if(g_SP_measure == 0){
					g_SP_measure = 1;
				}
				else if(g_SP_measure == 1){
					g_SP_measure = 2;

				}
				else{
					if(g_SP_measure ==2){
						g_SP_measure = 1;
					}
				}
			}
			//else block to not update g_EN_measure if incorrent command revcd
			else{
				g_SP_measure =  g_SP_measure ;
			}
		}
}

/**
 * Interrupt triggered by input capture
 */
uint32_t current_value = 0  ;
uint32_t g_time_between_pulses =  0;
uint8_t g_new_pulse = 0;
uint16_t g_lmt01_sens_temp =  0 ;
char dig_sens_temp[3] = {};
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_IC_CaptureCallback could be implemented in the user file
   */
  if(htim->Instance == TIM2){
	  g_new_pulse = 1;
	  g_time_between_pulses =  __HAL_TIM_GET_COUNTER(&htim2)  - current_value;  //Time between rising edges

	  if( g_time_between_pulses<13){
		  current_value = __HAL_TIM_GET_COUNTER(&htim2) ;
		  pulse_count++ ;

	  }else{
		  g_TO1_temp = (pulse_count/4096.0)*256 - 50 ; //calculate new temp - BUG Fixed: 4094 changed to 4096
		 current_value = __HAL_TIM_GET_COUNTER(&htim2) ;
		 pulse_count = 0;
	  }
  }
}

/**
 * Function flashed LED D3 at specified interval of 50 ms
 */
void flash_led_d3(){
	if(HAL_GetTick() - g_time_passed >= 50 && g_LED_D3_ON == 0){
		g_LED_D3_ON = 1; // set D2 on
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET) ;

	}
	else if(HAL_GetTick() - g_time_passed >= 100 && g_LED_D3_ON == 1){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET) ;
		g_time_passed =  HAL_GetTick() ;
		g_LED_D3_ON = 0;  //set D2 off

	}

}

/**
 * Function flashed LED D2 at specified interval of 100 ms
 */
void flash_led_d2(){
	if(HAL_GetTick() - g_time_passed >= 100 && g_LED_D2_ON == 0){
		g_LED_D2_ON = 1; // set D2 on
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET) ;

	}
	else if(HAL_GetTick() - g_time_passed >= 200 && g_LED_D2_ON == 1){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET) ;
		g_time_passed =  HAL_GetTick() ;
		g_LED_D2_ON = 0;  //set D2 off

	}

}

void store_system_state_in_buffer(char analog_temp[], char dig_temp[], char system_state[], uint8_t len_of_sys_arr ){
	  for(int i = 0; i < len_of_sys_arr ; i++){
		  switch(i){
		  case 0:
			  system_state_transmit[0] = '&' ;
			  break ;
		  case 1:
			  system_state_transmit[1] = '_' ;

			  break;
		  case 2:
			  system_state_transmit[2] = analog_temp[0] ;

			  break;
		  case 3:
			  system_state_transmit[3] = analog_temp[1] ;

			  break;
		  case 4:
			  system_state_transmit[4] = analog_temp[2] ;

			  break;
		  case 5:
			  system_state_transmit[5] = '_' ;

			  break;
		  case 6:
			  system_state_transmit[6] = dig_temp[0] ;

			  break;
		  case 7:
			  system_state_transmit[7] = dig_temp[1] ;

			  break;
		  case 8:
			  system_state_transmit[8] = dig_temp[2] ;

			  break;
		  case 9:
			  system_state_transmit[9] = '_' ;

			  break;
		  case 10:
			  system_state_transmit[10] = '0' ;

			  break;
		  case 11:
			  system_state_transmit[11] = '0' ;

			  break;
		  case 12:
			  system_state_transmit[12] = '0' ;

			  break;
		  case 13:
			  system_state_transmit[13] = '_' ;

			  break;
		  case 14:
			  system_state_transmit[14] = '*' ;

			  break;
		  case 15:
			  system_state_transmit[15] = '\n' ;

			  break;
		  default:
			  break;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(150);
  HAL_UART_Transmit_IT(&huart2, (uint8_t*)studentNum, 13) ;

  HAL_UART_Receive_IT(&huart2, (uint8_t*)char_rcvd, 1) ;

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1)  ; // input to trigger interrupt - LMT01 sensor

  g_time_passed = HAL_GetTick() ; //snapshot of time

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  system_state_update() ;

	  //UR3: Evironment measure: measure Ta & measure Tb
	  if(g_EN_measure == 1){

		  // ignore bottom button press and SP command while measuring
		  if(g_bottom_button_pressed ==1 || g_SP_config_command_rcvd ==1){
			  g_bottom_button_pressed = 0 ;
			  g_SP_config_command_rcvd = 0 ;
		  }
		  //ANALOGUE SENSOR CALIBRATION
		  g_temp_in_deg = get_adc_value_and_celsius_temperature() ;
		  store_temp_in_string(g_temp_in_deg, g_temperature, LEN);

		  // DIGITAL SENSOR CALIBRATION
		  g_lmt01_sens_temp =  (uint16_t)g_TO1_temp ;
		  store_temp_in_string(g_lmt01_sens_temp, dig_sens_temp, LEN) ;

		  //re-prime system state update
		  g_transmit_system_state =1; //send the system state again

		  //Flash D3 LED -> put in function
		  flash_led_d3();

	  }
	  else if(g_EN_measure == 2){
		  //set LED D3
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET) ;

		  //store system state to transmit
		  store_system_state_in_buffer(g_temperature, dig_sens_temp, system_state_transmit, 17) ;

		  // Transmit system state via the UART
		  if(g_transmit_system_state  == 1){
			  g_transmit_system_state = 0;
			  HAL_UART_Transmit_IT(&huart2, (uint8_t*)system_state_transmit, 16);

		  }

	  }

	  //UR2: PV Module
	  if(g_SP_measure == 1){
		  // ignore top button press and EN command while measuring
		  if(g_top_button_pressed ==1 || g_EN_config_command_rcvd ==1){
			  g_top_button_pressed = 0 ;
			  g_EN_config_command_rcvd = 0;
		  }
		  //Flash D2 LED
		  flash_led_d2() ;
	  }

	  else if(g_SP_measure == 2){
		  //set LED D2
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET) ;

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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_ODD;
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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD2_Pin PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
