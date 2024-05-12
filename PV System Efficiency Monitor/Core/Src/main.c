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
#include "lcd.h"
#include <math.h>
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

RTC_HandleTypeDef hrtc;

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

//LMTO1 sensor
extern uint32_t pulse_count ;
uint8_t Tsp_temp_ready = 0 ;
uint32_t g_pulse_window_period =  0 ;
uint16_t g_num_pulses = 0;
double g_TO1_temp = 0;
uint16_t g_temp_in_deg_dig_sens = 0;

//SFH235 Photodiode
double g_raw_lux_value = 0; ;
uint16_t g_get_lxd_value= 0;
char g_lxd_value[6] = {} ;

//Solar Panel
double g_v1_pv = 0; //PV voltage mV - ADC input 1
double g_v2_pv = 0; //PV voltage mV - ADC input 2

uint16_t g_i_pv = 0; //PV current mA
uint16_t g_i_sc_pv = 0; //PV current mA
uint16_t g_prev_i_pv = 0; //PV current mA
uint16_t g_limiting_current= 0 ; //to avoid over or underflow as a result of the fluctuating voltages v1, v2 becuase of the small

uint16_t g_p_pv = 0; //PV power mW
uint16_t g_prev_p_pv = 0; //PV power mW

uint16_t g_v_mpp = 0; //PV voltage mpp
uint16_t g_i_mpp = 0; //PV current mpp
uint16_t g_p_mpp = 0; //PV power mpp

uint8_t g_pv_eff = 0 ; //PV panel eff. %

//voltages (mv) from ADC conversion
uint16_t g_PV_vol1 = 0;
uint16_t g_v_oc_pv = 0;
uint16_t g_prev_v_pv = 0;


uint16_t g_PV_vol2 = 0;

char g_lcd_amb_val[9] = {} ;
char g_lcd_sb_val[8] = {} ;
char g_lcd_lux_val[10] = {} ;
uint16_t lcd_scaled_lux = 0;


char g_panel_voltage[9] = {};
char g_panel_mpp_voltage[5] = {};

char g_panel_current[4] ={};
char g_panel_mpp_current[4] ={};

char g_panel_power[4] = {} ;
char g_panel_mpp_power[4] = {} ;

//reduction in LCD real time value update frequecy, to improve response time.
char g_panel_voltage_and_current[17] = {};

char g_envir_measure_temperatures[17] = {} ;  //store temperatue measures

char g_panel_power_and_efficiency[17] = {};
uint8_t g_read_real_time_values = 1 ;
uint32_t g_previous_time_of_lcd = 0;

char g_panel_eff[5] = {} ;

// SYSTEM state machine variables
char g_system_config[17] = {} ;
uint8_t g_byte_count = 0 ;
uint8_t g_config_command_rcvd = 0;    // check for when config recvd

// EN measure
uint8_t g_EN_measure= 0;
uint8_t g_EN_measure_LCD_display= 0;

uint32_t g_time_passed = 0 ;
uint8_t g_LED_D3_ON  =0 ;   // LED D3 state initially off
uint8_t g_EN_config_command_rcvd = 0;

// EN measure
uint8_t g_SP_measure= 0;
uint8_t g_SP_measure_LCD_diplay= 0;

uint8_t g_LED_D2_ON  =0 ;   // LED D2 state initially off
uint8_t g_SP_config_command_rcvd = 0;

// CA (calibration)
uint8_t g_CA_config_command_rcvd = 0;
uint8_t g_LED_D4_ON  =0 ;   // LED D2 state initially off
uint8_t g_CA_measure= 0;


char system_state_transmit[19] = {} ;
char system_state_SP_transmit[22] = {} ;

uint8_t g_transmit_system_state = 1;
uint8_t g_transmit_SP_system_state = 1;

uint8_t g_lcd_mode = 0;
uint8_t g_lcd_default_mode = 1;
uint8_t display_result = 0 ;
uint8_t clear_lcd_display = 1 ;


//LCD variable
Lcd_PortType ports[] = { GPIOB, GPIOA, GPIOA, GPIOC };
Lcd_PinType pins[] = {GPIO_PIN_12, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_6};
Lcd_HandleTypeDef lcd;

//RTC variables

RTC_TimeTypeDef sTime ;
RTC_DateTypeDef sDate ;
uint8_t g_update_RTC  = 0 ;  //indicates if parameter should be updated
uint8_t g_RTC_parameter  =0;
char g_date[11] = {};
char g_time[9] = {};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

uint16_t get_adc_value_and_celsius_temperature() ;
void store_temp_in_string(uint16_t temperature, char temp[], int len) ;
void system_state_update() ;
void flash_led_d3() ;
void flash_led_d2() ;
void flash_led_d4() ;
void store_system_state_in_buffer(char analog_temp[], char dig_temp[],char lux_value[],char system_state[], uint8_t len_of_sys_arr );
void en_measurements_and_responses();
void ADC_Select_CH0(void) ;
void ADC_Select_CH14(void);
void ADC_Select_CH9(void);
void ADC_Select_CH15(void);
uint16_t get_adc_value_conver_to_lux();
uint16_t get_pv_panel_adc1_input();
uint16_t get_pv_panel_adc2_input();
void change_lcd_display_mode();
void set_RTC_date_and_time();
void en_measurement_update() ;
void sp_measurement_update() ;
void sp_measurements_and_responses() ;
void ca_measurements_and_responses();
void g_clock_menu_set_and_parameter_update() ;
void lcd_display_mode_change_on_button_press() ;
void lcd_Mode_1() ;
void lcd_Mode_2() ;
void lcd_Mode_3();
void change_between_dispplay_modes() ;
void ca_measurements_update();
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

	//select adc channel 0
	ADC_Select_CH0() ;
	HAL_ADC_Start(&hadc1) ;
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	g_raw  = HAL_ADC_GetValue(&hadc1) ;
	HAL_ADC_Stop(&hadc1) ;  //stop adc

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
 * The system is not to be updates when performing a RTC update
 */
void system_state_update(){

	//RTC menu enter/exit
	g_clock_menu_set_and_parameter_update();

	if(g_update_RTC == 0){
		//CHECK FOR TYPE OF MEASUREMENT command rcvd via UART
		if( g_config_command_rcvd == 1){
			g_config_command_rcvd = 0;


			if( g_system_config[2]=='E' && g_system_config[3] == 'N'){
				// EN measure comand
				g_EN_config_command_rcvd =1 ;
				//stop SP measure
				g_SP_config_command_rcvd =0 ;
				//stop CA (calibration)
				g_SP_config_command_rcvd = 0;

			}

			else if(g_system_config[2]=='S' && g_system_config[3] == 'P'){
				// SP command
				g_SP_config_command_rcvd =1 ;
				//stop EN command
				g_EN_config_command_rcvd =0;
				//stop CA (calibration)command
				g_CA_config_command_rcvd = 0;

			}

			//extend to CA command recieved
			else if(g_system_config[2]=='C' && g_system_config[3] == 'A'){

				g_CA_config_command_rcvd = 1;
				// SP command
				g_SP_config_command_rcvd =0 ;
				//stop EN command
				g_EN_config_command_rcvd =0;

			}

			else{
				HAL_UART_Transmit_IT(&huart2, (uint8_t*)"Invalid Command\n", 16);
			}
		}

		//perform en measurements
		en_measurement_update() ;
		//perform sp measurement
		sp_measurement_update() ;
		//perform calibration measurements
		ca_measurements_update();
	}
	else{
		//dont update any states while in the RTC menu
	}

}

/**
 * This function begins the calibrations sequence
 * It is to start the measurement for EN and SP where the values for lux is to be stored for later measurements
 *
 */

void ca_measurements_update(){

	// CA - update via push button
	if(g_right_button_pressed  == 1 && g_CA_config_command_rcvd == 0){
		g_right_button_pressed = 0;

		g_CA_measure++  ;

		if(g_CA_measure >2 ){
			g_CA_measure = 1;
		}
	}

	//CA update via UART
	else if(g_right_button_pressed ==0  && g_CA_config_command_rcvd == 1){
		g_CA_config_command_rcvd = 0;

		//check that the correct UART message recvd
		if(g_system_config[0]== '&' && g_system_config[1 ]== '_' && g_system_config[2]=='C' && g_system_config[3] == 'A' && g_system_config[4] =='_'&& g_system_config[5] =='*' &&  g_system_config[6] =='\n' ){
			if(g_CA_measure == 0){
				g_CA_measure = 1;
			}
			else if(g_CA_measure == 1){
				g_CA_measure = 2;

			}
			else{
				if(g_CA_measure ==2){
					g_CA_measure = 1;
				}
			}
		}
		//else block to not update g_EN_measure if incorrent command revcd
		else{
			g_CA_measure =  g_CA_measure ;
		}
	}

}
/**
 * This function start/stops the environment measurements
 * This is done by both the UART and push button
 */
void en_measurement_update(){
	// Environment Measure - And not measuring SP - modularise
	// EN - update via push button
	if(g_top_button_pressed  == 1 && g_EN_config_command_rcvd ==0 && g_SP_config_command_rcvd ==0 && (g_SP_measure == 0 || g_SP_measure ==2)  && (g_CA_measure == 0 || g_CA_measure ==2)){
		g_top_button_pressed = 0;

		g_EN_measure++  ;

		if(g_EN_measure >2 ){
			g_EN_measure = 1;
		}
	}
	//EN update via UART
	else if(g_top_button_pressed ==0  && g_EN_config_command_rcvd == 1 && g_SP_config_command_rcvd ==0  && (g_SP_measure == 0 || g_SP_measure ==2) ){
		g_EN_config_command_rcvd = 0;

		//check that the correct UART message recvd
		if(g_system_config[0]== '&' && g_system_config[1 ]== '_' && g_system_config[2]=='E' && g_system_config[3] == 'N' && g_system_config[4] =='_'&& g_system_config[5] =='*' &&  g_system_config[6] =='\n' ){
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

}

/**
 * This function start/stops the panel measurements
 * This is done by both the UART and push button
 */
void sp_measurement_update(){
	//SP Measure - update via bottom push button - modularise
	if(g_bottom_button_pressed  == 1  && g_EN_config_command_rcvd == 0 && g_SP_config_command_rcvd ==0  && (g_EN_measure == 0 || g_EN_measure ==2)){
		//clear lcd display after EN measurement
		Lcd_clear(&lcd); //to remove EN measure values
		g_bottom_button_pressed = 0;

		g_SP_measure++  ;

		if(g_SP_measure >2 ){
			g_SP_measure = 1;
		}
	}

	// SP update via uart
	else if(g_bottom_button_pressed ==0  && g_EN_config_command_rcvd == 0 && g_SP_config_command_rcvd ==1 && (g_EN_measure == 0 || g_EN_measure ==2)){

		//clear lcd display after EN measurement
		Lcd_clear(&lcd); //to remove EN measure values

		g_SP_config_command_rcvd = 0;

		//check that the correct UART message recvd
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

/**
 * Function flashed LED D4 at specified interval of 200 ms
 */
void flash_led_d4(){
	if(HAL_GetTick() - g_time_passed >= 200 && g_LED_D4_ON == 0){
		g_LED_D4_ON = 1; // set D2 on
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET) ;

	}
	else if(HAL_GetTick() - g_time_passed >= 400 && g_LED_D4_ON == 1){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET) ;
		g_time_passed =  HAL_GetTick() ;
		g_LED_D4_ON = 0;  //set D2 off

	}

}

void store_system_state_in_buffer(char analog_temp[], char dig_temp[],char lux_value[],char system_state[], uint8_t len_of_sys_arr ){
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
			  system_state_transmit[10] = lux_value[0] ;

			  break;
		  case 11:
			  system_state_transmit[11] = lux_value[1] ;

			  break;
		  case 12:
			  system_state_transmit[12] = lux_value[2] ;

			  break;
		  case 13:
			  system_state_transmit[13] = lux_value[3] ;

			  break;
		  case 14:
			  system_state_transmit[14] = lux_value[4] ;

			  break;
		  case 15:
			  system_state_transmit[13] = '_' ;

			  break;
		  case 16:
			  system_state_transmit[14] = '*' ;

			  break;
		  case 17:
			  system_state_transmit[15] = '\n' ;

			  break;
		  default:
			  break;
		  }
	  }
}

//digital sensore
void ADC_Select_CH0(void){
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}


}
// photodiode
void ADC_Select_CH14(void){
	ADC_ChannelConfTypeDef sConfig = {0};

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}


}

//PV- Panel- ADC 1
void ADC_Select_CH9(void){
	ADC_ChannelConfTypeDef sConfig = {0};

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

//PV - Panel - ADC2
void ADC_Select_CH15(void){
	ADC_ChannelConfTypeDef sConfig = {0};

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * Function starts ADC CH14, connected to ouput of light diode
 * get the ADC value of diode
 */
double ip_diode = 0;
uint16_t lux_value = 0;
uint16_t get_adc_value_conver_to_lux(){
	ADC_Select_CH14() ;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) ;
	g_raw_lux_value = HAL_ADC_GetValue(&hadc1) ;
	HAL_ADC_Stop(&hadc1);

	//scale adc value [0,99999] - For 30000 lux
//	g_raw_lux_value = g_raw_lux_value*(29999.0/4095.0) ;

	//make these into your own values
	g_raw_lux_value = g_raw_lux_value*(3.3/4095.0) ;
	ip_diode = g_raw_lux_value/1200.0 ;
	lux_value = pow((ip_diode/(0.085*pow(10,-6))),1/1.01) ;

//	return g_raw_lux_value ;
	return lux_value ;
}
/**
 * This funtion performs the measurement for UR3: Environement measure.
 * It measures the ambient temperature, solar panel temperature and light intensity,
 * the board is exposed to.
 * Measurements and responses, transmitted to UART and LCD
 */
void en_measurements_and_responses(){


	if(g_EN_measure == 1){

	  // ignore bottom and left button press and SP command while measuring
	  if(g_bottom_button_pressed ==1 || g_SP_config_command_rcvd ==1 ||  g_left_button_pressed ==1){
		  g_bottom_button_pressed = 0 ;
		  g_SP_config_command_rcvd = 0 ;
		  g_left_button_pressed = 0;
	  }
	  //ANALOGUE SENSOR CALIBRATION
	  g_temp_in_deg = get_adc_value_and_celsius_temperature() ;
	  store_temp_in_string(g_temp_in_deg, g_temperature, LEN);

	  //PHOTODIOCE ouput
	  g_get_lxd_value = get_adc_value_conver_to_lux();
	  snprintf(g_lxd_value, sizeof(g_lxd_value), "%05d",g_get_lxd_value);

	  // DIGITAL SENSOR CALIBRATION
	  g_lmt01_sens_temp =  (uint16_t)g_TO1_temp ;
	  store_temp_in_string(g_lmt01_sens_temp, dig_sens_temp, LEN) ;

	  //re-prime system state update
	  if(g_transmit_system_state ==0){
		  g_transmit_system_state =1; //send the system state again

	  }

	  //Flash D3 LED -> put in function
	  flash_led_d3();
	}
	else if(g_EN_measure == 2){
		//enter state only once
		g_EN_measure = 0;

		//clear lcd
		 Lcd_clear(&lcd);
		//update LCD mode - to EN mode
		g_EN_measure_LCD_display =  1 ;
		g_SP_measure_LCD_diplay = 0 ; //dont display SP measurements
	  //set LED D3
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET) ;

	  //store system state to transmit
	  snprintf(system_state_transmit,sizeof(system_state_transmit),"&_%03d_%03d_%05d_*\n",g_temp_in_deg,g_lmt01_sens_temp, g_get_lxd_value );

	  // Transmit system state via the UART
	  if(g_transmit_system_state  == 1){

		  g_transmit_system_state = 0;
		  HAL_UART_Transmit_IT(&huart2, (uint8_t*)system_state_transmit, 18);

	  }

	}
}

/**
 * This funtion performs the measurement for UR2: Photocvoltaic measurements.
 * It measures the PV voltage, PV current, PV power and PV efficiency,
 * Measurements and responses, transmitted to UART and LCD
 */
void sp_measurements_and_responses(){
	  if(g_SP_measure == 1){

		// ignore top button and left button press and EN command while measuring
		if(g_top_button_pressed ==1 || g_EN_config_command_rcvd ==1 || g_left_button_pressed ==1){
		  g_top_button_pressed = 0 ;
		  g_EN_config_command_rcvd = 0;
		  g_left_button_pressed = 0;
		}

		//reprime state transmission
		if(g_transmit_SP_system_state == 0){
			g_transmit_SP_system_state = 1;
		}
		//PV panel data points measure
		g_PV_vol1 = get_pv_panel_adc2_input() ; //Voc = Vsp
		//*ADC input(2) Vb = V_var
		g_PV_vol2= get_pv_panel_adc1_input() ;

		//GET VALUES
		//Voc measure - Voc, vpv
		if(g_PV_vol1 > g_prev_v_pv){
		  g_prev_v_pv = g_PV_vol1 ;
		  //capture maximum open circuit voltage
		  g_v_oc_pv = g_PV_vol1 ;
		}

		//curent measure - Isc, Ipv
		if(g_PV_vol1 - g_PV_vol2 >0){
			g_i_pv = g_PV_vol1 - g_PV_vol2 ;
		}
		else{
			g_i_pv = g_i_pv ; //dont update current
		}

		//power measure - multiply by 1000, to get result in mW
		g_p_pv = ( (g_PV_vol1 * g_i_pv)/1000000.0) *1000;


		/*mpp values measure using power -Pmpp, Vmpp, Impp*/
		//check is power increasing
		if(g_p_pv > g_prev_p_pv ){
			g_p_mpp = g_p_pv ;
			g_v_mpp = g_PV_vol1 ;
			g_i_mpp = g_i_pv ;
			g_prev_p_pv = g_p_pv ;
		}


		if(clear_lcd_display == 1){
			clear_lcd_display = 0;
			Lcd_clear(&lcd);
		}
		//LCD write - real-time measured Vpv (mV), Ipv (mA), Ppv (mW), Peff = 0 while measuring
		//few commands - (have 2 write commands, and reduce rate of sampling/real time value display)

		//update valus only every 30ms
		if(HAL_GetTick() - g_previous_time_of_lcd >=30){
			//write to lcd - //row 1
			Lcd_cursor(&lcd, 0, 0) ;
			snprintf(g_panel_voltage_and_current, sizeof(g_panel_voltage_and_current),"V:%04dmV I:%03dmA",g_PV_vol1,g_i_pv);
			Lcd_string(&lcd, g_panel_voltage_and_current);

			//2nd row
			Lcd_cursor(&lcd, 1, 0) ;
			snprintf(g_panel_power_and_efficiency, sizeof(g_panel_power_and_efficiency),"P: %03dmW E:%03d%%",g_p_pv, g_pv_eff);
			Lcd_string(&lcd, g_panel_power_and_efficiency);

			g_previous_time_of_lcd = HAL_GetTick() ;
		}


		//Flash D2 LED
		flash_led_d2() ;
	  }

	  else if(g_SP_measure == 2){
		  //enter this state once
		  g_SP_measure = 0;

		  //clear lcd
//		  Lcd_clear(&lcd);
		  //update LCD mode to SP measurements

		  g_SP_measure_LCD_diplay =  1;
		  g_EN_measure_LCD_display = 0; //dont diplay EN measurements

		  //set LED D2
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET) ;
		  snprintf(system_state_SP_transmit,sizeof(system_state_SP_transmit), "&_%04d_%03d_%03d_%03d_*\n",g_v_mpp,g_i_mpp,g_p_mpp,g_pv_eff);
		  //transmit system SP response
		  if(g_transmit_SP_system_state == 1){

			  g_transmit_SP_system_state = 0 ;
			  //transmit over UART
			  HAL_UART_Transmit_IT(&huart2,(uint8_t*)system_state_SP_transmit, 21) ;

		  }
	  }
}

void ca_measurements_and_responses(){
	if(g_CA_measure == 1){ // find calibrated measurements



		flash_led_d4() ;
	}
	else if(g_CA_measure == 2){
		//calibration ended
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET) ;
	}
}
/**
 * This function retrieves the ADC panel voltage,
 * across adc input 1
 * Gets the input voltage to the adc
 * Scales the voltage to the appropriate PV voltage
 */
uint16_t get_pv_panel_adc1_input(){
	//READ ADC VALUE
	ADC_Select_CH15() ;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) ;
	g_v1_pv = HAL_ADC_GetValue(&hadc1) ;
	HAL_ADC_Stop(&hadc1);

	//get voltage (mv) - from ADC formula
	g_v1_pv = g_v1_pv*(3.3/4095.0) ;

	//scale voltage up to PV panel voltage (V)
	g_v1_pv = g_v1_pv*(99000.0/39000.0);

	//multiply by 1000 to get voltage to mv
	g_v1_pv = g_v1_pv*1000;

	return g_v1_pv ;
}

/**
 * This function returns the adc panel voltage, across adc input
 * 2
 * Gets the input voltage to the adc
 * Scales the voltage to the appropriate PV voltage
 */
uint16_t get_pv_panel_adc2_input(){
	//read adc value
	ADC_Select_CH9() ;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) ;
	g_v2_pv = HAL_ADC_GetValue(&hadc1) ;
	HAL_ADC_Stop(&hadc1);

	//get voltage (mv) -from adc formula
	g_v2_pv = g_v2_pv*(3.3/4095.0) ;

	//scale voltage up to PV panel voltage
	g_v2_pv = g_v2_pv*(99000.0/39000.0);

	//multiply by 1000 to get voltage to mv
	g_v2_pv = g_v2_pv*1000;

	return g_v2_pv ;

}

/**
 * SP Measurement
 */
void lcd_Mode_1(){
	Lcd_clear(&lcd);

	//first row
	Lcd_cursor(&lcd, 0, 0) ;
	snprintf(g_panel_voltage_and_current, sizeof(g_panel_voltage_and_current),"V:%04dmV I:%03dmA",g_v_mpp,g_i_mpp);
	Lcd_string(&lcd, g_panel_voltage_and_current);

	//2nd row
	Lcd_cursor(&lcd, 1, 0) ;
	snprintf(g_panel_power_and_efficiency, sizeof(g_panel_power_and_efficiency),"P: %03dmW E:%03d%%",g_p_mpp, g_pv_eff);
	Lcd_string(&lcd, g_panel_power_and_efficiency);

}

/**
 * EN measurements
 */
void lcd_Mode_2(){
	//clear current LCD contents
	Lcd_clear(&lcd);
	//first row
	Lcd_cursor(&lcd, 0, 0) ;
	snprintf(g_envir_measure_temperatures, sizeof(g_envir_measure_temperatures),"AMB:%03dC SP:%03dC",g_temp_in_deg,g_lmt01_sens_temp);
	Lcd_string(&lcd, g_envir_measure_temperatures);

	//scale lux value: [0: 30000]?
	Lcd_cursor(&lcd, 1,0);
	snprintf(g_lcd_lux_val, sizeof(g_lcd_lux_val),"LUX:%05d",g_get_lxd_value);
	Lcd_string(&lcd,g_lcd_lux_val);
}

/**
 * RTC measurements
 */
void lcd_Mode_3(){
	Lcd_clear(&lcd);

	//display the updated time and date
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) ;
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) ;

	snprintf(g_date, sizeof(g_date),"%02d/%02d/20%02d",sDate.Date, sDate.Month,sDate.Year);
	snprintf(g_time, sizeof(g_time),"%02d:%02d:%02d",sTime.Hours, sTime.Minutes, sTime.Seconds);
	//display date
	Lcd_cursor(&lcd, 0, 0) ;
	Lcd_string(&lcd, g_date);

	//display time - second row
	Lcd_cursor(&lcd, 1, 0) ;
	Lcd_string(&lcd, g_time);
}
/**
 * This function displays the values of the different measurement,
 * by altering the display mode on the LCD.
 * Changing the view is independent of the system ,and does not change any
 * state of the system
 */

uint8_t default_switch_mode = 1;
void change_lcd_display_mode(){
	//display default display mode
	if(g_lcd_default_mode == 1){

		g_lcd_default_mode = 0;

		//update the correspoding lcd mode
		g_lcd_mode =1 ;

		Lcd_clear(&lcd);

		//first row
		Lcd_cursor(&lcd, 0, 0) ;
		snprintf(g_panel_voltage_and_current, sizeof(g_panel_voltage_and_current),"V:%04dmV I:%03dmA",g_v_mpp,g_i_mpp);
		Lcd_string(&lcd, g_panel_voltage_and_current);

		//2nd row
		Lcd_cursor(&lcd, 1, 0) ;
		snprintf(g_panel_power_and_efficiency, sizeof(g_panel_power_and_efficiency),"P: %03dmW E:%03d%%",g_p_mpp, g_pv_eff);
		Lcd_string(&lcd, g_panel_power_and_efficiency);

		//get default date and time
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) ;

		snprintf(g_date, sizeof(g_date),"%02d/%02d/20%02d",sDate.Date, sDate.Month,sDate.Year);
		snprintf(g_time, sizeof(g_time),"%02d:%02d:%02d",sTime.Hours, sTime.Minutes, sTime.Seconds);

	}


	//dont change lcd modes when updating the RTC
	if(g_update_RTC ==0){
		lcd_display_mode_change_on_button_press() ;

	}
	else{
		if(g_left_button_pressed == 1){
			g_left_button_pressed = 0 ; //if left button pressed when updating the RTC parameteers
		}
	}


	if(display_result == 1){
		display_result = 0;


		if(g_lcd_mode == 2){//display mode 2: EN measurement
			display_result = 0; //display contents once only

			lcd_Mode_2() ;

		}
		else if(g_lcd_mode == 1 ){//disply mode 1: SP measurements

			display_result = 0 ; //display content only once

			lcd_Mode_1() ;

		}

		else{
			if(g_lcd_mode == 3){ //display mode 3: RTC
				display_result = 0 ; //display content only once

				lcd_Mode_3();

			}
		}

	}

	//check for LCD MODE 4 -> switch between the different display modes at interval of 2s
	else if(display_result ==0 && g_lcd_mode == 4 ){
		//change to default mode 1 in the beginning
//		if(default_switch_mode == 1){
////			Lcd_clear(&lcd);
////			lcd_Mode_1() ;
//
//			default_switch_mode = 0;
//		}

		change_between_dispplay_modes();
	}

}

uint32_t time_passed_between_mode = 0 ;
uint8_t display_mode_x = 2; //start at mode 2

void change_between_dispplay_modes(){

	if(HAL_GetTick()- time_passed_between_mode >= 2000 && display_mode_x == 1){
		display_mode_x = 2 ;
		lcd_Mode_1() ;

	}

	else if(HAL_GetTick()- time_passed_between_mode >=4000 && display_mode_x == 2){
		display_mode_x = 3 ;
		lcd_Mode_2();

	}

	if(HAL_GetTick()- time_passed_between_mode >=6000 && display_mode_x == 3){
		display_mode_x= 1;
		time_passed_between_mode = HAL_GetTick() ;
		lcd_Mode_3() ;

	}

}
/**
 * This function works to change the LCD display modes when the left button is pressed
 */
void lcd_display_mode_change_on_button_press(){

	//update state based on button press - DONT UPDATE ANYS STATES WHILE MEASURING
	if(g_left_button_pressed == 1 && g_EN_measure_LCD_display ==0 && g_SP_measure_LCD_diplay ==0 && g_SP_measure !=1 && g_EN_measure !=1){

		//display/update lcd results
		display_result= 1 ;

		g_left_button_pressed = 0;
		g_lcd_mode ++;

		//cycles between the mode
		if(g_lcd_mode>4){
			g_lcd_mode = 1 ;
		}
	}

	//update lcd state based on  EN stop command
	else if(g_left_button_pressed == 0 && g_EN_measure_LCD_display == 1 && g_SP_measure_LCD_diplay ==0  && g_SP_measure !=1 && g_EN_measure !=1 ){
		g_EN_measure_LCD_display = 0 ;

		g_lcd_mode = 2; //display EN measurements

		//display/update lcd results
		display_result= 1 ;
	}

	//update LCD based on SP command
	else if(g_left_button_pressed ==0 && g_EN_measure_LCD_display== 0 && g_SP_measure_LCD_diplay== 1  && g_SP_measure !=1 && g_EN_measure !=1){
		g_SP_measure_LCD_diplay =0;

		g_lcd_mode = 1; //display SP measurements

		//display/update lcd results
		display_result= 1 ;
	}

	//otherwise dont update display maode
	else{
		g_lcd_mode = g_lcd_mode ;
	}

}

/**
 * This function sets the date and time on the real time clock.
 * On every middle button press input , the parameter updates get switched
 *In this mode the top and bottom buttons get used to increment or decrement the parameters
 *The system is designed such that until all the parameters are set ,no other state is execessible.
 *Once the Menu state is entered. It can only be exited on when the left button is pressed. This restores normal system state functionality
 *The top and bottom measurements can be used to update /start or stop the measurement sequence
 */

void RTC_date_and_time_update(uint8_t paramx){

	//IF UPDATING RTC VALUES - do not use top button and bottom button for measurement sequence

	if(paramx == 1){ //update date

		//top button pressed - increment date
		if(g_top_button_pressed== 1){
			sDate.Date++ ;

			//oveflow
			if(sDate.Date > 30){
				sDate.Date = 1 ;
			}

			g_top_button_pressed =0 ;

			//display date - first row
			snprintf(g_date, sizeof(g_date),"%02d/%02d/20%02d",sDate.Date, sDate.Month,sDate.Year);
			Lcd_cursor(&lcd, 0, 0) ;
			Lcd_string(&lcd, g_date);

			Lcd_cursor(&lcd, 1, 0	) ;
			Lcd_string(&lcd, g_time);


		}

		//bottom button press -decrement date
		if(g_bottom_button_pressed== 1){
			sDate.Date-- ;

			//oveflow
			if(sDate.Date < 1){
				sDate.Date = 31 ;
			}

			//display date - first row
			snprintf(g_date, sizeof(g_date),"%02d/%02d/20%02d",sDate.Date, sDate.Month,sDate.Year);
			Lcd_cursor(&lcd, 0, 0) ;
			Lcd_string(&lcd, g_date);

			Lcd_cursor(&lcd, 1, 0) ;
			Lcd_string(&lcd, g_time);


			g_bottom_button_pressed =0 ;
		}

	}
	else if(paramx == 2){ //update month

		//top button pressed - increment month
		if(g_top_button_pressed== 1){
			sDate.Month++ ;

			//oveflow
			if(sDate.Month > 12){
				sDate.Month = 1 ;
			}
			//display date - first row
			snprintf(g_date, sizeof(g_date),"%02d/%02d/20%02d",sDate.Date, sDate.Month,sDate.Year);
			Lcd_cursor(&lcd, 0, 0) ;
			Lcd_string(&lcd, g_date);

			Lcd_cursor(&lcd, 1, 0) ;
			Lcd_string(&lcd, g_time);

			g_top_button_pressed =0 ;
		}

		//bottom button press -decrement month

		if(g_bottom_button_pressed== 1){
			sDate.Month-- ;

			//oveflow
			if(sDate.Month < 1){
				sDate.Month = 12 ;
			}

			//display date - first row
			snprintf(g_date, sizeof(g_date),"%02d/%02d/20%02d",sDate.Date, sDate.Month,sDate.Year);
			Lcd_cursor(&lcd, 0, 0) ;
			Lcd_string(&lcd, g_date);

			Lcd_cursor(&lcd, 1, 0) ;
			Lcd_string(&lcd, g_time);

			g_bottom_button_pressed =0 ;
		}

	}
	else if(paramx == 3){ //update year -
		//top button pressed - increment Year
		if(g_top_button_pressed== 1){
			sDate.Year++ ;

			//oveflow
			if(sDate.Year > 99){
				sDate.Year = 0 ;
			}

			//display date - first row
			snprintf(g_date, sizeof(g_date),"%02d/%02d/20%02d",sDate.Date, sDate.Month,sDate.Year);
			Lcd_cursor(&lcd, 0, 0) ;
			Lcd_string(&lcd, g_date);

			Lcd_cursor(&lcd, 1, 0) ;
			Lcd_string(&lcd, g_time);

			g_top_button_pressed =0 ;
		}

		//bottom button press -decrement year

		if(g_bottom_button_pressed== 1){
			sDate.Year-- ;

			//oveflow
			if(sDate.Year < 1){
				sDate.Year = 99 ;
			}

			//display date - first row
			snprintf(g_date, sizeof(g_date),"%02d/%02d/20%02d",sDate.Date, sDate.Month,sDate.Year);
			Lcd_cursor(&lcd, 0, 0) ;
			Lcd_string(&lcd, g_date);

			Lcd_cursor(&lcd, 1, 0) ;
			Lcd_string(&lcd, g_time);

			g_bottom_button_pressed =0 ;
		}
	}

	else if(paramx == 4){//update hour
		//top button pressed - increment Year
		if(g_top_button_pressed== 1){
			sTime.Hours++ ;

			//oveflow
			if(sTime.Hours > 24){
				sTime.Hours = 1 ;
			}

			g_top_button_pressed =0 ;

			Lcd_cursor(&lcd, 0, 0) ;
			Lcd_string(&lcd, g_date);

			//display time - second row
			snprintf(g_time, sizeof(g_time),"%02d:%02d:%02d",sTime.Hours, sTime.Minutes, sTime.Seconds);

			Lcd_cursor(&lcd, 1, 0) ;
			Lcd_string(&lcd, g_time);


		}

		//bottom button press -decrement month

		if(g_bottom_button_pressed== 1){
			sTime.Hours-- ;

			//oveflow
			if(sTime.Hours < 1){
				sTime.Hours = 24 ;
			}

			Lcd_cursor(&lcd, 0, 0) ;
			Lcd_string(&lcd, g_date);
			//display time - second row
			snprintf(g_time, sizeof(g_time),"%02d:%02d:%02d",sTime.Hours, sTime.Minutes, sTime.Seconds);

			Lcd_cursor(&lcd, 1, 0) ;
			Lcd_string(&lcd, g_time);

			g_bottom_button_pressed =0 ;
		}
	}

	else if(paramx ==5){//update minutes
		//top button pressed - increment minutes
		if(g_top_button_pressed== 1){
			sTime.Minutes++ ;

			//oveflow
			if(sTime.Minutes > 60){
				sTime.Minutes = 1 ;
			}

			Lcd_cursor(&lcd, 0, 0) ;
			Lcd_string(&lcd, g_date);

			//display time - second row
			snprintf(g_time, sizeof(g_time),"%02d:%02d:%02d",sTime.Hours, sTime.Minutes, sTime.Seconds);

			Lcd_cursor(&lcd, 1, 0) ;
			Lcd_string(&lcd, g_time);

			g_top_button_pressed =0 ;
		}

		//bottom button press -decrement month

		if(g_bottom_button_pressed== 1){
			sTime.Minutes-- ;

			//oveflow
			if(sTime.Minutes < 1){
				sTime.Minutes = 60 ;
			}

			Lcd_cursor(&lcd, 0, 0) ;
			Lcd_string(&lcd, g_date);

			//display time - second row
			snprintf(g_time, sizeof(g_time),"%02d:%02d:%02d",sTime.Hours, sTime.Minutes, sTime.Seconds);

			Lcd_cursor(&lcd, 1, 0) ;
			Lcd_string(&lcd, g_time);

			g_bottom_button_pressed =0 ;
		}
	}

	else if(paramx == 6){//update seconds
		//top button pressed - increment Year
		if(g_top_button_pressed== 1){
			sTime.Seconds++ ;

			//oveflow
			if(sTime.Seconds > 60){
				sTime.Seconds = 1 ;
			}

			Lcd_cursor(&lcd, 0, 0) ;
			Lcd_string(&lcd, g_date);

			//display time - second row
			snprintf(g_time, sizeof(g_time),"%02d:%02d:%02d",sTime.Hours, sTime.Minutes, sTime.Seconds);

			Lcd_cursor(&lcd, 1, 0) ;
			Lcd_string(&lcd, g_time);

			g_top_button_pressed =0 ;
		}

		//bottom button press -decrement month

		if(g_bottom_button_pressed== 1){
			sTime.Seconds-- ;

			//oveflow
			if(sTime.Seconds < 1){
				sTime.Seconds = 60 ;
			}
			Lcd_cursor(&lcd, 0, 0) ;
			Lcd_string(&lcd, g_date);

			//display time - second row
			snprintf(g_time, sizeof(g_time),"%02d:%02d:%02d",sTime.Hours, sTime.Minutes, sTime.Seconds);

			Lcd_cursor(&lcd, 1, 0) ;
			Lcd_string(&lcd, g_time);

			g_bottom_button_pressed =0 ;
		}
	}

	else{ //exit the RTC menu

		if(paramx == 7){
			g_update_RTC = 0 ; //done updating the RTC
			//update date and time
			set_RTC_date_and_time() ;

			snprintf(g_date, sizeof(g_date),"%02d/%02d/20%02d",sDate.Date, sDate.Month,sDate.Year);
			snprintf(g_time, sizeof(g_time),"%02d:%02d:%02d",sTime.Hours, sTime.Minutes, sTime.Seconds);

			Lcd_cursor(&lcd, 0, 0) ;
			Lcd_string(&lcd, g_date);

			Lcd_cursor(&lcd, 1, 0) ;
			Lcd_string(&lcd, g_time);
		}

	}


}

void g_clock_menu_set_and_parameter_update(){
	if(g_middle_button_pressed == 1){
		g_middle_button_pressed = 0;

		//RTC menu
		if(g_update_RTC ==0){
			g_update_RTC = 1;

			//set LCD in mode 3 - LCD parameter
			display_result = 1;

			g_lcd_mode = 3  ;
		}

		//increment parameter to update
		g_RTC_parameter++ ;

		if(g_RTC_parameter>7){
			g_RTC_parameter = 1; //cycle back to first parameter once RTC entered again
		}

	}

	//update parameters
	if(g_update_RTC == 1){
		RTC_date_and_time_update(g_RTC_parameter) ;
	}

}

/**
 * This function sets the updated time
 */
void set_RTC_date_and_time(){
	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) ;
	HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) ;
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
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(150);
  HAL_UART_Transmit_IT(&huart2, (uint8_t*)studentNum, 13) ;

  HAL_UART_Receive_IT(&huart2, (uint8_t*)char_rcvd, 1) ;

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1)  ; // input to trigger interrupt - LMT01 sensor

  g_time_passed = HAL_GetTick() ; //snapshot of time

  //Write to LCD
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET) ;

  lcd = Lcd_create(ports, pins, GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_2, LCD_4_BIT_MODE);
  Lcd_clear(&lcd);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //update system state based on input recvd
	  system_state_update() ;

	  //UR3: Evironment measure: measure Ta & measure Tb  (Put in Function)/Modularize
	  en_measurements_and_responses() ;

	  //UR2: PV Module -(Put in Function)/Modularize
	  sp_measurements_and_responses();

	  //UR5: Calibration of device
	  ca_measurements_and_responses();

	  //update LCD - code runs seqeuntionally and lcd updates based on variable states above
	  change_lcd_display_mode();


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x16;
  sTime.Minutes = 0x20;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
  sDate.Month = RTC_MONTH_SEPTEMBER;
  sDate.Date = 0x19;
  sDate.Year = 0x24;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_D5_Pin|DB5_Line_Pin|DB6_Line_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, E_Line_Pin|LED_D4_Pin|DB4_Line_Pin|RS_Line_Pin
                          |R_W_Line_Pin|LED_D2_Pin|LED_D3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DB7_Line_GPIO_Port, DB7_Line_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD2_Pin LED_D5_Pin DB5_Line_Pin DB6_Line_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LED_D5_Pin|DB5_Line_Pin|DB6_Line_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : E_Line_Pin LED_D4_Pin DB4_Line_Pin RS_Line_Pin
                           R_W_Line_Pin LED_D2_Pin LED_D3_Pin */
  GPIO_InitStruct.Pin = E_Line_Pin|LED_D4_Pin|DB4_Line_Pin|RS_Line_Pin
                          |R_W_Line_Pin|LED_D2_Pin|LED_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DB7_Line_Pin */
  GPIO_InitStruct.Pin = DB7_Line_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DB7_Line_GPIO_Port, &GPIO_InitStruct);

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
