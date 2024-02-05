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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

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
uint16_t strobe_delay =  512 ; // UNIT = ms f = 0.9765HZ (default on time)
uint32_t strobe_ticks = 0 ; // strobe delay count
uint8_t led_strobe_on = 0 ; // flag to alternate between on/off in strobe
uint32_t timePassed = 0 ;
uint16_t strobe_led_Intensity = 256 ;
//SOS morse
char letter[]={'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S'
					,'T','U','V','W','X','Y','Z', '1','2','3','4','5','6','7','8','9','0'} ;

char *morse_symbol[]={". -","- . . .","- . - .","- . ." ,"." ,". . - .","- - .",". . . .",". .",". - - -",
				"- . -",". - . ." ,"- -" ,"- ." ,"- - -" ,". - - .","- - . -",". - .",". . .","-",
				". . -",". . . -",". - -","- . . -","- . - -","- - . .", ". - - - -",". . - - -",
				". . . - -",". . . . -",". . . . .","- . . . .","- - . . .","- - - . .",
				"- - - - .","- - - - -"};

uint16_t time_unit = 512 ; //ms
uint32_t morse_current_time = 0 ;
uint8_t space_in_letter= 0;
uint8_t space_between_letters = 1 ;
uint8_t space_between_words = 1 ;
uint8_t DOT = 1 ;
uint8_t DASH = 1 ;
uint8_t next_char_check = 0 ; //  check if the next character is a dot or dash
uint8_t next_char_checked = 0 ;
char SOS[] = ". . . - - - . . ." ; // SOS Morse code ;

char character = '\0' ;

// Mood mode
uint8_t MM_mode_default = 1;
uint16_t R_channel_Intensity= 128;
uint16_t G_channel_Intensity = 128;
uint16_t B_channel_Intensity = 128 ;


uint8_t studentNum[13]="#:23765518:$\n" ;
char recvd_char[1];

char set_or_ret_sys_state[19] = {' '} ;
uint8_t num_characters = 0 ;
uint8_t UART_set_syst_state = 0;  // FLAG - to update system state
uint8_t UART_ret_sys_state = 0;   // FLAG - to requets/return current system state
uint8_t UART_state_update = 0;
uint8_t custom_morse_msg_rcvd = 0 ;  // Indicates whether a custom morse msg been received
uint16_t state  = 0 ;
uint16_t param1 = 0 ;
uint16_t param2 = 0 ;
uint8_t READ_SYS = 0;
char STATE[3] = {' '} ;
char PARAM1[3] = {' '} ;
char PARAM2[3] ={' '};
char Custom_Morse_Msg[3] = {' '};

// Variables to remember previous state of system
// Flashlight mode state values
uint16_t MF_state = 0 ;
uint16_t MF_param1 = 0;
char MF_param2[3] = {' '} ;
// Emergency mode state values
uint16_t ME_state = 0 ;
uint16_t ME_param1 = 0 ;
char ME_param2[3] = "000" ;
// Mood mode state values
uint16_t MM_state = 0;
uint16_t MM_param1 = 0 ;
uint16_t MM_param2 = 0 ;

// return system state
char ret_state[3] = {' '} ;
char ret_param1[3] = {' ' };
char ret_param2[3] = {' '} ;

char value[19] ;  // stores system requested system state



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void MF_mode_LED() ;
void ME_mode_LED();
void MM_mode_LED();

void adc_dma_val_processing() ;
void system_state_update() ;
void right_button_state_update() ;
void TURN_LED_ON_OFF() ;
void EM_mode_Strobe(uint16_t strobe_delay) ;
void Emergency_Mode_State_Update();
void convert_UART_state_params_to_Int();
void Mood_Mode_State_Update() ;
void Request_return_system_state() ;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	// store recieved characters one at at time
	set_or_ret_sys_state[num_characters] = recvd_char[0] ;

	num_characters++ ;

	if(recvd_char[0] == '\n'){
		if(num_characters == 19){
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)"set mode\n",9 ) ;
			UART_set_syst_state = 1 ;
		}else if( num_characters == 7){
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)"request mode\n", 13) ;
			UART_ret_sys_state = 1 ;
		}else{
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)"Incorrect status request size\n", 30) ;
		}

		num_characters =  0;

	}
	// recieve character - re-prime receiver to receive single characters at a time
	HAL_UART_Receive_IT(&huart2, (uint8_t*)recvd_char, 1);
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

/**
 * Depending on input recvd system changes to specified state
 */
void system_state_update(){
	// button system state update
	 if(left_button_pressed ==1 && UART_set_syst_state == 0 && UART_ret_sys_state == 0 ){

		 button_count++ ;
		 if(button_count > 2){
			 button_count = 0 ;
		 }
		 update_led_via_ADC =  0 ; // don't read ADC by default in next state
		 // snapshot of ADC taken in next state
		 if(adc_conv_complete == 1){
			 adc_val_capture =1  ; // capture ADC value
		 }

		 // Emergency MODE DEFAULT
		 if(button_count != 1){
			 strobe_led_Intensity = 256 ;
			 strobe_delay = 512;
		 }

		 left_button_pressed = 0 ;

	 }
	 // UART system state update
	 else if( left_button_pressed ==0 && UART_set_syst_state == 1 && UART_ret_sys_state == 0){ // System state update to come from only one source
		 UART_set_syst_state = 0;
		// stop reading ADC when UART set command  received
		if(adc_conv_complete == 1){
			adc_val_capture = 1 ; // capture slider value
			update_led_via_ADC = 0 ; // dont read until slider moved
		}

		 UART_state_update =1;

		 if(set_or_ret_sys_state[3] == 'F'){
			 button_count =0 ;
		 }
		 else if(set_or_ret_sys_state[3] =='E'){
			 button_count =1;

		 }else{
			 if(set_or_ret_sys_state[3] == 'M'){
				 button_count =2 ;

			 }
		 }
	 }
	 // read system state
	 else{ //dont update the system in any way - read current and previous states
		 if( left_button_pressed ==0 && UART_set_syst_state == 0 && UART_ret_sys_state == 1){

			 // stop reading adc
			if(adc_conv_complete == 1){
				adc_val_capture = 1 ; // capture slider value
				update_led_via_ADC = 0 ; // dont read until slider moved
			}

			 UART_ret_sys_state = 0;
			 READ_SYS =1;

		 }
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

/**
 * Middle button press turns LED ON/OFF
 */
void TURN_LED_ON_OFF(){
	if(middle_button_pressed == 1 && UART_set_syst_state == 0){
		 LED_ON = !LED_ON ;  // turns the LED on OR off

		 // Middle button press -> LED ON / OFF
		 if(LED_ON == 1){
			 htim2.Instance->CCR1 = 1 ; // LED ON
		 }else if(LED_ON ==0){
			 htim2.Instance->CCR1 = 0 ; //LED OFFS
		 }

		 middle_button_pressed = 0 ;
	 }
	else if(middle_button_pressed == 0 && UART_state_update == 1  ){
		if( state>0){
			LED_ON  = 1 ;
		}else{
			LED_ON = 0 ;
		}
	}
}

/**
 * LED strobed with provided number of ms
 */
void EM_mode_Strobe(uint16_t strobe_delay){

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

		 if(update_led_via_ADC ==1){ // update LED intensity if the slider moved
			 htim2.Instance->CCR1 = LED_intensity ;
		 }else{ // if no slider movement strobe with default intensity
			 htim2.Instance->CCR1 = strobe_led_Intensity;
		 }
	 }

}
void convert_UART_state_params_to_Int(){
	if(UART_set_syst_state) {
		for(int i = 0; i < 19 ; i++){
			switch(i){
			case 5:
				STATE[0]= set_or_ret_sys_state[i] ;
				break;
			case 6:
				STATE[1]= set_or_ret_sys_state[i] ;
				break;
			case 7:
				STATE[2] = set_or_ret_sys_state[i] ;
				break;

			case 9:
				PARAM1[0] = set_or_ret_sys_state[i];
				break;
			case 10:
				PARAM1[1] = set_or_ret_sys_state[i] ;
				break;
			case 11:
				PARAM1[2] = set_or_ret_sys_state[i] ;
				break ;
			case 13:
				PARAM2[0] = set_or_ret_sys_state[i] ;
				break;
			case 14:
				PARAM2[1] = set_or_ret_sys_state[i] ;
				break;
			case 15:
				PARAM2[2] = set_or_ret_sys_state[i ];
				break;

			default:
				break ;
			}

		}

		state = atoi(STATE) ;
		param1 = atoi(PARAM1);
		// problem - perform check for if non integer characters are passed!
		if(strcmp(PARAM2, "000") == 0 ){

			param2 = atoi(PARAM2) ;  // SOS MORSE OUTPUT

			ME_param2[0] = param2/100 + 48;
			ME_param2[1] = (param2  -(param2/100)*100)/10 +48 ;
			ME_param2[2] = (param2 - (param2/10)*10) + 48 ;
		}
		else if( strcmp(PARAM2, "000") != 0 ){

			Custom_Morse_Msg[0] = PARAM2[0] ; // CUSTOM MORSE output - declare variable to store the output
			Custom_Morse_Msg[1] = PARAM2[1] ;
			Custom_Morse_Msg[2] = PARAM2[2] ;


			ME_param2[0] = Custom_Morse_Msg[0];
			ME_param2[1] = Custom_Morse_Msg[1] ;
			ME_param2[2] = Custom_Morse_Msg[2] ;

			custom_morse_msg_rcvd = 1;
		}


	}
}

/**
 * Function updates the necessary states/ values when UART command reached to
 * update the system state
 */
void Emergency_Mode_State_Update(){

	if(UART_state_update == 1 && state > 0 && set_or_ret_sys_state[3] =='E'){
		// Dont read ADC
		if(adc_conv_complete == 1){
			adc_val_capture = 1 ; // capture slider value
			update_led_via_ADC = 0 ; // dont read until slider moved
		}

		//update LED intensity
		strobe_led_Intensity = state;

		// strobe
		if(param1 > 0 ){
			em_count = 0 ;
			strobe_delay = param1 ;  // update the ON/off time of strobe

		}
		// SOS output
		else if(param1 == 0 && param2 == 0 && custom_morse_msg_rcvd == 0 ){
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)"SOS MORSE\n", 10) ;
			em_count =1 ; // SOS mode
		}
		// CUSTOM morse msg received
		else{
			if(param1 == 0 && custom_morse_msg_rcvd ==1 ){
				custom_morse_msg_rcvd = 0;
				HAL_UART_Transmit_IT(&huart2, (uint8_t*)"CUSTOM MORSE\n", 13) ;
				em_count = 2 ; // CUSTOM MORSE

			}
		}


		UART_state_update = 0;
	}
	 // copy previous states information
		 ME_state = strobe_led_Intensity ;
		 ME_param1 =  strobe_delay;
//		 if(strcmp(PARAM2, "000") == 0){
//	//		ME_param2 = param2 ;
////			sprintf(ME_param2, "%d", param2) ;
//			ME_param2[0] = param2/100 + 48;
//			ME_param2[1] = (param2  -(param2/100)*100)/10 +48 ;
//			ME_param2[2] = (param2 - (param2/10)*10) + 48 ;
//
//
//		}else{
//			ME_param2[0] = Custom_Morse_Msg[0];
//			ME_param2[1] = Custom_Morse_Msg[1] ;
//			ME_param2[2] = Custom_Morse_Msg[2] ;
//		}


}

void Mood_Mode_State_Update(){
	if(UART_state_update ==1  && set_or_ret_sys_state[3] == 'M'){

		// set R channel intensity
		R_channel_Intensity = state ;
		// set G channel intensity
		G_channel_Intensity = param1 ;
		// set B channel intensity
		B_channel_Intensity = param2 ;

		UART_state_update = 0;
	}
}

void Request_return_system_state(){
	if(READ_SYS ==1 ){
		// flash light mode
		if(set_or_ret_sys_state[3] == 'F'){
			/* here manual copy*/
			ret_state[0] = MF_state/100 + 48 ; // hundred
			ret_state[1] = (MF_state -(MF_state/100)*100)/10 + 48 ; //tens
			ret_state[2] = (MF_state - (MF_state/10)*10) + 48 ;  //units

			ret_param1[0] = MF_param1/100 + 48 ; // hundred
			ret_param1[1] = (MF_param1 -(MF_param1/100)*100)/10 + 48 ; //tens
			ret_param1[2] = (MF_param1 - (MF_param1/10)*10) + 48 ;  //units

			if(custom_morse_msg_rcvd == 1){
				ret_param2[0] = MF_param2[0]; // hundred
				ret_param2[1] = MF_param2[1] ; //tens
				ret_param2[2] = MF_param2[2] ; //units
			}else{
				ret_param2[0] = param2/100 + 48;  // hundred
				ret_param2[1] =	(param2 -(param2/100)*100)/10 + 48 ; //tens
				ret_param2[2] = (param2 - (param2/10)*10) + 48 ; //units
			}


		}
		// emergency mode
		else if(set_or_ret_sys_state[3] == 'E'){
			/* here manual copy*/
			ret_state[0] = ME_state/100 + 48 ; // hundred
			ret_state[1] = (ME_state -(ME_state/100)*100)/10 + 48 ; //tens
			ret_state[2] = (ME_state - (ME_state/10)*10) + 48 ;  //units

			ret_param1[0] = ME_param1/100 + 48 ; // hundred
			ret_param1[1] = (ME_param1 -(ME_param1/100)*100)/10 + 48 ; //tens
			ret_param1[2] = (ME_param1 - (ME_param1/10)*10) + 48 ;  //units

			// check whether param2 was 0 OR CUSTOM morse message recvd
			if(strcmp(ME_param2, "000") == 0){

//				strcpy(ret_param2, ME_param2) ;
				ret_param2[0] = ME_param2[0];
				ret_param2[1] = ME_param2[1] ;
				ret_param2[2] = ME_param2[2] ;

			}else{
				ret_param2[0] = Custom_Morse_Msg[0];
				ret_param2[1] = Custom_Morse_Msg[1] ;
				ret_param2[2] = Custom_Morse_Msg[2] ;

			}

		}
		// mood mode
		else{
			if(set_or_ret_sys_state[3] == 'M'){
				/* here manual copy*/
				ret_state[0] = MM_state/100 + 48 ; // hundred
				ret_state[1] = (MM_state -(MM_state/100)*100)/10 + 48 ; //tens
				ret_state[2] = (MM_state - (MM_state/10)*10) + 48 ;  //units

				ret_param1[0] = MM_param1/100 + 48 ; // hundred
				ret_param1[1] = (MM_param1 -(MM_param1/100)*100)/10 + 48 ; //tens
				ret_param1[2] = (MM_param1 - (MM_param1/10)*10) + 48 ;  //units

				ret_param2[0] = MM_param2/100 + 48 ; // hundred
				ret_param2[1] = (MM_param2 -(MM_param2/100)*100)/10 + 48 ; //tens
				ret_param2[2] = (MM_param2 - (MM_param2/10)*10) + 48 ;  //units

			}
		}

		// construct message to return
		for(int i = 0; i<19  ; i++){
			switch(i){
			case 0:
				value[i] ='#';

				break ;
			case 1:
				value[i] = ':' ;

				break ;
			case 2:
				value[i] = 'M';

			break ;

			case 3:
				value[i] = set_or_ret_sys_state[3];

				break ;
			case 4:
				value[i] = ':';

				break ;
			case 5:
				value[i] = ret_state[0];

			break ;

			case 6:
				value[i] = ret_state[1];

				break ;
			case 7:
				value[i] = ret_state[2];

				break ;
			case 8:
				value[i] =  ':';

			break ;

			case 9:
				value[i] = ret_param1[0];

				break ;
			case 10:
				value[i] = ret_param1[1];

				break ;
			case 11:
				value[i] = ret_param1[2];

			break ;

			case 12:
				value[i] = ':';

				break ;
			case 13:
				value[i] = ret_param2[0];

				break ;
			case 14:
				value[i] = ret_param2[1];

				break ;

			case 15:
				value[i] = ret_param2[2];

				break ;
			case 16:
				value[i] = ':';

					break ;
			case 17:
				value[i] = '$';

				break ;
			case 18:
				value[i] = '\n' ;
				break;

			default:
				break;

			}
		}

		HAL_UART_Transmit_IT(&huart2, (uint8_t*)value, 19) ;



		READ_SYS = 0 ;
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(200);
  HAL_UART_Transmit(&huart2, studentNum, 13, 150); //transmit student number

  HAL_UART_Receive_IT(&huart2, (uint8_t*)recvd_char, 1);

  //Startup ADC
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, adc_buf_len) ;

  // TIM2_CH1 start PWM
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1) ;
  // TIM2_CH4 start PWM - red LED
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) ;
  // TIM3_CH4 start PWM - GREEN LED
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  // TIM4_CH1 start PWM - BLUE LED
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) ;

  strobe_ticks  = HAL_GetTick() ;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // left button press to update system state (MF -> ME -> MM)
	  system_state_update() ;
	  //run adc and capture previous snapshot of ADC value and adc movement processing
	  adc_dma_val_processing();
	  // Turn LED ON/OFF
	  TURN_LED_ON_OFF() ;
	  // read UART params
	  convert_UART_state_params_to_Int() ;
	  // REAS sys state
	  Request_return_system_state() ;

	 // system state
	 if(button_count == 0 || start_up == 1 ){

		 start_up = 0 ; //for default MF state

		 MF_mode_LED() ; // sets the corresponding mode LED
		 em_count=0;     // reset the emergency mode count
		 em_default = 1; // to re-enter the EM state

		if(LED_ON == 1){
			 // if LED_ON and SLIDER MOVED -> updated LED intensity
			if(update_led_via_ADC == 1 && UART_state_update == 0){

				htim2.Instance->CCR1 =  LED_intensity ; // vary the duty cycle of the LED [1:512]
				MF_state = LED_intensity ;
				MF_param1 = param1  ;
			}
			else if(UART_state_update == 1 && state > 0 && set_or_ret_sys_state[3] =='F' ){

				htim2.Instance->CCR1 = state ;

				MF_state = state ;  // for when sys request made
				MF_param1 = param1  ;

				if(strcmp(Custom_Morse_Msg, "000") == 0){
					sprintf(ME_param2, "%d", param2) ;

				}else{
					MF_param2[0] = Custom_Morse_Msg[0];
					MF_param2[1] = Custom_Morse_Msg[1] ;
					MF_param2[2] = Custom_Morse_Msg[2] ;
				}
				UART_state_update = 0;
			}

		}
	 }else if(button_count == 1 ){// right button system state updated
		 ME_mode_LED() ; // sets the corresponding modes LED

		  // set EM mode states
		  Emergency_Mode_State_Update() ;

	 }else{
		 if(button_count == 2){ // Mood Mode
			 // SET THE NECESSARY STATES
			 em_count=0; // reset the emergency mode state
			 em_default = 1; // to re-enter EM state

			 MM_mode_LED() ; //sets the corresponding modes LED

			 Mood_Mode_State_Update() ; // update the necessary MM states
			 if(LED_ON == 1){

				 //red channel
				 htim2.Instance->CCR4 = R_channel_Intensity ;
				 MM_state = R_channel_Intensity ;
				 // GREEN channel
				 htim3.Instance->CCR4 = G_channel_Intensity ;
				 MM_param1 = G_channel_Intensity ;
				 // BLUE channel
				 htim4.Instance->CCR1 = B_channel_Intensity ;
				 MM_param2 = B_channel_Intensity ;


			 }else{
				 // put all channels off
				 //red channel
				 htim2.Instance->CCR4 =  0;
				 // GREEN channel
				 htim3.Instance->CCR4 = 0 ;
				 // BLUE channel
				 htim4.Instance->CCR1 = 0 ;
			 }

		 }
	 }

	 // right button state update
	 right_button_state_update() ;
	 //EMERGENCY MODES
	  if(button_count ==1 ){

		 if(em_count == 0 || em_default ==1){ //strobe wit default intensity
			 em_default = 0 ; //default state reached
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);


			 if(LED_ON){ //LED_on =?
				 // strobe LED with provided on time
				 EM_mode_Strobe(strobe_delay) ;
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
                              |RCC_PERIPHCLK_TIM2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 36;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 512;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 36;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
