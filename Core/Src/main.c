/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WIGGLEANGLE 500
#define BUFFERSIZE 8
#define FULLROTATION 4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t stepCount = 0;
uint32_t startsteps = 0;
uint16_t position = 0;
uint32_t setpos = 0;
uint32_t offset = 0;
uint32_t potipos = 0;
uint32_t charCounter = 0;
uint32_t timersteps = 0;
_Bool direction = false;
_Bool wiggleFlag = false;
_Bool startFlag = true;
_Bool setTimeFlag = false;
_Bool switchstate = false;
_Bool rxDone = false;
_Bool buttonPressed = false;
_Bool transmitDone = false;
_Bool doneOnce = false;
static _Bool receiveFlag = false;
int datapointer = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t receiveBuffer[BUFFERSIZE];
char *receiveMsg = "\rAck   \n ";
char *notAcknowledge = "\rNAck   \n";
char *waitForInput = "\rInput: \n";
char dataBuffer[200];

typedef enum{
	  start_state,
	  set_time_state,
	  timer_state,
	  ring_state
}System_state;

System_state system_state = start_state;

void receiveUART(UART_HandleTypeDef uart, uint8_t receiveBuffer[BUFFERSIZE]){
	  HAL_UART_Receive_IT(&huart2, receiveBuffer, BUFFERSIZE);
	  HAL_UART_Receive_IT(&huart1, receiveBuffer, BUFFERSIZE);
	  	  for (;rxDone == false;){
	  		  datapointer = 0;
				if(dataBuffer[datapointer] == '#'){
					datapointer++;
				  if(dataBuffer[datapointer] ==  'p'){
					  datapointer++;
					  for(; dataBuffer[datapointer] != ','; datapointer++);
					  sscanf(dataBuffer+1+datapointer, "%d\n", &potipos);
					  buttonPressed = true;
					  rxDone = true;
					  }

				}
	  	  }

	  	  for(datapointer = 0; dataBuffer[datapointer] != 0; datapointer++){
	  		  dataBuffer[datapointer] = 0;
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
	char uart_buf[50];
	int uart_buf_len = 0;

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
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MS2_GPIO_Port, MS2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim6);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  switch(system_state) { //State machine wechselt zwischen den verschiedenen zuständen die das System einnehmen kann

	  	  case set_time_state:{ //State um die Zeit einzustellen
	  		setTimeFlag = true;

	  		if(!receiveFlag){
	  			HAL_UART_Transmit_IT(&huart2, (uint8_t*)waitForInput, BUFFERSIZE);
	  			receiveUART(huart2, receiveBuffer);
	  			receiveFlag = true;
	  		}

	  		if(buttonPressed){
	  			float temp = potipos;
	  			setpos = (int) 4096 * (temp / 100);
	  			HAL_NVIC_EnableIRQ(54);
	  			buttonPressed = false;
	  		}

	  		if(switchstate){ //Motor hat sich zu der gewünschten Position bewegt
	  			system_state = timer_state;
	  			receiveFlag = false;
	  			switchstate = false;
	  			doneOnce = false;

	  		}
	  	  }break;

	  	  case timer_state:{ //state während die Zeit abläuft

	  		HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_SET);
	  		HAL_GPIO_WritePin(MS2_GPIO_Port, MS2_Pin, GPIO_PIN_SET);

	  		if (!doneOnce){ //enabled beim ersten durchlauf wieder den Interrupt, damit sich der Motor bewegt
	  			HAL_NVIC_EnableIRQ(54);
	  			doneOnce = true;
	  		}

			  if(switchstate){ //bereitet die Variablen für den nächsten state vor
				   system_state = ring_state;
				   receiveFlag = false;
				   switchstate = false;
				   rxDone = false;
				   stepCount = 0;

			  }
	  	  }break;

	  	  case ring_state:{ //state während dem klingeln
	  		  wiggleFlag = true;
	  		  HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_RESET); //wechselt Geschwindigkeit
	  		  HAL_GPIO_WritePin(MS2_GPIO_Port, MS2_Pin, GPIO_PIN_RESET);

	  		  if(!receiveFlag){
	  			  HAL_UART_Transmit_IT(&huart2, (uint8_t*)waitForInput, BUFFERSIZE);
	  			  receiveUART(huart2, receiveBuffer);
	  			  receiveFlag = true;
	  		  }

	  		  if(buttonPressed){//bereitet die Variablen für den nächsten state vor
	  			  system_state = set_time_state;
	  			  receiveFlag = false;
	  			  buttonPressed = false;
	  			  rxDone = false;
	  			  //HAL_NVIC_EnableIRQ(54);
	  		  }
	  	  }break;

	  	  case start_state:{ //start state, zeigt wenn bereit

	  		  startFlag = true;

	  		  if(switchstate){//bereitet die Variablen für den nächsten state vor
	  			  system_state = set_time_state;
	  			  switchstate = false;
	  		  }
	  	  }break;

	  	  default:{
	  		  system_state = start_state;
	  		  break;
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 32000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MS1_Pin|EN_Pin|MS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR_Pin|STP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MS1_Pin EN_Pin MS2_Pin */
  GPIO_InitStruct.Pin = MS1_Pin|EN_Pin|MS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin STP_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|STP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	HAL_GPIO_TogglePin(STP_GPIO_Port, STP_Pin);


	switch(system_state){
		case set_time_state:{
			if (position < setpos){ //richtige Richtung einstellen um zur gewünschten Position zu fahren
				HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, RESET);
				direction = false;
			} else {
				HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, SET);
				direction = true;
			}
			if(position == setpos){ //wenn die gewünschte Position erreicht wurde
				HAL_NVIC_DisableIRQ(54);
				switchstate = true;
			}
			break;
		}
		case timer_state:{
			if(direction == false){ //change direction
				HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, SET);
				direction = true;
			}
			if(position == 0){ //Zeit abgelaufen
				//HAL_NVIC_DisableIRQ(54);
				switchstate = true;
				HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, RESET); //prepare direction for ring_state
				direction = false;

			}
			break;
		}
		case ring_state:{
			stepCount++;
			if (stepCount%WIGGLEANGLE == 0){ //Wenn es sich um den Winkel bewegt hat
				stepCount = stepCount % WIGGLEANGLE;
				//HAL_GPIO_TogglePin(DIR_GPIO_Port, DIR_Pin);
				if(direction == false){ //Richtung wechseln wenn Winkel richtig
					HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, SET);
					direction = true;
				}else{
					HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, RESET);
					direction = false;
				}
			}
			break;
		}
		case start_state:{
			startsteps++;
			if(startsteps < WIGGLEANGLE * 2){//einmal klingeln
				if(startsteps <= WIGGLEANGLE){
					HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, RESET);
					direction = false;
				} else if (startsteps > WIGGLEANGLE) {
					HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, SET);
					direction = true;
				}
			}else{
				HAL_NVIC_DisableIRQ(54);//Motor stoppen
				switchstate = true;
			}
			break;
		}
	}

	if(direction == false){ //false entspricht einer Drehung laut Uhrzeigersinn und Position ++
		//position++;

		if(system_state == timer_state){
			timersteps++;
			if(timersteps%8 == 0){
				timersteps = 0;
				position++;
			}
		}else{
			position++;
		}
		position = position % FULLROTATION;
	}else { //andere Richtung entspricht position--
		if(system_state == timer_state){
			timersteps++;
			if(timersteps%8 == 0){
				timersteps = 0;
				position--;
			}
		}else{
			position--;
		}


		position = position % FULLROTATION;
	}

	//if(stepCount == 2048*2){		Function to measure steps
		//HAL_NVIC_DisableIRQ(54);
	//}*/
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	for(uint8_t i = 0; i <= 7; i++){ //Kopiert alles aus dem UART buffer in mein Databuffer
		dataBuffer[i] = receiveBuffer[i];
		if(receiveBuffer[i] == '\r'){
			rxDone = false;
			charCounter = i;
		}
	}

	HAL_UART_Transmit_IT(&huart2, (uint8_t *)receiveMsg, BUFFERSIZE); //schickt Acknowledge
	HAL_NVIC_DisableIRQ(54);

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	transmitDone = true;
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
