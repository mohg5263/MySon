/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * AUTHOR:  Mohammad Ghanatian, PhD Student, Department of Mechanical Engineering, The University of Alabama
 *
 *
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
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Global Variables */
volatile uint8_t msg_ready = 0;
volatile uint8_t update_control_params = 0;
volatile uint8_t update_button=0;
volatile char uart_msg_buffer[50];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// limit data to be within bounds (specific to the AK80-9 motor)
#define P_MIN -12.5f	//position
#define P_MAX 12.5f
#define V_MIN -50.0f	//velocity
#define V_MAX 50.0f
#define T_MIN -18.0f	//torque
#define T_MAX 18.0f
#define Kp_MIN 0.0f
#define Kp_MAX 500.0f
#define Kd_MIN 0.0f
#define Kd_MAX 5.0f



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void CAN1_Tx_motor_on(int);
void CAN1_Tx_motor_zero(int);
void CAN1_Tx_motor_off(int);
void CAN1_Rx_message_read(void);
float interpolateHipAngle(float);
float interpolateKneeAngle(float);
void Stop_Motors(int, int, int, int);


/* Function Prototypes */
void handleRightButtonPress();
void handleLeftButtonPress();
void updateControlParameters();
void updateRamp(int motorIdx); // NEW: Prototype for updateRamp function



// Prototype of the template function

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];	//08 bytes of data transmit array, data type dictated by HAL
uint8_t RxData[8];	//08 bytes of data receive array, data type dictated by HAL

uint32_t TxMailbox;	//data type dictated by HAL



int heelSwitchLeft=2;
int heelSwitchRight=0;

int previous_heelSwitchRight = 0;
int previous_heelSwitchLeft = 0;

//Receiving parameter variables
//for motor 1
float position_1 = 0;
float velocity_1 = 0;
float torque_1 = 0;
float temp_1 = 0;
//for motor 2
float position_2 = 0;
float velocity_2 = 0;
float torque_2 = 0;
float temp_2 = 0;
//for motor 3
float position_3 = 0;
float velocity_3 = 0;
float torque_3 = 0;
float temp_3 = 0;
//for motor 4
float position_4 = 0;
float velocity_4 = 0;
float torque_4 = 0;
float temp_4 = 0;

float wPos1 = 0;
float wPos2 = 0;
float wPos3 = 0;
float wPos4 = 0;

float posF1 = 0;
float posF2 = 0;
float posF3 = 0;
float posF4 = 0;


float rAngle1=0;
float rAngle2=0;
float rAngle3=0;
float rAngle4=0;

//********************************************************************************





// PID variables
int motorID1 = 1;
int motorID2 = 2;
int motorID3 = 3;
int motorID4 = 4;

int motorActivated1 = 1;
int motorActivated2 = 1;
int motorActivated3 = 1;
int motorActivated4 = 1;

double Kp1;
double Kd1;

double Kp2;
double Kd2;

double Kp3;
double Kd3;

double Kp4;
double Kd4;

double samplingTime = 2.5;	//ms

double error1, lastError1 = 0;
double integral1 = 0;
double derivative1;
float torqueF1 = 0;
float speedF1 = 0;
float lastPos1;
float torqSignal1 = 0;

double error2, lastError2 = 0;
double integral2 = 0;
double derivative2;
float torqueF2 = 0;
float speedF2 = 0;
float lastPos2;
float torqSignal2 = 0;

double error3, lastError3 = 0;
double integral3 = 0;
double derivative3;
float torqueF3 = 0;
float speedF3 = 0;
float lastPos3;
float torqSignal3 = 0;

double error4, lastError4 = 0;
double integral4 = 0;
double derivative4;
float torqueF4 = 0;
float speedF4 = 0;
float lastPos4;
float torqSignal4 = 0;

float startTime = 0;
float currentTime = 0;
float ttime = 0;
float ttime1 = 0;
float ttime2 = 0;




uint32_t RAMP_DURATION = 40; // ms

// MODIFIED Ramp Struct for smooth transitions
typedef struct {
    float start;        // Starting value of the ramp
    float target;       // Desired final value of the ramp
    float current;      // Current value during the ramp
    uint32_t startTime; // Tick count when the ramp started
    uint8_t active;     // Flag to indicate if a ramp is in progress (using uint8_t as boolean)
    uint32_t duration;  // Duration of the ramp in milliseconds
} Ramp;

Ramp ramp[4]; // One for each rAngle1 to rAngle4






//********************************************************************************

float saturate(float x, float x_min, float x_max) {
	if (x < x_min)
		x = x_min;
	else if (x > x_max)
		x = x_max;
	return x;
}

//When sending packets, all numbers need to go through the following function to be converted into integer values before being sent to the motor
int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
	//Converts a float to an unsigned int, given range and number of intended bits
	float span = x_max - x_min;
	if (x < x_min)
		x = x_min;
	else if (x > x_max)
		x = x_max;
	return (int) ((x - x_min) * ((float) ((1 << bits) / span)));
}

//When receiving, convert all values to floating-point numbers using the following function
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
	// converts unsigned int to float, given range and number of intended bits
	float span = x_max - x_min;
	float offset = x_min;
	return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
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
  MX_CAN1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	if (HAL_CAN_Start(&hcan1) != HAL_OK) //CAN is started and checked for error.
			{
		Error_Handler();
	}
	CAN1_Tx_motor_on(motorID1);
	CAN1_Rx_message_read();

	CAN1_Tx_motor_zero(motorID1);
	CAN1_Rx_message_read();

	char msg[50];
	sprintf(msg, "Motor 1 Initialized!\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	CAN1_Tx_motor_on(motorID2);
	CAN1_Rx_message_read();

	CAN1_Tx_motor_zero(motorID2);
	CAN1_Rx_message_read();

	sprintf(msg, "Motor 2 Initialized!\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	CAN1_Tx_motor_on(motorID3);
	CAN1_Rx_message_read();

	CAN1_Tx_motor_zero(motorID3);
	CAN1_Rx_message_read();

	sprintf(msg, "Motor 3 Initialized!\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	CAN1_Tx_motor_on(motorID4);
	CAN1_Rx_message_read();

	CAN1_Tx_motor_zero(motorID4);
	CAN1_Rx_message_read();

	sprintf(msg, "Motor 4 Initialized!\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);



	// Start timer
	HAL_TIM_Base_Start_IT(&htim6);




	// MODIFIED: Initialize the ramps
	for (int i = 0; i < 4; i++) {
	    ramp[i].active = 0;     // Not active initially
	    ramp[i].current = 0.0f; // Start at 0 radians (or your initial desired position)
	    ramp[i].target = 0.0f;  // Target also 0 initially
	    ramp[i].start = 0.0f;   // Start of ramp also 0
	    ramp[i].startTime = 0;  // No start time yet
	    ramp[i].duration = RAMP_DURATION; // Use the defined constant
	}



//	cycle = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	  if (update_control_params) {
	      updateControlParameters();
	      update_control_params = 0;
	  }


	  if (update_button) {
		  updateButtons();
		  update_button = 0;
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

	//This is the CAN filter configuration (part of the CAN_INIT code)
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 0; // which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0; //this has to be mentioned (FIFO0/ FIFO1)
	canfilterconfig.FilterIdHigh = 0; //Next 04 lines --> no filter mask protocol is going on. All messages are allowed to pass to Nucleo (Accept all frames)
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = 0;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 0; // how many filters to assign to the CAN1 (master can)

	if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK) {
		Error_Handler();
	}
  /* USER CODE END CAN1_Init 2 */

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
  htim6.Init.Prescaler = 4-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 50000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void CAN1_Tx_motor_on(int motor_id) {
	TxHeader.DLC = 8;	//data length
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;	//standard 11 bit data frame, not extended.
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = motor_id;	//ID of the motor 01
	TxHeader.TransmitGlobalTime = DISABLE;

	TxData[0] = 0xFF;
	TxData[1] = 0xFF;
	TxData[2] = 0xFF;
	TxData[3] = 0xFF;
	TxData[4] = 0xFF;
	TxData[5] = 0xFF;
	TxData[6] = 0xFF;
	TxData[7] = 0xFC;

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
		// No available mailbox, wait and retry
		HAL_Delay(1); // Short delay to prevent a tight loop
		char msg[50];
		sprintf(msg, "Transmission needed wait!\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}

	//message is transmitted through CAN
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}

//	// Polling method, Wait for message to get out of pending state, that means wait until the message is transmitted successfully.
//	while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox))
//		;

}

void CAN1_Tx_motor_zero(int motor_id) {
	TxHeader.DLC = 8;	//data length
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;	//standard 11 bit data frame, not extended.
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = motor_id;	//ID of the motor 01
	TxHeader.TransmitGlobalTime = DISABLE;

	TxData[0] = 0xFF;
	TxData[1] = 0xFF;
	TxData[2] = 0xFF;
	TxData[3] = 0xFF;
	TxData[4] = 0xFF;
	TxData[5] = 0xFF;
	TxData[6] = 0xFF;
	TxData[7] = 0xFE;

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
		// No available mailbox, wait and retry
		HAL_Delay(1); // Short delay to prevent a tight loop
		char msg[50];
		sprintf(msg, "Transmission needed wait!\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}

	//message is transmitted through CAN
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}

//	// Polling method, Wait for message to get out of pending state, that means wait until the message is transmitted successfully.
//	while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox))
//		;
}

void CAN1_Tx_motor_off(int motor_id) {
	TxHeader.DLC = 8;	//data length
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;	//standard 11 bit data frame, not extended.
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = motor_id;	//ID of the motor 01
	TxHeader.TransmitGlobalTime = DISABLE;

	TxData[0] = 0xFF;
	TxData[1] = 0xFF;
	TxData[2] = 0xFF;
	TxData[3] = 0xFF;
	TxData[4] = 0xFF;
	TxData[5] = 0xFF;
	TxData[6] = 0xFF;
	TxData[7] = 0xFD;

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
		// No available mailbox, wait and retry
		HAL_Delay(1); // Short delay to prevent a tight loop
		char msg[50];
		sprintf(msg, "Transmission needed wait!\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}
	//message is transmitted through CAN
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}

//	// Polling method, Wait for message to get out of pending state, that means wait until the message is transmitted successfully.
//	while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox))
//		;
}

void CAN1_Tx_send_to_motor(int motor_id, float pos, float vel, float kp,
		float kd, float torque) {
//	// For UART
//	char msg[50];

	TxHeader.DLC = 8;	//data length 08 bytes
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;	//standard 11 bit data frame, not extended.
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = motor_id;	//ID of the motor 01
	TxHeader.TransmitGlobalTime = DISABLE;

	//ensure that the values of pos, vel, kp, kd, and torque fall within specified bounds
	pos = fminf(fmaxf(P_MIN, pos), P_MAX);
	vel = fminf(fmaxf(V_MIN, vel), V_MAX);
	kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
	kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
	torque = fminf(fmaxf(T_MIN, torque), T_MAX);

	// convert floats to unsigned integers of intended bit size (int uses 4 bytes, 32 bits (same as floats) in this compiler).
	int p_int = float_to_uint(pos, P_MIN, P_MAX, 16);
	int v_int = float_to_uint(vel, V_MIN, V_MAX, 12);
	int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
	int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
	int t_int = float_to_uint(torque, T_MIN, T_MAX, 12);

	// pack integer values into the CAN transfer data array
	//here, the integer values are divided into 08 bit outputs
	TxData[0] = p_int >> 8; // Position High 8
	TxData[1] = p_int & 0xFF; // Position Low 8
	TxData[2] = v_int >> 4; // Speed High 8 bits
	TxData[3] = ((v_int & 0xF) << 4) | (kp_int >> 8); // Speed Low 4 bits, KP High 4 bits
	TxData[4] = kp_int & 0xFF; // KP Low 8 bits
	TxData[5] = kd_int >> 4; // Kd High 8 bits
	TxData[6] = ((kd_int & 0xF) << 4) | (t_int >> 8); // KP Low 4 bits, Torque High 4 bits
	TxData[7] = t_int & 0xff; // Torque Low 8 bits

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
		// No available mailbox, wait and retry
		HAL_Delay(1); // Short delay to prevent a tight loop
		char msg[50];
		sprintf(msg, "Transmission needed wait!\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}
	//message is transmitted through CAN
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}

//	// Polling method, Wait for message to get out of pending state, that means wait until the message is transmitted successfully.
//	while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox));

}

void CAN1_Rx_message_read(void) {

//	char msg[50];	//for showing over the UART

	//we are waiting for at least one message into the RX FIFO0. When that happens, the following code will get out of the loop.
	//In RX FIFO, you can store 03 CAN messages.
	uint32_t startTick = HAL_GetTick();
	while (!HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0))
	{
	    if (HAL_GetTick() - startTick > 10)  // 100 ms timeout
	    {
	        // handle timeout
	        return;
	    }
	}

	//Polling based reception of message
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) //we have to specify the FIFO number
			{
		Error_Handler();
	}

	//Unpacking the received parameter integers from the RxData
	int id = RxData[0]; //motor ID
	int p_int = (RxData[1] << 8) | RxData[2];	//Motor position data
	int v_int = (RxData[3] << 4) | (RxData[4] >> 4);	//Motor velocity data
	int i_int = ((RxData[4] & 0xF) << 8) | RxData[5];	//Motor torque data
	int T_int = RxData[6];	//temperature data
	/// convert int to float
	float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
	float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
	float i = uint_to_float(i_int, T_MIN, T_MAX, 12);
	float T = T_int;
	if (id == motorID1)	//if motor 1 data is received
			{
		position_1 = p; // Read corresponding data based on ID
		velocity_1 = v;
		torque_1 = i;
		temp_1 = T - 40; // Temperature range: -40~215
	}
	if (id == motorID2)	//if motor 1 data is received
			{
		position_2 = p; // Read corresponding data based on ID
		velocity_2 = v;
		torque_2 = i;
		temp_2 = T - 40; // Temperature range: -40~215
	}
	if (id == motorID3)	//if motor 1 data is received
			{
		position_3 = p; // Read corresponding data based on ID
		velocity_3 = v;
		torque_3 = i;
		temp_3 = T - 40; // Temperature range: -40~215
	}
	if (id == motorID4)	//if motor 1 data is received
			{
		position_4 = p; // Read corresponding data based on ID
		velocity_4 = v;
		torque_4 = i;
		temp_4 = T - 40; // Temperature range: -40~215
	}

}




void signalMotors(float rAngle1, float rAngle2, float rAngle3, float rAngle4){

	// Command motor 1 and read current CAN message
	if (motorActivated1) {
		CAN1_Tx_send_to_motor(motorID1, -rAngle1, 0, Kp1, Kd1, 0); //position mode
	} else {
		CAN1_Tx_send_to_motor(motorID1, 0, 0, 0, 0, 0);
	}

	CAN1_Rx_message_read();
	posF1 = -position_1    * 180.0 / M_PI / 65 * 12;
	speedF1 = velocity_1;
	torqueF1 = torque_1;

	// Command motor 1 and read current CAN message
	if (motorActivated2) {
		CAN1_Tx_send_to_motor(motorID2, -rAngle2, 0, Kp2, Kd2, 0); //position mode
	} else {
		CAN1_Tx_send_to_motor(motorID2, 0, 0, 0, 0, 0);
	}

	CAN1_Rx_message_read();
	posF2 = -position_2    * 180.0 / M_PI / 65 * 12;
	speedF2 = velocity_2;
	torqueF2 = torque_2;

	// Command motor 1 and read current CAN message
	if (motorActivated3) {
		CAN1_Tx_send_to_motor(motorID3, rAngle3, 0, Kp3, Kd3, 0); //position mode
	} else {
		CAN1_Tx_send_to_motor(motorID3, 0, 0, 0, 0, 0);
	}

	CAN1_Rx_message_read();
	posF3 = position_3  * 180.0 / M_PI / 65 * 12;
	speedF3 = velocity_3;
	torqueF3 = torque_3;

	// Command motor 1 and read current CAN message
	if (motorActivated4) {
		CAN1_Tx_send_to_motor(motorID4, rAngle4, 0, Kp4, Kd4, 0); //position mode
	} else {
		CAN1_Tx_send_to_motor(motorID4, 0, 0, 0, 0, 0);
	}

	CAN1_Rx_message_read();
	posF4 = position_4 * 180.0 / M_PI / 65 * 12;
	speedF4 = velocity_4;
	torqueF4 = torque_4;






}


double hipFactorKp1=1;
double hipFactorKd1=1;


double AngleH1 = 20;
double AngleK1 = 5;
double AngleH2 = -5;
double AngleK2 = 10;
double AngleH3 = -20;
double AngleK3 = 15;
double AngleH4 = 40;
double AngleK4 = 50;


double tolerance=5;

//1      Swing Extension
double KpH1=3;
double KdH1=0.3;
double KpK1=3;
double KdK1=0.3;
//2     Stance Flexion/Extension
double KpH2=2.5;
double KdH2=0.3;
double KpK2=2.5;
double KdK2=0.3;
//3     Pre-Swing
double KpH3=3.5;
double KdH3=0.5;
double KpK3=3.5;
double KdK3=0.2;
//4     Swing Flexion
double KpH4=3.5;
double KdH4=0.2;
double KpK4=3.5;
double KdK4=0.2;

int skipForDelay=40;
int skippedRight=0;
int skippedLeft=0;



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    update_control_params = 1;
    update_button = 1;
}





void updateButtons(void){
    static uint8_t lastButtonStateRight = GPIO_PIN_RESET;
    static uint8_t lastButtonStateLeft = GPIO_PIN_RESET;

    uint8_t buttonStateRight = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);
    uint8_t buttonStateLeft = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);


	if (buttonStateRight == GPIO_PIN_SET && lastButtonStateRight == GPIO_PIN_RESET) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		if (heelSwitchRight==0 || heelSwitchRight==1) {
			heelSwitchRight++;
		    sprintf((char *)uart_msg_buffer, "Heel Switch Right Triggered!\r\n");
		    HAL_UART_Transmit(&huart2, (uint8_t *)uart_msg_buffer, strlen((char *)uart_msg_buffer), HAL_MAX_DELAY);
		}
	}

	if (buttonStateLeft == GPIO_PIN_SET && lastButtonStateLeft == GPIO_PIN_RESET) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		if (heelSwitchLeft==0 || heelSwitchLeft==1) {
			heelSwitchLeft=heelSwitchLeft+1;
	        sprintf((char *)uart_msg_buffer, "Heel Switch Left Triggered!\r\n");
	        HAL_UART_Transmit(&huart2, (uint8_t *)uart_msg_buffer, strlen((char *)uart_msg_buffer), HAL_MAX_DELAY);
		}
	}


    lastButtonStateRight = buttonStateRight;
    lastButtonStateLeft = buttonStateLeft;

}



void updateControlParameters() {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);

    // Define temporary variables for the calculated target angles (in radians for the motor)
    float rAngle_tar1, rAngle_tar2, rAngle_tar3, rAngle_tar4;

    // A small threshold to determine if a new ramp should be initiated
    // This prevents restarting a ramp if the target angle hasn't meaningfully changed
    const float ANGLE_CHANGE_THRESHOLD = 0.005f; // Radians, approx 0.28 degrees



    switch (heelSwitchLeft) {
        // Swing Extension
        case 1:
            Kp1 = KpH1;
            Kd1 = KdH1;
            Kp2 = KpK1;
            Kd2 = KdK1;
            rAngle_tar1 = AngleH1 * M_PI * 65 / (12 * 180.0);
            rAngle_tar2 = AngleK1 * M_PI * 65 / (12 * 180.0);
            break;
        // Stance
        case 2:
            Kp1 = KpH2;
            Kd1 = KdH2;
            Kp2 = KpK2;
            Kd2 = KdK2;
            rAngle_tar1 = AngleH2 * M_PI * 65 / (12 * 180.0);
            rAngle_tar2 = AngleK2 * M_PI * 65 / (12 * 180.0);
            if (-3 < speedF1 && speedF1 < 3 && posF1 < (AngleH2 + tolerance)) {
                heelSwitchLeft = 3;
            }
            break;
        // Preswing
        case 3:
            Kp1 = KpH3;
            Kd1 = KdH3;
            Kp2 = KpK3;
            Kd2 = KdK3;
            rAngle_tar1 = AngleH3 * M_PI * 65 / (12 * 180.0);
            rAngle_tar2 = AngleK3 * M_PI * 65 / (12 * 180.0);
            if (previous_heelSwitchRight == 1 && heelSwitchRight == 2 || heelSwitchRight == 3) {
                heelSwitchLeft = 5;
            }
            break;
        // Delay
        case 5:
            if (skippedLeft < skipForDelay) {
                skippedLeft++;
            } else {
                heelSwitchLeft = 4;
                skippedLeft = 0;
            }
            break;
        // Swing Flexion
        case 4:
            Kp1 = KpH4;
            Kd1 = KdH4;
            Kp2 = KpK4;
            Kd2 = KdK4;
            rAngle_tar1 = AngleH4 * M_PI * 65 / (12 * 180.0);
            rAngle_tar2 = AngleK4 * M_PI * 65 / (12 * 180.0);
            if (-1 < speedF2 && speedF2 < 1 && posF2 > (AngleK4 - tolerance)) {
                heelSwitchLeft = 1;
            }
            break;
    }



    // Initiate/Update Ramp for Motor 1 (Left Hip)
    // Check if the calculated target is significantly different or if the ramp is not active
    if (fabs(ramp[0].target - rAngle_tar1) > ANGLE_CHANGE_THRESHOLD && ramp[0].active == 0) {
        ramp[0].start = ramp[0].current; // Start the ramp from the current position
        ramp[0].target = rAngle_tar1;
        ramp[0].startTime = HAL_GetTick();
        ramp[0].active = 1;
        ramp[0].duration = RAMP_DURATION;
    }
    // Initiate/Update Ramp for Motor 2 (Left Knee)
    if (fabs(ramp[1].target - rAngle_tar2) > ANGLE_CHANGE_THRESHOLD && ramp[1].active == 0) {
        ramp[1].start = ramp[1].current;
        ramp[1].target = rAngle_tar2;
        ramp[1].startTime = HAL_GetTick();
        ramp[1].active = 1;
        ramp[1].duration = RAMP_DURATION;
    }




    switch (heelSwitchRight) {
        case 0:
            Kp3 = KpH1;
            Kd3 = KdH1;
            Kp4 = KpK1;
            Kd4 = KdK1;
            rAngle_tar3 = 0;
            rAngle_tar4 = 0;
            break;
        case 1:
            Kp3 = KpH1;
            Kd3 = KdH1;
            Kp4 = KpK1;
            Kd4 = KdK1;
            rAngle_tar3 = AngleH1 * M_PI * 65 / (12 * 180.0);
            rAngle_tar4 = AngleK1 * M_PI * 65 / (12 * 180.0);
            break;
        case 2:
            Kp3 = KpH2;
            Kd3 = KdH2;
            Kp4 = KpK2;
            Kd4 = KdK2;
            rAngle_tar3 = AngleH2 * M_PI * 65 / (12 * 180.0);
            rAngle_tar4 = AngleK2 * M_PI * 65 / (12 * 180.0);
            if (-3 < speedF3 && speedF3 < 3 && posF3 < (AngleH2 + tolerance)) {
                heelSwitchRight = 3;
            }
            break;
        case 3:
            Kp3 = KpH3;
            Kd3 = KdH3;
            Kp4 = KpK3;
            Kd4 = KdK3;
            rAngle_tar3 = AngleH3 * M_PI * 65 / (12 * 180.0);
            rAngle_tar4 = AngleK3 * M_PI * 65 / (12 * 180.0);
            if (previous_heelSwitchLeft == 1 && heelSwitchLeft == 2 || heelSwitchLeft==3) {
                heelSwitchRight = 5;
            }
            break;

        case 5:
            if (skippedRight < skipForDelay) {
                skippedRight++;
            } else {
                heelSwitchRight = 4;
                skippedRight = 0;
            }
            break;

        case 4:
            Kp3 = KpH4;
            Kd3 = KdH4;
            Kp4 = KpK4;
            Kd4 = KdK4;
            rAngle_tar3 = AngleH4 * M_PI * 65 / (12 * 180.0);
            rAngle_tar4 = AngleK4 * M_PI * 65 / (12 * 180.0);
            if (-1 < speedF4 && speedF4 < 1 && posF4 > (AngleH4 - tolerance)) {
                heelSwitchRight = 1;
            }
            break;
    }



    // Initiate/Update Ramp for Motor 3 (Right Hip)
	if (fabs(ramp[2].target - rAngle_tar3) > ANGLE_CHANGE_THRESHOLD && ramp[2].active == 0) {
		ramp[2].start = ramp[2].current;
		ramp[2].target = rAngle_tar3;
		ramp[2].startTime = HAL_GetTick();
		ramp[2].active = 1;
		ramp[2].duration = RAMP_DURATION;
	}
	// Initiate/Update Ramp for Motor 4 (Right Knee)
	if (fabs(ramp[3].target - rAngle_tar4) > ANGLE_CHANGE_THRESHOLD && ramp[3].active == 0) {
		ramp[3].start = ramp[3].current;
		ramp[3].target = rAngle_tar4;
		ramp[3].startTime = HAL_GetTick();
		ramp[3].active = 1;
		ramp[3].duration = RAMP_DURATION;
	}




	// --- Apply Ramping ---
	// Call updateRamp for each motor to smoothly change its current setpoint
	updateRamp(0); // Motor 1 (Left Hip)
	updateRamp(1); // Motor 2 (Left Knee)
	updateRamp(2); // Motor 3 (Right Hip)
	updateRamp(3); // Motor 4 (Right Knee)

	// Assign the current ramped values to the global rAngle variables
	// These are the actual setpoints sent to the motors
	rAngle1 = ramp[0].current;
	rAngle2 = ramp[1].current;
	rAngle3 = ramp[2].current;
	rAngle4 = ramp[3].current;





    signalMotors(rAngle1, rAngle2, rAngle3, rAngle4);
    previous_heelSwitchRight = heelSwitchRight;
    previous_heelSwitchLeft = heelSwitchLeft;


    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
}


// NEW: Ramping function
void updateRamp(int motorIdx) {
    if (ramp[motorIdx].active) {
        uint32_t elapsedTime = HAL_GetTick() - ramp[motorIdx].startTime;
        float progress = (float)elapsedTime / (float)ramp[motorIdx].duration;

        if (progress >= 1.0f) {
            progress = 1.0f;
            ramp[motorIdx].active = 0; // Ramp finished
        }

        // Linear interpolation: current = start + (target - start) * progress
        ramp[motorIdx].current = ramp[motorIdx].start + (ramp[motorIdx].target - ramp[motorIdx].start) * progress;
    }
}



void Stop_Motors(int motor1, int motor2, int motor3, int motor4) {

	if (motor1) {
		motorActivated1 = 0;
	}
	if (motor2) {
		motorActivated2 = 0;
	}
	if (motor3) {
		motorActivated3 = 0;
	}
	if (motor4) {
		motorActivated4 = 0;
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
	char msg[50];
	sprintf(msg, "Error Handler Trigerred!\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	Stop_Motors(1, 1, 1, 1);

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
