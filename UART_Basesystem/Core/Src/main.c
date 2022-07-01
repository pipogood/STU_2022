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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Parameter for Adjust
#define dt 0.01
#define Q 100000000
#define R 500000000
#define NFW_ADDR (0x23)<<1
#define NFR_ADDR ((0x23)<<1)+1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
float velo_Kp = 0;
float velo_Ki = 0;
float velo_Kd = 0;
float pos_Kp = 4.07697250571067;
float pos_Ki = 3.71617437483172;
float pos_Kd = 0.25445435315971;
uint8_t setzero = 1;
float offset = 1.565;
/* ---------------------------- < Communications's VARIABLE > ------ */

// ---------------------------- < State machine >
uint8_t nDestination = 0;			// Amount of destination.
uint16_t Destination[50] = {0};		// List of destinations.

uint8_t nStation = 0;				// Amount of station.			(NOT AVAILABLE, JUST FOR SHOWING)
uint8_t Station[50] = {0};			// Station stack.				(NOT AVAILABLE, JUST FOR SHOWING)
uint8_t nowStation = 0;				// Now position of machine.		(NOT AVAILABLE, JUST FOR SHOWING)
uint8_t j = 0;

uint8_t Reached = 0;				// Reached destination. (final destination)
uint8_t movingFlag = 0;				// Moving flag for each command(5,8,14). -> can implement single variable and assign different value

float rad = 0;
float limitOmega = 5;				// Limited angular velocity by the Base system.
float topOmega = 0;					// Top angular velocity recorded.ของเรา
uint8_t n = 0;						// Check for count

// ---------------------------- < UART >
uint8_t ACK1[2] = {88,117};
uint8_t ACK2[2] = {70,110};

uint8_t TxState = 0;				// Transmit state.
uint8_t RxState = 0;				// Receive state.
uint8_t Start = 0;					// Start bit of command.
uint8_t Mode = 0;					// Mode bit of command.
uint8_t nbyte = 1;					// 1 bytes by default.
uint8_t sum = 0;					// Sum of frame.

uint8_t Transmit[4] = {0};			// Transmit Buffer.
uint8_t Receive[16] = {0};			// Receive Buffer	-> Receive 1 bytes by default. ,260 bytes are maximum all of case. (+1(Header)+1(n Station)+256(max Station)+1(Checksum)=259bytes)
uint8_t keepACK[2] = {0};

uint8_t RxComplete = 0;			// Interrupt each period buffer(Receive[nbyte]).
uint8_t endReceive = 0;				// Flag to reset after finished transmission of frame.
uint8_t ACKFlag = 0;				// Received ACK flag.
uint32_t timeStamp = 0;				// Optional.
uint16_t HMEtimeStamp = 0;		// time stamp for transmit zero(theta) to base system.

//I2C
uint8_t NFREG_ON = 0x45;
uint8_t NFREG_INIT = 0x23;

uint8_t NFenable = 1;
uint8_t NFstate = 0;
uint8_t NFwriteFlag = 1;
uint8_t NFreadFlag = 0;

uint16_t NFtimest = 0;
uint8_t stateI2C = 0;

//spi
const float PI = 3.14159265;
uint8_t AMTGETPOS[2] 	= { 0x00, 0x00};					// Read position.
uint8_t AMTGETTURN[4] 	= { 0x00, 0xA0, 0x00, 0x00};		// Read turn.
uint8_t AMTRESET[2] 	= { 0x00, 0x60};					// Reset encoder.
uint8_t AMTHbyte;										// Received data high byte.
uint8_t AMTLbyte;										// Received data low byte.
uint8_t AMTlogic_checkbit = 0;								// Checkbit logic.
uint16_t rawPos = 0;										// Position variable
float current_rad = 0;												// Angle in radiant unit.
float current_rad_wrap = 0;
float deg = 0;												// Angle in degree unit.
uint8_t trigger = 0;
uint8_t AMTstate = 0;
uint8_t AMTcomplete = 1;

//Unwrapping
float p_n = 0;
float p_n_1 = 0; // input data ที่ n-1
float p_0 = 0; // ตัว�?ปร p0[n]
float p_0_1 = 0; // ตัว�?ปร p0[n-1]
float p_max = 2*PI; // �?ำหนดให้ค่า p_max อยู่ที่ 2*pi
float e = 0.65*2*PI;

//Parameter for coding
uint64_t _micro = 0;
uint8_t stop = 0;
float kal_position = 0; //Collect Result
float kal_velocity = 0; //Collect Result
float kal_acceleration = 0; //Collect Result
float tra_pos = 0;
float tra_velo = 0;
float tra_acc = 0;
static float tb;
static float vb_n;
static float tf;
float tua;
static float tuaall;
float tuastart = 0.1;
float inputpos = 0; //rad_in
float inputdeg = 0; //deg_in
float inputrpm = 5;
float postotra = 0;
float rad_before;
uint8_t direct = 1;
uint8_t update = 0;
uint8_t blue = 0;
uint8_t start_tra;
enum{Idle,Working, Laser,Emerstop}State = Idle;
float velostart;
uint8_t PIDon = 0;
float ch_velo;
float finish = 0;
float firstcheck = 1;

//PID
float limMin = -100;
float limMax = 100;
float integrator;
float prevError;
float differentiator;
float prevMeasurement;
float integrator2;
float prevError2;
float differentiator2;
float prevMeasurement2;
float PIDout;
float poscommand;
float velocom;
float error;
float volt_controller;

//Parameters For Kalman filer
float DegAbs[3][3]; // Z_k
float multiply[3][3];
float pm3_3[3][3];
float collect3_3[3][3];
float y_k[3][3];
float S_k[3][3];
float K_k[3][3];
float I[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
float F_t[3][3] = {{1,dt,(dt*dt)/2},{0,1,dt},{0,0,1}};
float F_t_tran[3][3] = {{1,0,0},{dt,1,0},{(dt*dt)/2,dt,1}};
float H_t[3][3] = {{0,1,0}};
float H_t_tran[3][3] = {{0},{1},{0}};
float R_t[3][3] = {{R}};
float X_t[3][3];
float X_hat_t[3][3] = {{0},{0},{0}};
float P_t[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
float P_hat_t[3][3];
float Q_t[3][3] = {{Q*dt*dt*dt*dt/4,Q*dt*dt*dt/2,Q*dt*dt/2},{Q*dt*dt*dt/2,Q*dt*dt,Q*dt},{Q*dt*dt/2,Q*dt,Q}}; // G*Q_variance*G'
float velo_lowpass;
float prevlowpass;
float velo_diff;
float prevPos;
float pos_lowpass;
float prevposlowpass;

float velo_diff2;
float prevPos2;

//Inverse tranfer
float volt_inverse;
float x_n;
float x_n_1;
float x_n_2;
float y_n_1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
//Control
void Trajectory(float qf,float vb);
void Drivemotor(int PWM);
uint64_t micros();
uint32_t aaabs(int x);
void Multiply_matrix(float A[][3], float B[][3],float multiply[][3],int row1,int col1,int row2,int col2);
void PlusOrMinus(float A[][3], float B[][3],int row,int col,int con);
void equal(float A[][3],float B[][3],int row,int col);
void prediction(X_hat_t_1, P_t_1, F_t, Q_t);
void Update(X_hat_t, P_hat_t, DegAbs, R_t, H_t);
void CascadeController();
void lowpass();
void reset();
void Diff_velo();
void inverse_tran();
void MoveToStation();
void set_home();
void Kalman_filter();
//spi
void AMT222getpos();
uint8_t AMT222checkbit( uint8_t *H, uint8_t *L);
void microsecDelay(uint8_t delayTime);
void unwrapping();
//uart
void callUART();
void responseUART();
void destinationACK();
uint16_t tranStation(uint8_t num);
uint8_t frameConfig(uint8_t mode);
uint8_t checkSum(uint8_t var);
uint8_t frameSum(uint8_t *frame, uint8_t num);
//I2C
void NFcontacton();
void NFgetstate();

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim11);
  HAL_UART_Receive_DMA(&huart2, Receive, nbyte);
  NFgetstate();
  NFstate = 0x78;
  //PWM Generator set
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  //Encoder set
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  static uint64_t timeset = 0;
	  static uint64_t timeset3 = 0;
	  static uint64_t timeStamp = 0;
	  static GPIO_PinState B1State2[2] = {0};
	  B1State2[0]= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);

	  if(AMTcomplete || micros()-timeStamp>dt*1000){
	  		 AMT222getpos(&rawPos);
	  	  }

	  switch(State){
	  default:
		  case Idle:
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			  stop = 0;
			  if (micros() - timeset3 > dt*1000000){ //Read Value while Idle
				  lowpass();
				  Diff_velo();
				  Kalman_filter();
				  timeset3 = micros();
			  }

			  if(setzero == 1){ // Condition for Set home
				  set_home();
			  }

			  if(nDestination != 0){
				  update = 1;
			  }
			  else{
				  update = 0;
			  }

			  if(update == 1){ //UART Update
				  MoveToStation();
			  }

			  if(blue == 1){ // Blue Button Switch
				  postotra = 3.14;
				  limitOmega = 5;
				  direct = 1;
				  rad_before = current_rad;
				  State = Working;
			  }
			  break;

		  case Working:
			blue = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			if (micros() - timeset > dt*1000000) { //1000 Hz
				lowpass();
				Diff_velo();
				Kalman_filter();
				if(postotra > 0.2){
					Trajectory(postotra,limitOmega);
					inverse_tran();
					CascadeController();
					/*if(PIDon == 1){
						inverse_tran();
						CascadeController();
					}
					else{
						if(postotra < 0.349){
							PIDon = 1;
						}
						if(direct == 1){
							PIDout = 2500;
							if(velo_diff >= tra_velo && tra_velo != 0){
								PIDon = 1;
							}
						}
						else{
							PIDout = -2500;
							if(velo_diff <= tra_velo && tra_velo != 0){
								PIDon = 1;
							}
						}
						error = tra_velo - velo_diff;
						prevError = error;
					}*/
				}
				else{
					if(tuaall < 0.1){ //short distant 5,10 degree
						if(direct == 1){
							PIDout = 2100;}
						else{
							PIDout = -2100;}
					}
					else{
						if(direct == 1){
							PIDout = 1300;}
						else{
							PIDout = -1300;}
					}
					tf = 0;
				}
				Drivemotor(PIDout);
				tuaall += dt;
				timeset = micros();
			}

			if(tuaall > tf && setzero == 0){ //End of working should do this condition
				if(direct == 1){
					if(deg > Destination[n]-0.1){
						finish = 1;
					}
				}
				else
				{
					if(deg < Destination[n]+0.7){
						finish = 1;
					}
				}
				if(finish == 1){
					n++;
					if(n < nDestination){
						update = 1;}
					else{
						update = 0;
						if(n == nStation || n == nDestination){
							Reached = 1;
						}
						nDestination = 0;
						n = 0;
					}
					State = Laser;
					NFwriteFlag = 1;
				}
			}

			if(tuaall > 15 && tuaall > tf){ //Condition when moving error
				n++;
				if(n < nDestination){
					update = 1;}
				else{
					update = 0;
					if(n == nStation || n == nDestination){
						Reached = 1;
					}
					nDestination = 0;
					n = 0;
				}
				State = Laser;
				NFwriteFlag = 1;
			}

			if(velo_diff > topOmega){
				topOmega = velo_diff*9.5493;
			}

			if(stop == 1){
				State = Emerstop;
				Drivemotor(0);
			}
			break;

		  case Emerstop:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			if(B1State2[1] == GPIO_PIN_SET && B1State2[0] == GPIO_PIN_RESET){
				reset();
				nDestination = 0;
				n = 0;
				State = Idle;
				setzero = 1;
			}
			B1State2[1] = B1State2[0];
			break;

		  case Laser:
			  if(setzero == 1){
				  State= Idle;
				  reset();
				  break;
			  }
			  reset();
			  //State= Idle;
			  switch(stateI2C){
			  // Reached state
			  case 0:
				  switch(NFenable){
				  case 1:
					  NFgetstate();
					  if( NFwriteFlag && NFstate == 0x78 && (hi2c1.State == HAL_I2C_STATE_READY) ){
						  NFcontacton();
						  NFwriteFlag = 0;		// Reached var
					  }else if(NFstate == 0x12 || NFstate == 0x34 || NFstate == 0x56){
						  stateI2C = 1;
						  NFstate = 0;
					  }
					  break;
				  case 0:
					  NFwriteFlag = 0;
					  State = Idle;
					  // state = start;
					  break;
				  }
				  break;
			  case 1:
				  NFgetstate();
				  if(NFstate == 0x78){
					  stateI2C = 0;
					  State = Idle;
				  }
				  break;
			  }
	  }

	  responseUART();
	  rad = current_rad_wrap;

	  // Keep this work flow at the bottom of while loop. bo Reached will be reset here.
	  destinationACK();

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 99;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  huart2.Init.BaudRate = 512000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SS_Pin */
  GPIO_InitStruct.Pin = SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

//----------------------------------------------------------------< CONTROL >
void CascadeController()
{
	/*if(tuaall < tb/2 && postotra > 0.2){
		pos_Kp = 3.07697250571067;
		pos_Ki = 0.71617437483172;
		pos_Kd = 0.25445435315971;
		velo_Kp = 0.5;
		velo_Ki = 0;
		velo_Kd = 0;
		/*if(direct == 1){
			if(kal_velocity > tra_velo){
				kal_velocity = tra_velo;
			}
		}
		else{
			if(kal_velocity < tra_velo){
				kal_velocity = tra_velo;
			}
		}
	}*/
	if(start_tra == 2){
		pos_Kp = 4.07697250571067;
		pos_Ki = 3.71617437483172;
		pos_Kd = 0.75445435315971;
		velo_Kp = 1.5;
		velo_Ki = 0;
		velo_Kd = 0;
	}
	else{
		pos_Kp = 4.07697250571067;
		pos_Ki = 3.71617437483172;
		pos_Kd = 0.75445435315971;
		velo_Kp = 1.5;
		velo_Ki = 0;
		velo_Kd = 0;
	}
	//position control******
	float error2 = tra_pos - current_rad_wrap; //setpoint - new_Data
	float proportional2 = pos_Kp * error2;
	integrator2 = integrator2 + (0.5*pos_Ki*dt*(error2+prevError2));
	float limMinInt2, limMaxInt2;
	if(limMax > proportional2){
		limMaxInt2 = limMax - proportional2;}
	else{
		limMaxInt2 = 0.0;}
	if(limMin < proportional2){
		limMinInt2 = limMin - proportional2;}
	else{
		limMinInt2 = 0.0;}
	//Clamp integrator
	if(integrator2 > limMaxInt2){
		integrator2 = limMaxInt2;}
	else if(integrator2 < limMinInt2){
		integrator2 = limMinInt2;}

	//differentiator2 = (2*pos_Kd*(kal_position-prevMeasurement2)+(2*tuaall-dt)*differentiator2)/(2*tuaall+dt);
	differentiator2 = error2 - prevError2;
	poscommand = proportional2 + integrator2 + (pos_Kd*differentiator2);

	//velocity control*******

	if(direct == 1){
		ch_velo = kal_velocity;
		if(ch_velo > limitOmega/9.5493){
			kal_velocity = limitOmega/9.5493;
		}
	}
	else{
		ch_velo = kal_velocity*-1;
		if(ch_velo > limitOmega/9.5493){
			kal_velocity = (limitOmega/9.5493)*-1;
		}
	}

	error = tra_velo+poscommand - kal_velocity; //setpoint+poscom - new_Data
	float proportional = velo_Kp * error;
	integrator = integrator + (0.5*velo_Ki*dt*(error+prevError));
	float limMinInt, limMaxInt;
	if(limMax > proportional){
		limMaxInt = limMax - proportional;}
	else{
		limMaxInt = 0.0;}
	if(limMin < proportional){
		limMinInt = limMin - proportional;}
	else{
		limMinInt = 0.0;}
	//Clamp integrator
	if(integrator > limMaxInt){
		integrator = limMaxInt;}
	else if(integrator < limMinInt){
		integrator = limMinInt;}

	//differentiator = (2*velo_Kd*(kal_velocity-prevMeasurement)+(2*tuaall-dt)*differentiator)/(2*tuaall+dt);
	differentiator = error - prevError;
	volt_controller = proportional + integrator + (velo_Kd*differentiator);
	if(volt_controller > 12.0){
		volt_controller = 12.0;}
	else if(volt_controller < -12.0){
		volt_controller = -12.0;}

	//PIDout = PIDout*10000/12.0;
	PIDout = (volt_inverse+volt_controller)*10000/12.0;
	/*if(tuaall > tf){
		if(direct == 1){
			PIDout = 1200;}
		else{
			PIDout = -1200;
		}
	}*/
	//****try****//
	/*if(limitOmega > 4){
		tuastart = 0;
	}
	else{
		tuastart = 0;
	}

	if(tuaall <= tuastart){
		if(direct == 1){
			PIDout = 2500;}
		else{
			PIDout = -2500;
		}
	}*/
	/*if(start_tra == 1){
		if(PIDout < 2500 && PIDout > -2500){
			if(direct == 1){
				PIDout = 2500;}
			else{
				PIDout = -2500;
			}
		}
	}*/
	/*if(start_tra ==1){
		if(direct == 1){
			if(PIDout < 0){
				PIDout = 0;
			}
		}
		else{
			if(PIDout < 0){
				PIDout = 0;
			}
		}
	}*/
	prevError2 = error2;
	prevMeasurement2 = current_rad_wrap;
	prevError = error;
	prevMeasurement = kal_velocity;
}

void reset()
{
	tuaall = 0;
	Drivemotor(0);
	PIDon = 0;
	PIDout = 0;
	finish = 0;
}

void set_home()
{
	  static uint64_t timeset2 = 0;
	  if (timeset2 < 500000) {
		  Drivemotor(-2500);
		  ;}
	  else if (timeset2 < 1500000) {
		  Drivemotor(2500);
		  ;}
	  else{
		  postotra = 0 - current_rad_wrap;
		  rad_before = current_rad_wrap;
		  Drivemotor(-2500);
		  if(current_rad >= 1.55 && current_rad <= 1.6){
			  Reached = 1;
			  HMEtimeStamp = HAL_GetTick();
			  setzero = 0;
			  Drivemotor(0);
		  }
	  }
	  timeset2 = micros();
}

void MoveToStation()
{
	  topOmega = 0;
	  if(firstcheck == 1){
		  if(Destination[n] != 0){
			  inputpos = Destination[n]/57.2957795;
			  postotra = inputpos-current_rad_wrap;
			  rad_before = current_rad_wrap;
			  if(postotra >= 0){
				  direct = 1;}
			  else{
				  postotra = postotra*-1;
				  direct = 0;}
			  State = Working;
			  firstcheck = 0;
			  update = 0;
		  }
	  }
	  else{
		  inputpos = Destination[n]/57.2957795;
		  postotra = inputpos-current_rad_wrap;
		  rad_before = current_rad_wrap;
		  if(postotra >= 0){
			  direct = 1;}
		  else{
			  postotra = postotra*-1;
			  direct = 0;}
		  State = Working;
		  update = 0;
	  }
}

void inverse_tran(){
	x_n = tra_velo;
	volt_inverse = (x_n - (0.9724*x_n_1) + (0.003346*x_n_2) - (0.0004612*y_n_1))/0.002803;
	x_n_2 = x_n_1;
	x_n_1 = x_n;
	y_n_1 = volt_inverse;
}
void lowpass()
{
	velo_lowpass = 0.99529869*velo_lowpass + 0.00235066*kal_velocity + 0.00235066*prevlowpass;
	prevlowpass = kal_velocity;
	pos_lowpass = 0.96906992*pos_lowpass + 0.01546504*current_rad_wrap + 0.01546504*prevposlowpass;
	prevposlowpass = current_rad_wrap;
}
void Diff_velo(){
	velo_diff =  (pos_lowpass - prevPos)/dt;
	//velo_diff2 = (current_rad_wrap -  prevPos2)/dt;
	DegAbs[0][0] = velo_diff;
	prevPos = pos_lowpass;
}
void Kalman_filter()
{
	prediction(X_hat_t,P_t,F_t,Q_t);
	Update(X_hat_t,P_hat_t,DegAbs,R_t,H_t);
	kal_position = X_t[0][0];
	kal_velocity = X_t[1][0];
	kal_acceleration = X_t[2][0];
	equal(X_hat_t,X_t,3,3);  //X_hat_t = X_t
	equal(P_hat_t,P_t,3,3); //P_hat_t = P_t
}
void prediction(X_hat_t_1, P_t_1, F_t, Q_t)
{
	Multiply_matrix(F_t,X_hat_t_1,multiply, 3,3,3,3);
	equal(X_hat_t,multiply,3,3);

	Multiply_matrix(F_t,P_t_1,multiply, 3,3,3,3);
	equal(collect3_3,multiply,3,3);
	Multiply_matrix(collect3_3,F_t_tran,multiply, 3,3,3,3);
	PlusOrMinus(multiply,Q_t,3,3,1);
	equal(P_hat_t,pm3_3,3,3);
}

void Update(X_hat_t, P_hat_t, DegAbs, R_t, H_t)
{
	Multiply_matrix(H_t,X_hat_t,multiply,3,3,3,3);
	PlusOrMinus(DegAbs,multiply,3,3,2);
	equal(y_k,pm3_3,3,3);

	Multiply_matrix(H_t,P_hat_t,multiply,3,3,3,3);
	equal(collect3_3,multiply,3,3);
	Multiply_matrix(collect3_3,H_t_tran,multiply, 3,3,3,3);
	PlusOrMinus(multiply,R_t,3,3,1);
	equal(S_k,pm3_3,3,3);

	S_k[0][0] = 1/S_k[0][0];
	Multiply_matrix(P_hat_t,H_t_tran,multiply,3,3,3,3);
	equal(collect3_3,multiply,3,3);
	Multiply_matrix(collect3_3,S_k,multiply, 3,3,3,3);
	equal(K_k,multiply,3,3);

	Multiply_matrix(K_k,y_k,multiply,3,3,3,3);
	PlusOrMinus(X_hat_t,multiply,3,3,1);
	equal(X_t,pm3_3,3,3);

	Multiply_matrix(K_k,H_t,multiply,3,3,3,3);
	PlusOrMinus(I,multiply,3,3,2);
	Multiply_matrix(pm3_3,P_hat_t,multiply,3,3,3,3);
	equal(P_t,multiply,3,3);
}

void Multiply_matrix(float A[][3], float B[][3],float multiply[][3],int row1,int col1,int row2,int col2)
{
	uint8_t i,j,k;
	//set zero
	for(i=0; i<3 ;i++){
		for(j=0; j<3;j++){
			multiply[i][j] = 0;
		}
	}

	for(i=0;i<row1;i++){
		for(j=0;j<col2;j++){
			for(k=0;k<col1;k++){
				multiply[i][j] += A[i][k]*B[k][j];
			}
		}
	}
}

void PlusOrMinus(float A[][3], float B[][3],int row,int col,int con)
{
	uint8_t i,j;
	//set zeros
	for(i=0; i<3 ;i++){
			for(j=0; j<3;j++){
				pm3_3[i][j] = 0;
			}
		}
	//condition for plus
	if(con == 1){
		for(i = 0; i < row; ++i){
			for (j = 0; j < col; ++j){
				pm3_3[i][j] = A[i][j] + B[i][j];
			}
		}
	}
	//condition for minus
	else if(con == 2){
		for(i = 0; i < row; ++i){
			for (j = 0; j < col; ++j){
				pm3_3[i][j] = A[i][j] - B[i][j];
			}
		}
	}
}

void equal(float A[][3],float B[][3],int row,int col)
{
	uint8_t i,j;
	for(i = 0; i < row; ++i){
		for (j = 0; j < col; ++j){
			A[i][j] = B[i][j];
		}
	}
}


void Trajectory(float qf,float vb)
{
	vb = vb/9.5493;
	float ab = 0.3;
	if(qf < 0.1){
		ab = 0.3;
		vb = 1.5/9.5493;
	}
	if(tuaall >= 0){
		tua = tuaall - 0;
		if(qf/vb > vb/ab){
			tb = vb/ab;
			vb_n = vb;

		}
		else{
			tb = sqrt(qf/ab);
			vb_n = sqrt(qf*ab);
		}
		tf = (2*tb) + ((qf-(vb_n*vb_n/ab))/vb_n);
		if(tua <= tb){
			tra_pos = 0.5*ab*tua*tua;
			tra_velo = ab*tua;
			tra_acc = ab;
			/*if(tra_velo < velostart){
				tra_velo = velostart;
			}*/
			start_tra = 1;
		}
		else if(tua < (tf-tb)){
			tra_pos = (0.5*ab*tb*tb) + (vb_n*(tua-tb));
			tra_velo = vb_n;
			tra_acc = 0;
			start_tra = 0;

		}
		else if(tua >= tf-tb && tua <= tf){
			tra_pos = (0.5*ab*tb*tb) + (vb_n*(tua-(tf-tb))) + (vb_n*(tf-(2*tb))) - (0.5*ab*(tua-(tf-tb))*(tua-(tf-tb)));
			tra_velo = vb_n - (ab*(tua-(tf-tb)));
			tra_acc = -ab;
			start_tra = 2;
		}
		if(tua <= tf){
			if(direct == 0){
				tra_pos = tra_pos*-1;
				tra_velo = tra_velo *-1;
				tra_acc = tra_acc *-1;
				tra_pos += rad_before;
			}
			else{
				tra_pos += rad_before;
			}
		}
	}
	else{
		tra_pos = 0;
		tra_velo = 0;
		tra_acc = 0;
	}


}
void Drivemotor(int PWM){
	if(PWM<=0 && PWM>=-10000){
		PWM = 10000-aaabs(PWM);
		htim2.Instance->CCR1=aaabs(PWM);
		htim2.Instance->CCR2=10000;
		}
	else if (PWM<-10000){
		htim2.Instance->CCR1=0;
		htim2.Instance->CCR2=10000;
	}else if(PWM>0 && PWM<=10000){
		PWM = 10000-PWM;
		htim2.Instance->CCR1=10000;
		htim2.Instance->CCR2=aaabs(PWM);
	}else if(PWM>10000){
		htim2.Instance->CCR1=10000;
		htim2.Instance->CCR2=0;
	}
}

void NFcontacton(){
	HAL_I2C_Master_Transmit(&hi2c1, NFW_ADDR, &NFREG_ON, 1, 10);
	microsecDelay(60);
	//NFwriteFlag = 0;
}

void NFgetstate(){
	if( (hi2c1.State == HAL_I2C_STATE_READY && (HAL_GetTick()-NFtimest>500))){
		NFtimest = HAL_GetTick();
		HAL_I2C_Master_Transmit(&hi2c1, NFW_ADDR, &NFREG_INIT, 1, 10);
		microsecDelay(100);
		HAL_I2C_Master_Receive(&hi2c1, NFR_ADDR, &NFstate, 1, 10);
		microsecDelay(100);
		//NFreadFlag = 0;
	}
}

void AMT222getpos(uint16_t *data){

	switch(AMTstate){
	case 0:
		timeStamp = micros();
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	    HAL_SPI_TransmitReceive_DMA(&hspi3, &AMTGETPOS[0], &AMTHbyte, 1);
	    AMTcomplete = 0;
		break;
	case 1:
		//microsecDelay(5);
		AMTstate+=1;
		AMTcomplete = 1;
		break;
	case 2:
		HAL_SPI_TransmitReceive_DMA(&hspi3, &AMTGETPOS[1], &AMTLbyte, 1);
		AMTcomplete = 0;
		break;
	case 3:
		AMTlogic_checkbit = AMT222checkbit( AMTHbyte, AMTLbyte);
	    if( AMTlogic_checkbit){
		    // Shift high bit to correct position and sum them.
		    rawPos = AMTLbyte + ((AMTHbyte & 0b00111111)<<8);
		     // Convert uint16 to angle variable.
		    current_rad = (((rawPos)*2*PI)/(16384));
		    if(setzero == 0){
		    current_rad = offset - current_rad;
		    unwrapping();
			if(current_rad_wrap-prevPos2 > 0.1 || current_rad_wrap-prevPos2 < -0.1){
				current_rad_wrap = prevPos2;
			}
		    prevPos2 = current_rad_wrap;
		    deg = current_rad_wrap*57.2957795;
		    }
			  /*if(current_rad <= 0){
				  current_rad = 6.28-(current_rad*-1);
			  }*/
	    }
		AMTstate = 0;
		AMTcomplete = 0;
		break;
	}
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi == &hspi3){
		AMTstate+=1;
		AMTcomplete = 1;
		if(AMTstate == 1){
			HAL_SPI_DMAStop(&hspi3);
		}
		else if(AMTstate == 3){
			//microsecDelay(5);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
			HAL_SPI_DMAStop(&hspi3);
		}
	}
}

uint8_t AMT222checkbit( uint8_t *H, uint8_t *L){

	uint8_t K1 = 0;
	uint8_t K0 = 0;

	K1 = !( ((*H>>5) & 0x01) ^ ((*H>>3) & 0x01) ^ ((*H>>1) & 0x01) ^
			((*L>>7) & 0x01) ^ ((*L>>5) & 0x01) ^ ((*L>>7) & 0x03) ^
			((*L>>1) & 0x01) );

	K0 = !( ((*H>>4) & 0x01) ^ ((*H>>2) & 0x01) ^ ((*H>>0) & 0x01) ^
			((*L>>6) & 0x01) ^ ((*L>>4) & 0x01) ^ ((*L>>2) & 0x03) ^
			((*L>>0) & 0x01) );

	if( (K1 == ((*H>>7) & 0x01)) && (K0 == ((*H>>6) & 0x01)) ){
		return 1;
	}
	else{
		return 0;
	}
}
void unwrapping(){
	  p_n = current_rad;
		if (p_n - p_n_1 <= -1*e)
		{
			p_0 = p_0_1 + p_max;
		}
		else if(p_n - p_n_1 >= e)
		{
			p_0 = p_0_1 - p_max;
		}
		else
		{
			p_0 = p_0_1;
		}
		//DegAbs[0][0] = (p_n + p_0); //สั�?�?าณที่ผ่าน�?าร unwrap �?ล้ว
		current_rad_wrap = (p_n + p_0);
		//Memory ให้�?ับตัว�?ปร p[n-1] �?ละ p0[n-1]
		p_n_1 = p_n;
		p_0_1 = p_0;
}
void microsecDelay(uint8_t delayTime){
	uint32_t startTime = micros(); 	//reference point to count passed time
	uint32_t passedTime  = 0;
	while (passedTime<delayTime){
		passedTime = micros() - startTime;
	}
}


uint32_t aaabs(int x){

	if(x<0){
		return x*-1;
	}else{
		return x;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13){
		blue = 1; // 0% duty cycle
	}

	if(GPIO_Pin == GPIO_PIN_8){
		static GPIO_PinState B1State[2] = {0};
		B1State[0]= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
		if(B1State[1] == GPIO_PIN_RESET && B1State[0] == GPIO_PIN_SET){
			stop = 1;
		}
		B1State[1] = B1State[0];


	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim11) {
		_micro += 65535;
	}
}

uint64_t micros()
{
	return _micro + htim11.Instance->CNT;
}

//----------------------------------------------------------------< UART >

void callUART(){
	HAL_UART_DMAStop(&huart2);
	switch(RxState){
	case 0:											// Receive 1 byte first by default.
		Start = (Receive[0] & 0xF0)>>4;
		Mode = (Receive[0] & 0x0F);
		if(Start == 0b1001){
			// 0b1001, 0b0100]
			sum = 0;
			sum += (Receive[0]);					// Sum the START & MODE bit.
			if(Mode == 0b0111){						// Mode7
				RxState = 2;
			}
			else{
				RxState = 1;						// Go to Interrupt Receive next round.
			}
			nbyte = frameConfig(Mode);				// Set a new size of buffer.
		}
		break;
	case 1:
		sum += frameSum( Receive, nbyte-1);			// (nbyte-1) Minus checksum out.
		if(Receive[nbyte-1] == checkSum(sum)){		// Corrected.
			// Receive[nbyte-1] == checkSum(sum)
			TxState = 1;
		}
		if(Mode == 1 || Mode == 9 || Mode == 10 || Mode == 11){
			nbyte = 1;								// For receive ACK
			RxState = 3;
		}
		else{
			endReceive = 1;
		}

		break;
	case 2:
		sum += (Receive[0]);
		nStation = Receive[0];					// Set received station.
		nDestination = Receive[0];				// Set received station.
		if(Receive[0] % 2 == 1){
			nbyte = ((Receive[0] + 1)/2)+1;
		}
		else{
			nbyte = (Receive[0]/2)+1;
		}
		RxState = 1;
		break;
	case 3:						// For Receive ACK frame
		if(Receive[0] == ACK1[0]){
			keepACK[0] = Receive[0];
		}
		else if(Receive[0] == ACK1[1]){
			keepACK[1] = Receive[0];
		}
		else{
			Start = (Receive[0] & 0xF0)>>4;
			Mode = (Receive[0] & 0x0F);
			if(Start == 0b1001){
				// 0b1001, 0b0100
				sum = 0;
				sum += (Receive[0]);				// Sum the START & MODE bit.
				if(Mode == 0b0111){					// Mode7
					RxState = 2;
				}
				else{
					RxState = 1;					// Go to Interrupt Receive next round.
				}
				nbyte = frameConfig(Mode);			// Set a new size of buffer.
			}
			TxState = 0;
			RxState = 1;
		}
		if(keepACK[0] == ACK1[0] && keepACK[1] == ACK1[1]){			// It's ACK
			keepACK[0] = 0;
			keepACK[1] = 0;
			ACKFlag = 1;
			RxState = 0;
			endReceive = 1;
		}
		break;
	}
	// if complete or fail -> reset communication rxstate=0, sum=0, mode=0, nbyte=1,
	if(endReceive){
		RxState = 0;
		nbyte = 1;
		sum = 0;
		endReceive = 0;
	}

	//RxComplete = 0;
}

void responseUART(){

	switch(Mode){

	case 0:
		break;

	case 1:	// <<<< 95% COMPLETE >>>>
		switch(TxState){
		case 0:
			break;
		case 1:
			HAL_UART_Transmit_DMA(&huart2, ACK1, 2);
			TxState = 2;
			break;
		case 2:
			Transmit[0] = 0b10010001;								// Store byte to Transmit.
			Transmit[1] = Receive[0];
			Transmit[2] = Receive[1];
			Transmit[3] = checkSum(frameSum(Transmit, 3));			// Let the last byte equal to checksum of 1st-3rd byte.
			RxState = 3;
			TxState = 3;
			break;
		case 3:
			if(huart2.gState == HAL_UART_STATE_READY){
				HAL_UART_Transmit_DMA(&huart2, Transmit, 4);		// Transmit command back.
				TxState = 4;
			}
			break;
		case 4:
			if(ACKFlag){
				ACKFlag = 0;
				TxState = 0;
			}
			break;
		}
		break;

	case 2:	// <<<< COMPLETE >>>>
		// Connect
		switch(TxState){
		case 0:
			break;
		case 1:
			HAL_UART_Transmit_DMA(&huart2, ACK1, 2);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
			TxState = 0;
			break;
		}
		break;

	case 3:	// <<<< COMPLETE >>>>
		// Disconnect
		switch(TxState){
		case 0:
			break;
		case 1:
			HAL_UART_Transmit_DMA(&huart2, ACK1, 2);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
			TxState = 0;
			break;
		}
		break;

	case 4:	// <<<< COMPLETE >>>>
		// Set limit of omega
		switch(TxState){
		case 0:
			break;
		case 1:
			limitOmega = (Receive[1]*10.0)/(255.0);
			HAL_UART_Transmit_DMA(&huart2, ACK1, 2);
			TxState = 0;
			break;
		}
		break;

	case 5: // <<<< 95% COMPLETE >>>>
		// Set new position()
		switch(TxState){
		case 0:
			break;
		case 1:
			nDestination = 1;
			Destination[0] = ( ( (Receive[0]*256)+(Receive[1]) )*(180) )/(10000*3.14);
			HAL_UART_Transmit_DMA(&huart2, ACK1, 2);
			TxState = 0;
			//timeStamp = HAL_GetTick();								// Optional
			break;
		}
		break;

	case 6:	// <<<< 95% COMPLETE >>>>
		// Set 1 station
		switch(TxState){
		case 0:
			break;
		case 1:
			nStation = 1;
			nDestination = 1;
			Destination[0] = tranStation(Receive[1]);
			HAL_UART_Transmit_DMA(&huart2, ACK1, 2);
			TxState = 0;
			//timeStamp = HAL_GetTick();								// Optional
			break;
		}
		break;

	case 7:
		// set n station
		switch(TxState){
		case 0:
			break;
		case 1:
			nDestination = nStation;
			for(int i=0; i<(nStation+(nStation%2)); i+=2){
				Destination[i] = tranStation(Receive[j] & 0x0F);
				Destination[i+1] = tranStation((Receive[j] & 0xF0)>>4);
				j+=1;
			}
			HAL_UART_Transmit_DMA(&huart2, ACK1, 2);
			j = 0;
			TxState = 0;
			break;
		}
		break;

	case 8:	// <<<< 95% COMPLETE >>>>
		// Go!
		switch(TxState){
		case 0:
			break;
		case 1:
			HAL_UART_Transmit_DMA(&huart2, ACK1, 2);
			movingFlag = 8;
			TxState = 0;
			//timeStamp = HAL_GetTick();								// Optional
			break;
		}
		break;

	case 9:	// <<<< 95% COMPLETE >>>>
		// transmit station
		switch(TxState){
		case 0:
			break;
		case 1:
			HAL_UART_Transmit_DMA(&huart2, ACK1, 2);
			TxState = 2;
			break;
		case 2:
			Transmit[0] = 0b10011001;								// Store byte to Transmit.
			Transmit[1] = 0;
			Transmit[2] = nowStation;								// Value from state machine.
			Transmit[3] = checkSum(frameSum(Transmit, 3));			// Let the last byte equal to checksum of 1st-3rd byte.
			RxState = 3;
			TxState = 3;
			break;
		case 3:
			if(huart2.gState == HAL_UART_STATE_READY){
				HAL_UART_Transmit_DMA(&huart2, Transmit, 4);		// Transmit command back.
				TxState = 4;
			}
			break;
		case 4:
			if(ACKFlag){
				ACKFlag = 0;
				TxState = 0;
			}
			break;
		}
		break;

	case 10:	// <<<< 95% COMPLETE >>>>
		// transmit theta (rad variable)
		switch(TxState){
		case 0:
			break;
		case 1:
			HAL_UART_Transmit_DMA(&huart2, ACK1, 2);
			TxState = 2;
			break;
		case 2:
			Transmit[0] = 0b10011010;								// Store byte to Transmit.
			Transmit[1] = ( (uint16_t)(current_rad_wrap*10000)) >> 8;									// 16bit value from encoder.
			Transmit[2] = ( (uint16_t)(current_rad_wrap*10000)) & 0xFF;
			if(HAL_GetTick()-HMEtimeStamp < 1000){
				Transmit[1] = 0;
				Transmit[2] = 0;
			}
			Transmit[3] = checkSum(frameSum(Transmit, 3));			// Let the last byte equal to checksum of 1st-3rd byte.
			RxState = 3;
			TxState = 3;
			break;
		case 3:
			if(huart2.gState == HAL_UART_STATE_READY){
				HAL_UART_Transmit_DMA(&huart2, Transmit, 4);		// Transmit command back.
				TxState = 4;
			}
			break;
		case 4:
			if(ACKFlag){
				ACKFlag = 0;
				TxState = 0;
			}
			break;
		}
		break;

	case 11:	// <<<< 95% COMPLETE >>>>
		// transmit top omega (rad variable)
		switch(TxState){
		case 0:
			break;
		case 1:
			HAL_UART_Transmit_DMA(&huart2, ACK1, 2);
			TxState = 2;
			break;
		case 2:
			Transmit[0] = 0b10011011;								// Store byte to Transmit.
			Transmit[1] = 0;
			Transmit[2] = (topOmega*255)/(10);
			// Transmit[2] = ((topOmega*255)/(10))
			Transmit[3] = checkSum(frameSum(Transmit, 3));			// Let the last byte equal to checksum of 1st-3rd byte.
			RxState = 3;
			TxState = 3;
			break;
		case 3:
			if(huart2.gState == HAL_UART_STATE_READY){
				HAL_UART_Transmit_DMA(&huart2, Transmit, 4);		// Transmit command back.
				TxState = 4;
			}
			break;
		case 4:
			if(ACKFlag){
				ACKFlag = 0;
				TxState = 0;
			}
			break;
		}
		break;

	case 12:
		// Enable end-effector
		switch(TxState){
		case 0:
			if(huart2.gState == HAL_UART_STATE_READY){
				//enable endf
			}
			break;
		case 1:
			NFenable = 1;
			HAL_UART_Transmit_DMA(&huart2, ACK1, 2);
			TxState = 0;
			break;
		}
		break;

	case 13:
		// Disable end-effector
		switch(TxState){
		case 0:
			break;
		case 1:
			NFenable = 0;
			HAL_UART_Transmit_DMA(&huart2, ACK1, 2);
			TxState = 0;
			break;
		}
		break;

	case 14:	// <<<< 95% COMPLETE >>>>
		// Set home
		switch(TxState){
		case 0:
			break;
		case 1:
			//HAL_UART_Transmit_DMA(&huart2, ACK1, 2);
			setzero = 1;
			movingFlag = 14;
			TxState = 0;
			//timeStamp = HAL_GetTick();								// Optional
			break;
		}
		break;
	}
}

uint16_t tranStation(uint8_t num){
	switch(num){
	case 1:
		return 90;
		break;
	case 2:
		return 180;
		break;
	case 3:
		return 270;
		break;
	case 4:
		return 0;
		break;
	case 5:
		return 180;
		break;
	case 6:
		return 210;
		break;
	case 7:
		return 250;
		break;
	case 8:
		return 180;
		break;
	case 9:
		return 90;
		break;
	case 10:
		return 0;
		break;
	}
}

void destinationACK(){
	if(movingFlag != 0 && Reached ){
		//((HAL_GetTick()-timeStamp > 10000) || Reached)
	    switch(movingFlag){
	    case 8:
		  HAL_UART_Transmit_DMA(&huart2, ACK2, 2);
		  break;
	    case 14:
	    	HAL_UART_Transmit_DMA(&huart2, ACK1, 2);
		  break;
	    }
	    movingFlag = 0;
	    Reached = 0;
  }
}

uint8_t frameConfig(uint8_t mode){
	if(mode == 2 || mode == 3 || mode == 8 || mode == 9 || mode == 10 || mode == 11 || mode == 12 || mode == 13 || mode == 14){
		return 1;
	}
	else if(mode == 1 || mode == 4 || mode == 5 ||  mode == 6){
		return 3;
	}
	else if(mode == 7){
		return 1;									// Keep the number of byte first. (nStation)
	}
	return 0;
}

uint8_t checkSum(uint8_t var){
	//cSum = (~(var%256))%256;						// Spectrator variable
	return (~(var%256))%256;
}

uint8_t frameSum(uint8_t *frame, uint8_t num){
	uint8_t var = 0;
	for( int i=0; i<num; i+=1){
		var += frame[i];							// Sum all the bits in buffer.
	}
	return var;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//RxComplete = 1;
	callUART();
	HAL_UART_Receive_DMA(&huart2, Receive, nbyte);
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
