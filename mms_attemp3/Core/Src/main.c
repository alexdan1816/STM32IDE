/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <gyro.h>
#include <pid.h>
#include <motor.h>
#include <IR.h>
#include <stdbool.h>
#include <FSM.h>
#include <FLOODFILL.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PID_TypeDef RPID;
PID_TypeDef LPID;
PID_TypeDef TURNPID;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N_PULSES 350

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
Motor Left_motor;
Motor *pLeft = &Left_motor;
Motor Right_motor;
Motor *pRight = &Right_motor;


volatile bool tick_start;
 uint32_t check_count;
//-----MAZE VARIABLE------

Maze _MyMaze;
Maze *toMyMaze = &_MyMaze;

//-----MousePose profile

MousePose _MyMousePose;
MousePose *toMyMousePose = &_MyMousePose;

//-----Cell queue used for floodfilling

Cell_Queue _MyCellQueue;
Cell_Queue *toMyCellQueue = &_MyCellQueue;

//-----Action Stack used for execute action

Action_Stack _MyActionStack;
Action_Stack *toMyActionStack = &_MyActionStack;



//-----LOW LEVEL FSM STATE

/*extern State cur_state = IDLE;*/ //declared in motor.h and motor.c

//-----HIGH LEVEL FSM STATE

/*extern Phase cur_phase = SENSOR_PHR;*/ //declared in FSM.H and FSM.c



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  //Motor Initialization
  Motor_Init(&Right_motor, RIGHT,
		  	 AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin,
			 &htim2, TIM_CHANNEL_2, &htim3, 0.3, 1.9, 0.003);
  Motor_Init(&Left_motor, LEFT,
		  	 BIN1_GPIO_Port, BIN1_Pin, BIN1_GPIO_Port, BIN2_Pin,
			 &htim2, TIM_CHANNEL_1, &htim4, 0.3, 1.9, 0.003);
  Motor_SetTarget(pRight, 0);
  Motor_SetTarget(pLeft, 0);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim4, 0);


  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  PID(&RPID,
	  &(pRight->cur_speed),
	  &(pRight->Pid_output),
	  &(pRight->target_speed),
	  pRight->kp, pRight->ki, pRight->kd,
	  _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&RPID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&RPID, 2);
  PID_SetOutputLimits(&RPID, -499, 499);

  PID(&LPID,
	  &(pLeft->cur_speed),
	  &(pLeft->Pid_output),
	  &(pLeft->target_speed),
	  pLeft->kp, pLeft->ki, pLeft->kd,
	  _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&LPID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&LPID, 2);
  PID_SetOutputLimits(&LPID, -499, 499);

  PID(&TURNPID,&encoder_progress, &encoder_output, &encoder_target, 0.45,0.15,0.1,_PID_P_ON_E,_PID_CD_DIRECT);
  PID_SetMode(&TURNPID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&TURNPID, 1);
  PID_SetOutputLimits(&TURNPID, -90, 90);


  // State initialization
  cur_phase = BEGIN_PHR;
  cur_state = IDLE;
  check_count = 0;


  // Gyro initialization
  LSM6DS3_Init();


  // IR initialization
  HAL_ADCEx_Calibration_Start(&hadc1);
  ir_status = OKAY;

  // Maze initialization
  MazeInitialize(toMyMaze);

  // Mouse Pose init
  PoseInit(toMyMousePose, NORTH, Xstart, Ystart);

  // Action stack init
  Action_Stack_Init(toMyActionStack);

  // Cell queue init
  CellQueueInitialize(toMyCellQueue);


  //khởi tạo maze
  //khởi tạo danh sách cell
  //khởi tạo danh sách hành động
  //khởi tạo pose
  //

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(tick_start)  // tick 1ms
	  {
		  tick_start = false;

//		  cur_phase = SENSOR_PHR;
//		  ReadIR(&hadc1);
//		  HAL_Delay(100);


		  Motor_GetSpeed(&Left_motor);
		  Motor_GetSpeed(&Right_motor);

		  switch (cur_phase) {
		  	case BEGIN_PHR:
		  		if(Check_Start(&hadc1))
		  		{
		  			cur_phase = GYRO_PHR;
		  			LED_OFF();
		  			BUZ_OFF();
		  		}
		  		if (HAL_GetTick()-buz_time > 500 && cur_phase == BEGIN_PHR)
		  		{
		  			BUZ_TOG();
		  			buz_time = HAL_GetTick();
		  		}
		  		if (HAL_GetTick() - led_time > 250 && cur_phase == BEGIN_PHR)
		  		{
		  			LED_TOG();
		  			led_time = HAL_GetTick();
		  		}
		  		break;
		  	case GYRO_PHR:
		  		if(Gyro_Calibrate())
		  		{
		  			cur_phase = SENSOR_PHR;
		  			break;
		  		}
		  		else
		  		{
		  			cur_phase = GYRO_PHR;
		  			break;
		  		}
			case SENSOR_PHR:
				ReadIR(&hadc1);
				break;
			case UPDATE_PHR:
				MazeUpdate(toMyMaze, toMyMousePose);
				break;
			case FINDPATH_PHR:

				if(FindNextCell(toMyMaze, toMyMousePose, toMyActionStack))
				{
					cur_phase = EXECUTE_PHR;
					break;
				}
				else
				{
					cur_phase = ALGORITHM1_PHR;
					break;
				}
			case ALGORITHM1_PHR:
				MazeFloodFill(toMyMaze, toMyCellQueue, toMyMousePose);
				break;
			case ALGORITHM2_PHR:
				MazeFloodFill(toMyMaze, toMyCellQueue, toMyMousePose);
				break;
			case EXECUTE_PHR:
				switch (cur_state) {
					case IDLE:
						ExecuteAct(toMyMousePose, toMyActionStack);
						break;
					case TURN_LEFT:
						Move_Left(pLeft, pRight);
						if(cur_state == TURN_LEFT)
						{
						PID_Compute(&TURNPID);
						}
						if(cur_state == COOL_DOWN)
						{
							prevtime = HAL_GetTick();
							PID_SetMode(&TURNPID, _PID_MODE_MANUAL);
						}
						Motor_SetPwm(&Left_motor);
						Motor_SetPwm(&Right_motor);
						break;
					case TURN_RIGHT:
						Move_Right(pLeft, pRight);
						if(cur_state == TURN_RIGHT)
						{
						PID_Compute(&TURNPID);
						}
						if(cur_state == COOL_DOWN)
						{
							prevtime = HAL_GetTick();
							PID_SetMode(&TURNPID, _PID_MODE_MANUAL);
						}
						Motor_SetPwm(&Left_motor);
						Motor_SetPwm(&Right_motor);
						break;
					case MOVE:
						Move_forward(pLeft, pRight);
						if(cur_state == MOVE)
						{
						PID_Compute(&RPID);
						PID_Compute(&LPID);
						}
						if(cur_state == COOL_DOWN)
						{
							prevtime = HAL_GetTick();
							PID_SetMode(&RPID, _PID_MODE_MANUAL);
							PID_SetMode(&LPID, _PID_MODE_MANUAL);
//							PID_Compute(&RPID);
//							PID_Compute(&LPID);
						}
						Motor_SetPwm(&Left_motor);
						Motor_SetPwm(&Right_motor);
						break;
					case TURN_BACK:
						Move_backward(pLeft, pRight);
						PID_Compute(&TURNPID);
						if(cur_state == COOL_DOWN)
						{
							prevtime = HAL_GetTick();
							PID_SetMode(&TURNPID, _PID_MODE_MANUAL);
						}
						Motor_SetPwm(&Left_motor);
						Motor_SetPwm(&Right_motor);
						break;
					case COOL_DOWN:
						if(HAL_GetTick() - prevtime > 1000)
						{
							PID_SetMode(&TURNPID, _PID_MODE_AUTOMATIC);
							PID_SetMode(&RPID,_PID_MODE_AUTOMATIC);
							PID_SetMode(&LPID,_PID_MODE_AUTOMATIC);
							cur_state = IDLE;
							prevtime = HAL_GetTick();
						}
					default:
						break;
				}
				break;
			default:
				break;
		}


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  TIM_OC_InitTypeDef sOC = {0};
  sOC.OCMode     = TIM_OCMODE_ACTIVE;   // hoặc TOGGLE đều được, vì ta chỉ cần ngắt
  sOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sOC.Pulse      = 0;                   // sẽ set khi bắt đầu quay

  if (HAL_TIM_OC_ConfigChannel(&htim3, &sOC, TIM_CHANNEL_3) != HAL_OK) {
      Error_Handler();
  }

  // Bật NVIC cho TIM3 (nếu CubeMX chưa bật)
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE END TIM3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BIN1_Pin|BIN2_Pin|LED_RIGHT_Pin|LED_FORWARD_Pin
                          |FRIGHT_IR_EMIT_Pin|FLEFT_IR_EMIT_Pin|LEFT_IR_EMIT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_BACK_Pin|LED_LEFT_Pin|AIN2_Pin|AIN1_Pin
                          |RIGHT_IR_EMIT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BIN1_Pin BIN2_Pin LED_RIGHT_Pin LED_FORWARD_Pin
                           FRIGHT_IR_EMIT_Pin FLEFT_IR_EMIT_Pin LEFT_IR_EMIT_Pin */
  GPIO_InitStruct.Pin = BIN1_Pin|BIN2_Pin|LED_RIGHT_Pin|LED_FORWARD_Pin
                          |FRIGHT_IR_EMIT_Pin|FLEFT_IR_EMIT_Pin|LEFT_IR_EMIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_BACK_Pin LED_LEFT_Pin AIN2_Pin AIN1_Pin
                           RIGHT_IR_EMIT_Pin */
  GPIO_InitStruct.Pin = LED_BACK_Pin|LED_LEFT_Pin|AIN2_Pin|AIN1_Pin
                          |RIGHT_IR_EMIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)   // Timer đang chạy 2ms
    {
      tick_start = true;
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
  __disable_irq();
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
