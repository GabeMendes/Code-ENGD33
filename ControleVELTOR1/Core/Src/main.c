/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "pwm.h"
#include "qdc.h"
#include "task.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//motor 1
#define Kp_w1 1
#define Ki_w1 1
#define Kd_w1 1
#define Kp_i1 1
#define Ki_i1 1
#define Kd_i1 1
//motor 2
#define Kp_w2 1
#define Ki_w2 1
#define Kd_w2 1
#define Kp_i2 1
#define Ki_i2 1
#define Kd_i2 1
//motor 3
#define Kp_w3 1
#define Ki_w3 1
#define Kd_w3 1
#define Kp_i3 1
#define Ki_i3 1
#define Kd_i3 1
//outros
#define pi 3.14159
#define kt 18.803
#define PPR 1440
#define dt 0.01
#define dt2 0.0005
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void task_EncoderCrtlSpeed( void *pvParameters );
float ctrl_PID (float kp, float ki, float kd, float T, float input, float setpoint);
QueueHandle_t xQueue_tqSpeed1;
QueueHandle_t xQueue_tqSpeed2;
QueueHandle_t xQueue_tqSpeed3;
QueueHandle_t xQueue_setpointVel1; // filas para receber as velocidades definidas pelo usuário
QueueHandle_t xQueue_setpointVel2;
QueueHandle_t xQueue_setpointVel3;
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
  HAL_StatusTypeDef ADC_status1, ADC_status2, ADC_status3;
  PWM_Parameters_t Parameters1, Parameters2, Parameters3;
  QDC_Parameters_t QDC_Parameters1, QDC_Parameters2, QDC_Parameters3;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  xQueue_tqSpeed1 = xQueueCreate( 1, sizeof( float ) );
  xQueue_tqSpeed2 = xQueueCreate( 1, sizeof( float ) );
  xQueue_tqSpeed3 = xQueueCreate( 1, sizeof( float ) );
  xQueue_setpointVel1 = xQueueCreate( 1, sizeof( float ) );
  xQueue_setpointVel2 = xQueueCreate( 1, sizeof( float ) );
  xQueue_setpointVel3 = xQueueCreate( 1, sizeof( float ) );

  xTaskCreate(
  		 task_EncoderCrtlSpeed                     /* Funcao a qual esta implementado o que a tarefa deve fazer */
        ,  "EncoderCrtlSpeed"   /* Nome (para fins de debug, se necessário) */
        ,  256                          /* Tamanho da stack (em words) reservada para essa tarefa */
        ,  NULL                         /* Parametros passados (nesse caso, não há) */
        ,  1                            /* Prioridade */
        ,  NULL );                      /* Handle da tarefa, opcional (nesse caso, não há) */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  ADC_status1 = HAL_ADC_Start_IT(&hadc1); //Inicia o ADC1 com interrupção
  if (ADC_status1 != HAL_OK) //Verifica se ta tudo bem
  {
	  Error_Handler();
  }

  ADC_status2 = HAL_ADC_Start_IT(&hadc2); //Inicia o ADC2 com interrupção
  if (ADC_status2 != HAL_OK) //Verifica se ta tudo bem
  {
  	  Error_Handler();
  }

  ADC_status3 = HAL_ADC_Start_IT(&hadc3); //Inicia o ADC3 com interrupção
  if (ADC_status3 != HAL_OK) //Verifica se ta tudo bem
  {
      Error_Handler();
  }

  // motor 1
  Parameters1.Channel = PWM12_CH1_PB14;
  Parameters1.Duty = 50; // 50% pra iniciar
  Parameters1.Frequency = 15000.0;
  Parameters1.Mode = PWM_EDGE_ALIGNED;
  Parameters1.Preference = PWM_RESOLUTION;
  Parameters1.Reslution = PWM_8BITS;
  PWM_Init(0, Parameters1); //passa a estrutura com as configurações

  QDC_Parameters1.Port = QDC4_CH1_PD12_CH2_PD13;
  QDC_Parameters1.PullConfig = QDC_PULL_UP;
  QDC_Parameters1.Resolution = QDC_X1;
  QDC_Init(0, QDC_Parameters1); //Inicia Encoder

  //motor 2
  Parameters2.Channel = PWM12_CH2_PB15;
  Parameters2.Duty = 50; // 50% pra iniciar
  Parameters2.Frequency = 15000.0;
  Parameters2.Mode = PWM_EDGE_ALIGNED;
  Parameters2.Preference = PWM_RESOLUTION;
  Parameters2.Reslution = PWM_8BITS;
  PWM_Init(1, Parameters2); //passa a estrutura com as configurações

  QDC_Parameters2.Port = QDC3_CH1_PB4_CH2_PB5;
  QDC_Parameters2.PullConfig = QDC_PULL_UP;
  QDC_Parameters2.Resolution = QDC_X1;
  QDC_Init(1, QDC_Parameters2); //Inicia Encoder

  //motor 3
  Parameters3.Channel = PWM10_CH1_PB8;
  Parameters3.Duty = 50; // 50% pra iniciar
  Parameters3.Frequency = 15000.0;
  Parameters3.Mode = PWM_EDGE_ALIGNED;
  Parameters3.Preference = PWM_RESOLUTION;
  Parameters3.Reslution = PWM_8BITS;
  PWM_Init(2, Parameters3); //passa a estrutura com as configurações

  QDC_Parameters3.Port = QDC8_CH1_PC6_CH2_PC7;
  QDC_Parameters3.PullConfig = QDC_PULL_UP;
  QDC_Parameters3.Resolution = QDC_X1;
  QDC_Init(2, QDC_Parameters3); //Inicia Encoder

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  vTaskStartScheduler();
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  NVIC_SetPriority(ADC_IRQn,
      		  NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */
  NVIC_SetPriority(ADC_IRQn,
    		  NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */
  NVIC_SetPriority(ADC_IRQn,
    		  NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
//motor 1
void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef* hadc ) //entra nessa interrupção a cada T = 1,43us
{
	uint32_t leitura_ADC;
	float tq = 0, tqSpeed = 0, u1 = 0, dutc = 0;
	// motor 1
	if(hadc->Instance == ADC1)
	{
		leitura_ADC = HAL_ADC_GetValue(hadc); //leitura da corrente
		tq = kt*leitura_ADC; //coverte para torque T = K*I
		xQueueReceive(xQueue_tqSpeed1, (void *)&tqSpeed, pdMS_TO_TICKS(0.1));
		u1 = ctrl_PID (Kp_i1, Ki_i1, Kd_i1, dt2, tq, tqSpeed);
		HAL_ADC_Start_IT(hadc);
		//xQueueOverwrite(xQueue_Current, (void *)&leitura_ADC1);
		dutc = (100/12)*u1;
		PWM_SetDuty(0, dutc); // define a velocidade do motor de 0 a 100%
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	// motor 2
	if(hadc->Instance == ADC2)
	{
		leitura_ADC = HAL_ADC_GetValue(hadc); //leitura da corrente
		tq = kt*leitura_ADC; //coverte para torque T = K*I
		xQueueReceive(xQueue_tqSpeed2, (void *)&tqSpeed, pdMS_TO_TICKS(0.1));
		u1 = ctrl_PID (Kp_i2, Ki_i2, Kd_i2, dt2, tq, tqSpeed);
		HAL_ADC_Start_IT(hadc);
		//xQueueOverwrite(xQueue_Current, (void *)&leitura_ADC1);
		dutc = (100/12)*u1;
		PWM_SetDuty(1, dutc); // define a velocidade do motor de 0 a 100%
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	// motor 3
	if(hadc->Instance == ADC3)
	{
		leitura_ADC = HAL_ADC_GetValue(hadc); //leitura da corrente
		tq = kt*leitura_ADC; //coverte para torque T = K*I
		xQueueReceive(xQueue_tqSpeed3, (void *)&tqSpeed, pdMS_TO_TICKS(0.1));
		u1 = ctrl_PID (Kp_i3, Ki_i3, Kd_i3, dt2, tq, tqSpeed);
		HAL_ADC_Start_IT(hadc);
		//xQueueOverwrite(xQueue_Current, (void *)&leitura_ADC1);
		dutc = (100/12)*u1;
		PWM_SetDuty(2, dutc); // define a velocidade do motor de 0 a 100%
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}


void task_EncoderCrtlSpeed( void *pvParameters )
{
	int32_t QDC_pulses1, QDC_pulses2, QDC_pulses3;
	float velAng = 0, u1 = 0, u2 = 0, u3 = 0, setVel1 = 0, setVel2 = 0, setVel3 = 0;
	// motor 1
	xQueueReceive(xQueue_setpointVel1, (void *)&setVel1, pdMS_TO_TICKS(1));
	QDC_Read(0, &QDC_pulses1);
	velAng = ((QDC_pulses1*(2*pi))/PPR)/dt;
	u1 = ctrl_PID (Kp_w1, Ki_w1, Kp_w1, dt, velAng, setVel1);
	xQueueOverwrite(xQueue_tqSpeed1, (void *)&u1);

	// motor 2
	xQueueReceive(xQueue_setpointVel2, (void *)&setVel2, pdMS_TO_TICKS(1));
	QDC_Read(1, &QDC_pulses2);
	velAng = ((QDC_pulses2*(2*pi))/PPR)/dt;
	u2 = ctrl_PID (Kp_w2, Ki_w2, Kp_w2, dt, velAng, setVel2);
	xQueueOverwrite(xQueue_tqSpeed2, (void *)&u2);

	// motor 3
	xQueueReceive(xQueue_setpointVel3, (void *)&setVel3, pdMS_TO_TICKS(1));
	QDC_Read(2, &QDC_pulses3);
	velAng = ((QDC_pulses3*(2*pi))/PPR)/dt;
	u3 = ctrl_PID (Kp_w3, Ki_w3, Kp_w3, dt, velAng, setVel3);
	xQueueOverwrite(xQueue_tqSpeed3, (void *)&u3);

	vTaskDelay(pdMS_TO_TICKS(10));
}


float ctrl_PID ( float kp, float ki, float kd, float T, float input, float setpoint )
{
	float erro = 0, erro_1 = 0, erro_2 = 0, u = 0, u1 = 0;
	erro = setpoint - input;
	u = u1 + kp*(erro - erro_1) + ki*dt*erro + (kd/T)*(erro - 2*erro_1 - erro_2);
	u1 = u;
	erro_2 = erro_1;
	erro_1 = erro;
	return u;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
