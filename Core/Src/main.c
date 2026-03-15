/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "ESP8266_STM32.h"
#include "ism330dhcx.h"
#include "custom_bus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define MAG_REQ            1    // Enable Magnetometer
//#define SET_GYRO_CONFIG    1    // 250dps
//#define SET_ACC_CONFIG     2    // 2g
//#define SET_CLK_CONFIG     1    // Auto (PLL)
int16_t accel_data;
uint8_t imu_data[14];
uint32_t rawCounter = 0;
uint32_t counter = 0;
uint32_t lastcounter = 0;
  float RPM_A, RPM_B = 0;


const char *broker = "192.168.1.9";
const uint16_t port = 1883;
char *clientID = "MicroMouse";
char ip_local[16];
extern USBD_HandleTypeDef hUsbDeviceFS; // Use the global USB handle
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ISM330DHCX_Object_t ism330dhcx_obj;
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for Logger */
osThreadId_t LoggerHandle;
const osThreadAttr_t Logger_attributes = {
  .name = "Logger",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ToFRead */
osThreadId_t ToFReadHandle;
const osThreadAttr_t ToFRead_attributes = {
  .name = "ToFRead",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Definitions for LogQueue */
osMessageQueueId_t LogQueueHandle;
const osMessageQueueAttr_t LogQueue_attributes = {
  .name = "LogQueue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void LoggerTask(void *argument);
void MainCTR(void *argument);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//int _write(int file, char *ptr, int len) {
//    // Break the data into smaller chunks if necessary,
//    // but for simple logs, just ensure the previous transfer is finished.
//    uint8_t result = USBD_OK;
//
//    // CDC_Transmit_FS is non-blocking. We must wait until the
//    // hardware is ready or the data will be truncated.
//    while (CDC_Transmit_FS((uint8_t*)ptr, len) == USBD_BUSY) {
//        // Just wait
//    }
//    return len;
//}
int _write(int file, char *ptr, int len) {
    // Only attempt USB transmit if the cable is connected and configured
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
        uint32_t timeout = 0xFFFF;
        while (CDC_Transmit_FS((uint8_t*)ptr, len) == USBD_BUSY && timeout > 0) {
            timeout--;
        }
    }
    // Always fallback to UART so you can still see logs on your Serial adapter
//    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 10);
    return len;
}

typedef struct{
	char topic[64];
	char message[64];
}LoggerpublishQueue_t;

typedef struct {
    float x, y, theta;        // Position in mm and radians
    float wheel_separation;        // Distance between wheels (L)
    float mm_per_tick;        // Conversion factor
    int32_t last_left_ticks;
    int32_t last_right_ticks;
} Odometry_t;

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
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // 1. Setup the IO context to link the driver to your SPI1 hardware
  ISM330DHCX_IO_t io_ctx;
  io_ctx.BusType     = ISM330DHCX_SPI_4_WIRE; // 4-wire SPI as configured in your image
  io_ctx.Address     = 0;                      // Not used for SPI
  io_ctx.Init        = BSP_SPI1_Init;          // These functions are generated by CubeMX
  io_ctx.DeInit      = BSP_SPI1_DeInit;
  io_ctx.WriteReg    = BSP_SPI1_WriteReg;
  io_ctx.ReadReg     = BSP_SPI1_ReadReg;
  io_ctx.GetTick     = BSP_GetTick;

  // 2. Register the component and initialize it
  ISM330DHCX_RegisterBusIO(&ism330dhcx_obj, &io_ctx);
  ISM330DHCX_Init(&ism330dhcx_obj);

  // 3. Enable the Accelerometer and Gyroscope
  ISM330DHCX_ACC_Enable(&ism330dhcx_obj);
  ISM330DHCX_GYRO_Enable(&ism330dhcx_obj);
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

  /* Create the queue(s) */
  /* creation of LogQueue */
  LogQueueHandle = osMessageQueueNew (4, sizeof(uint16_t), &LogQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  if (LogQueueHandle == NULL) {
      printf("ERROR: Failed to create queue!\r\n");
      Error_Handler();
  }
  printf("Queue created: size=%d bytes\r\n", sizeof(LoggerpublishQueue_t));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Logger */
  LoggerHandle = osThreadNew(LoggerTask, NULL, &Logger_attributes);

  /* creation of ToFRead */
  ToFReadHandle = osThreadNew(MainCTR, NULL, &ToFRead_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4799;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, LEDONB_Pin|TOF3XSHUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ACS_Pin|AIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MCS_Pin|BIN2_Pin|BIN1_Pin|STBY_Pin
                          |AIN2_Pin|GPIO_PIN_4|TOF1XSHUT_Pin|TOF2XSHUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LEDONB_Pin TOF3XSHUT_Pin */
  GPIO_InitStruct.Pin = LEDONB_Pin|TOF3XSHUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : FRONT_TOF_INT_Pin LEFT_TOF_INT_Pin */
  GPIO_InitStruct.Pin = FRONT_TOF_INT_Pin|LEFT_TOF_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ACS_Pin AIN1_Pin */
  GPIO_InitStruct.Pin = ACS_Pin|AIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MCS_Pin BIN2_Pin BIN1_Pin STBY_Pin
                           AIN2_Pin PB4 TOF1XSHUT_Pin TOF2XSHUT_Pin */
  GPIO_InitStruct.Pin = MCS_Pin|BIN2_Pin|BIN1_Pin|STBY_Pin
                          |AIN2_Pin|GPIO_PIN_4|TOF1XSHUT_Pin|TOF2XSHUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RIGHT_TOF_INT_Pin */
  GPIO_InitStruct.Pin = RIGHT_TOF_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RIGHT_TOF_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Move_MotorA(int16_t magn) {
    uint32_t pwm_value = 0; // Use an unsigned variable for the timer

    if (magn > 0) {
        // Forward
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);

        // Map 1 to 100 -> 1000 to 4799
        pwm_value = magn;
    }
    else if (magn < 0) {
        // Backward
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);

        // Map -1 to -100 -> 1000 to 4799
        // We use -speed to make the input positive for the math
        pwm_value = (-magn);
    }
    else {
        // Stop
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        pwm_value = 0;
    }

    // Safety Constrain
    if (pwm_value > 4799) pwm_value = 4799;

    // Always apply a positive value to the PWM Compare Register
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_value);
}

void Move_MotorB(int16_t magn) {
    uint32_t pwm_value = 0;

    if (magn > 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        // Map 1 to 100 -> 1000 to 4799
        pwm_value = (magn);
    } else if (magn < 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        // Use the positive magnitude for mapping
        pwm_value = (-magn);
    } else {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        pwm_value = 0;
    }

    if (pwm_value > 4799) pwm_value = 4799;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_value);
}

float CalcRPM(int32_t current_counts, int32_t *previous_counts, uint32_t ppr, float dt) {
    // 1. Calculate the difference in ticks
    // This handles overflow automatically if using signed integers
    int32_t delta_counts = current_counts - *previous_counts;

    // 2. Update previous_counts for the next call
    *previous_counts = current_counts;

    // 3. RPM Math: (Delta / PPR) * (60 / dt)
    // We use 60.0f to ensure floating point math
    float rpm = ((float)delta_counts / (float)ppr) * (60.0f / dt);

    return rpm;
}
/**
 * @brief Generates a smooth S-curve target speed.
 * @param target The final desired RPM.
 * @param shadow Pointer to the linear shadow variable.
 * @param scurve Pointer to the smoothed s-curve variable.
 * @param accel_limit RPM/s limit.
 * @param smoothing 0.1 to 0.3 (lower is smoother).
 * @param dt Loop time in seconds (e.g., 0.1).
 */
void Apply_SCurve_Ramp(float target, float *shadow, float *scurve, float accel_limit, float smoothing, float dt) {
    // 1. Update Linear Shadow
    float step = accel_limit * dt;
    if (*shadow < target) {
        *shadow += step;
        if (*shadow > target) *shadow = target;
    } else if (*shadow > target) {
        *shadow -= step;
        if (*shadow < target) *shadow = target;
    }

    // 2. Generate S-Curve (Low Pass Filter)
    *scurve = (*scurve * (1.0f - smoothing)) + (*shadow * smoothing);
}

float CalcVelocity(int32_t current_counts, int32_t *previous_counts, float mm_per_tick, float dt) {
    int32_t delta_ticks = current_counts - *previous_counts;
    *previous_counts = current_counts;

    // Distance = ticks * mm_per_tick
    // Velocity = Distance / time
    return ((float)delta_ticks * mm_per_tick) / dt;
}

/**
 * @brief Calculates PWM output based on speed error.
 * @param setpoint The S-curve target RPM.
 * @param actual The measured RPM from encoder.
 * @param integral_sum Pointer to the error accumulator (Anti-windup handled inside).
 * @param dt Loop time in seconds.
 * @return int16_t PWM value (0 to 4799).
 */
int16_t PID_Compute(float setpoint, float actual, float *integral_sum, float dt) {
    // ADJUST THESE: Usually Kp should be higher than Ki for motor stability
    float Kp = 2.0f;
    float Ki = 75.0f;

    // 1. Calculate Error
    float error = setpoint - actual;

    // 2. Integral Accumulation with Directional Reset
    // If the setpoint crosses zero or is zero, clear the integral to prevent "spring" effect
    if ((setpoint >= 0 && *integral_sum < 0) || (setpoint <= 0 && *integral_sum > 0) || (abs(setpoint) < 0.1f)) {
        *integral_sum = 0;
    } else {
        *integral_sum += error * dt;
    }

    // 3. Anti-Windup (Clamping the Integral term)
    // This stops the motor from "freaking out" if it gets stuck
    if (*integral_sum > 1899.0f) *integral_sum = 1899.0f;
    if (*integral_sum < -1899.0f) *integral_sum = -1899.0f;

    // 4. Calculate PID output
    float pid_output = (Kp * error) + (Ki * (*integral_sum));

    // 5. Clamp to your Move_Motor input range (-100 to 100)
    if (pid_output > 4799.0f) pid_output = 4799.0f;
    if (pid_output < -4799.0f) pid_output = -4799.0f;

    return (int16_t)pid_output;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_LoggerTask */
/**
  * @brief  Function implementing the Logger thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LoggerTask */
void LoggerTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
//  LoggerpublishQueue_t pub;
  int count = 0;

  printf("[LoggerTask] Starting message loop\r\n");

  for(;;)
  {
	  LoggerpublishQueue_t pub;

	        // Clear the structure first
	  memset(&pub, 0, sizeof(LoggerpublishQueue_t));
	  snprintf(pub.topic, sizeof(pub.topic) - 1, "logger/RPM");
	        pub.topic[sizeof(pub.topic) - 1] = '\0'; // Force null terminator

	        snprintf(pub.message, sizeof(pub.message) - 1, "RPM_A:%.1f,RPM_B:%.1f,Cnt:%d",
	                 RPM_A, RPM_B, count);
	        pub.message[sizeof(pub.message) - 1] = '\0'; // Force null terminator

	        // Debug print before queuing
	        printf("[LoggerTask] Prepared msg #%d\r\n", count);
	        printf("  Topic: '%s'\r\n", pub.topic);
	        printf("  Message: '%s'\r\n", pub.message);

	        if (osMessageQueuePut(LogQueueHandle, &pub, 0, 100) == osOK) {
	            printf("[LoggerTask] ✓ Queued successfully\r\n");
	            count++;
	        } else {
	            printf("[LoggerTask] ✗ Queue PUT failed!\r\n");
	        }

	        osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_MainCTR */
/**
* @brief Function implementing the ToFRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MainCTR */
void MainCTR(void *argument)
{
  /* USER CODE BEGIN MainCTR */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

  int32_t prev_countA = 0;
  int32_t prev_countB = 0;
  float shadowA = 0, scurveA = 0;
  float shadowB = 0, scurveB = 0;
  float integralA = 0, integralB = 0;
  float targetA = 50.0f;
  float targetB = -50.0f;
  ISM330DHCX_Axes_t acc_data;
  ISM330DHCX_Axes_t gyro_data;
  for(;;)
  {

	  RPM_A = CalcRPM(__HAL_TIM_GET_COUNTER(&htim5), &prev_countA, 410, 0.1f);
	  RPM_B = CalcRPM(__HAL_TIM_GET_COUNTER(&htim2), &prev_countB, 410, 0.1f);

	  Apply_SCurve_Ramp(targetA, &shadowA, &scurveA, 50.0f, 0.5f, 0.1f);
	  Apply_SCurve_Ramp(targetB, &shadowB, &scurveB, 50.0f, 0.5f, 0.1f);

	  int16_t pwmA = PID_Compute(scurveA, RPM_A, &integralA, 0.1f);
	  int16_t pwmB = PID_Compute(scurveB, RPM_B, &integralB, 0.1f);

	  Move_MotorA(pwmA);
	  Move_MotorB(pwmB);

	  ISM330DHCX_ACC_GetAxes(&ism330dhcx_obj, &acc_data);
	  ISM330DHCX_GYRO_GetAxes(&ism330dhcx_obj, &gyro_data);

	  printf("ACC: %5ld, %5ld, %5ld | GYR: %5ld, %5ld, %5ld\r\n",
	              acc_data.x, acc_data.y, acc_data.z,
	              gyro_data.x, gyro_data.y, gyro_data.z);
    osDelay(100);
  }
  /* USER CODE END MainCTR */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
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
