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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "math.h"
#include "queue.h"
#include "limits.h"
#include "string.h"
#include "stdbool.h"
#include "MMC5983MA.h"
#include "ism330dhcx.h"
#include "custom_bus.h"
#include "semphr.h"
#include <VL53L0X.h>
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define VL53L0X_I2C_ADDR    (0x29 << 1) // 0x52 in 8-bit format

#define SENSOR_1_ADDR (0x30 << 1) // 0x60
#define SENSOR_2_ADDR (0x31 << 1) // 0x62
#define SENSOR_3_ADDR (0x32 << 1) // 0x64

/* Registers */
#define REG_I2C_SLAVE_ADDR      0x8A
#define REG_SYSRANGE_START      0x00
#define REG_RESULT_RANGE_RAW    0x14
#define VL53L0X_REG_RESULT_RANGE_STATUS 0x14

#define MAX_PWM     4799.0f
#define MIN_PWM     0.0f
#define BASE_SPEED  0.1f  // Default forward PWM speed (0 - 4799)

#define DEG_TO_RAD(deg) ((deg) * (M_PI / 180.0f))
#define CHASSIS_LENGTH 		  0.10f
#define WHEEL_RADIUS_M 		  (WHEEL_DIAMETER_M / 2)
#define WHEEL_DIAMETER_M      0.034f      // 34 mm wheel diameter
#define ENCODER_TICKS_PER_REV 410.0f     // Ticks per wheel revolution
#define METERS_TO_TICKS(v_ms) \
    ((int16_t)(((v_ms) / (M_PI * WHEEL_DIAMETER_M)) * ENCODER_TICKS_PER_REV))

typedef void (*pFunction)(void);

typedef struct {
    float Kp;           // Proportional gain
    float Ki;           // Integral gain
    float Kd;           // Derivative gain

    float integral;     // Accumulated error
    float prev_error;   // Previous error for derivative
    float max_integral; // Anti-windup limit
    float max_output;   // Max steering adjustment limit
} PID_Controller_t;


static PID_Controller_t speed_pid_A = {
    .Kp = 4.0f,
    .Ki = 2.25f,
    .Kd = 0.01f,

    .integral = 0.0f,
    .prev_error = 0.0f,
    .max_integral = 1000.0f,
    .max_output = 4799.0f
};

static PID_Controller_t speed_pid_B = {
    .Kp = 3.5f,
    .Ki = 2.25f,
    .Kd = 0.01f,

    .integral = 0.0f,
    .prev_error = 0.0f,
    .max_integral = 1000.0f,
    .max_output = 4799.0f
};

// 2. HARDCODE THE OPTIMAL LQR GAINS (The K Matrix)
// These values come from your Python/MATLAB tuning (Q and R matrices).
// K_y determines how aggressively to fix distance.
// K_theta determines how aggressively to fix angle.
float K_y = 0.008f;
float K_theta = 0.1f;

volatile int16_t latest_cntA = 0;
volatile int16_t latest_cntB = 0;

typedef struct {
    uint8_t addr;
    GPIO_TypeDef* xshut_port;
    uint16_t xshut_pin;
    uint16_t distance_mm;
} VL53L0X_Dev;

typedef struct {
	int x_front;
	int y_right;
	int y_left;
} tof_pack_t;

typedef struct {
	float acc[3];
	float gyro[3];
	float mag[3];
	float heading;
}imu_pack_t;

VL53L0X_Dev_t sensor1 = { .addr = SENSOR_1_ADDR, .xshut_port = TOF1XSHUT_GPIO_Port, .xshut_pin = TOF1XSHUT_Pin, .stop_variable = 0,
	    .distance_mm = 0};
VL53L0X_Dev_t sensor2 = { .addr = SENSOR_2_ADDR, .xshut_port = TOF2XSHUT_GPIO_Port, .xshut_pin = TOF2XSHUT_Pin, .stop_variable = 0,
	    .distance_mm = 0};
VL53L0X_Dev_t sensor3 = { .addr = SENSOR_3_ADDR, .xshut_port = TOF3XSHUT_GPIO_Port, .xshut_pin = TOF3XSHUT_Pin, .stop_variable = 0,
	    .distance_mm = 0};
//Mag configs
MMC5983MA_t myMag;
ISM330DHCX_Object_t ism330dhcx_obj;
SemaphoreHandle_t uartMutex;

const float offset_x = -10.1f;
const float offset_y = 21.00f - 5.50f;
const float offset_z = -3.50f ;

const float soft_iron[3][3] = {
{  1.003f,  0.001f, -0.017f },
{  0.001f,  1.002f,  0.005f },
{ -0.017f,  0.005f,  0.995f }
};

double initial_wrld_heading = -1000.0;
double relative_heading = 0.0;

QueueHandle_t xTofQueue;
QueueHandle_t xUartRxQueue;
QueueHandle_t xImuQueue;
BaseType_t xStatus;
TaskHandle_t motor_task_handler;
TaskHandle_t local_navigation_task_handler;
TaskHandle_t orientation_task_handler;
TaskHandle_t debug_task_handler;

QueueHandle_t xJoystickQueue;
TaskHandle_t joystick_task_handler = NULL;
volatile bool joystick_mode = false;

//SemaphoreHandle_t xSpiBusMutex;
//SemaphoreHandle_t xImuReadySem;

/* GLOBAL STARTUP GATE FLAG */
volatile uint8_t system_started = 0;
volatile bool is_robot_flipped = false;
volatile float user_target_speed_ms = 0.0f;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
#define CoreDebug_DEMCR (*(volatile uint32_t*) 0xE000EDFC)
#define DWT_CTRL        (*(volatile uint32_t*) 0xE0001000)
#define DWT_CYCCNT      (*(volatile uint32_t*) 0xE0001004)

uint16_t distance;
volatile bool use_usb_logging = false;
// UART RX Variables
uint8_t rx_data;
char rx_buffer[32];
uint8_t rx_index = 0;


/* ---- Nav state machine ------------------------------------------ */

typedef enum {
    NAV_FOLLOWING = 0,
    NAV_BLOCKED_TURNING
} nav_state_t;

/* ---- Persistent state (static so it survives across loop iters) - */

static bool        s_has_left_latched  = false;
static bool        s_has_right_latched = false;
static float        s_y_err_filt        = 0.0f;
static float        s_prev_omega        = 0.0f;
static uint8_t       s_front_block_count = 0;
static nav_state_t  s_nav_state         = NAV_FOLLOWING;
static int8_t         s_turn_dir          = 1; /* +1 = turn left, -1 = turn right */


#define TARGET_WALL_DIST_MM        100.0f
#define TARGET_HEADING_RAD         0.0f

/* Wall presence threshold. Should be clearly larger than
 * TARGET_WALL_DIST_MM (100mm) so the robot recognizes a wall with
 * room to react smoothly, but not so large it reacts to walls across
 * open areas / junctions. Start around 1.5-2x target distance. */
#define WALL_THRESHOLD_MM          130.0f
#define WALL_HYSTERESIS_MM         20.0f   /* gap between enter/exit thresholds */

#define FRONT_OBSTACLE_THRESHOLD_MM 100.0f
#define FRONT_BLOCK_DEBOUNCE_COUNT  3      /* consecutive close reads before reacting */

#define Y_ERR_FILTER_ALPHA          0.4f   /* lower = smoother, more lag */

#define MAX_OMEGA                  1.0f    /* rad/s, absolute saturation */
#define MAX_OMEGA_STEP              0.1f   /* rad/s change allowed per 40ms tick */

#define TURN_OMEGA                  2.0f   /* rad/s used while resolving a front block */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
//HAL_StatusTypeDef VL53L0X_WriteReg(uint8_t dev_addr, uint8_t reg, uint8_t value);
//HAL_StatusTypeDef VL53L0X_WriteReg16(uint8_t reg, uint16_t value);
//HAL_StatusTypeDef VL53L0X_WriteMulti(uint8_t reg, uint8_t *pData, uint16_t count);

//HAL_StatusTypeDef VL53L0X_ReadReg(uint8_t reg, uint8_t *pValue);
//HAL_StatusTypeDef VL53L0X_Read16(uint8_t dev_addr, uint8_t reg, uint16_t *pValue);
//HAL_StatusTypeDef VL53L0X_ReadMulti(uint8_t reg, uint8_t *pData, uint16_t count);
//HAL_StatusTypeDef VL53L0X_SetI2CAddress(uint8_t current_addr, uint8_t new_7bit_addr);

void VL53L0X_Init_All(void);

void Move_MotorA(int16_t magn);
void Move_MotorB(int16_t magn);
float Compute_PID(PID_Controller_t *pid, float error, float dt);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void CalcIK(float v, float ang_v, float *ang_R, float *ang_L);
static bool hysteresis_update(bool latched, float value, float threshold, float hyst);
static float rate_limit(float target, float prev, float max_step);
static float wrap_angle_rad(float angle);
static float front_speed_scale(float front_mm);
uint32_t get_us(void);
void DWT_Init(void);


// FreeRTOS METHODS
static void tof_task(void *parameters);
static void main_task(void *parameters);
static void imu_task(void *parameters);

//void vPID_task(void *parameters);
static void xOrientationCheckTask(void *parameters);
void vMotorController(void *parameters);
void vLocalNavigationTask(void *parameters);
void vDebugPrintTask(void *parameters);
static void vJoystickTask(void *parameters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//int _write(int file, char *ptr, int len) {
//    // Use blocking UART transmission with a timeout (e.g., 10ms or HAL_MAX_DELAY)
//    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, 10);
//    return len;
//}
int _write(int file, char *ptr, int len) {
    // 1. Generate the timestamp header string
    char header[32];
    int h_len = snprintf(header, sizeof(header), "[%lu us] ", get_us());

    // 2. Combine header and message into a single buffer to prevent interleaving
    #define MAX_LOG_LEN 256
    char combined[MAX_LOG_LEN];

    // Truncate payload if it exceeds the local buffer size
    int payload_len = (len < (int)(MAX_LOG_LEN - h_len - 1)) ? len : (int)(MAX_LOG_LEN - h_len - 1);

    memcpy(combined, header, h_len);
    memcpy(combined + h_len, ptr, payload_len);
    int total_len = h_len + payload_len;

    if (use_usb_logging) {
        // --- USB CDC Logging ---
        uint8_t status = USBD_BUSY;
        uint8_t retries = 0;

        do {
            status = CDC_Transmit_FS((uint8_t *)combined, total_len);
            if (status == USBD_BUSY) {
                vTaskDelay(pdMS_TO_TICKS(1)); // Yield to other tasks
                retries++;
            }
        } while (status == USBD_BUSY && retries < 10); // 10ms timeout

    } else {
        // --- UART DMA Logging ---
        while (huart1.gState == HAL_UART_STATE_BUSY_TX) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        HAL_UART_Transmit(&huart1, (uint8_t *)combined, (uint16_t)total_len, 10);
    }

    return len; // Return original length so standard library tracking stays accurate
}

void DWT_Init(void) {
    // 1. Enable TRCENA (Trace Enable) in CoreDebug DEMCR register
    CoreDebug_DEMCR |= (1 << 24);

    // 2. Reset the cycle counter
    DWT_CYCCNT = 0;

    // 3. Enable the CYCCNT (Cycle Counter) bit in DWT Control register
    DWT_CTRL |= (1 << 0);
}

uint32_t get_us(void) {
    return (uint32_t)(DWT_CYCCNT / (SystemCoreClock / 1000000));
}

/* --------------------------------------------------------------------------
 * ISM330DHCX SPI bus wrappers (CS = PA4)
 * -------------------------------------------------------------------------- */
int32_t BSP_SPI1_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Len) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    BSP_SPI1_Send((uint8_t*)&Reg, 1);
    int32_t ret = BSP_SPI1_Send(pData, Len);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    return ret;
}

int32_t BSP_SPI1_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Len) {
    uint8_t reg_addr = (uint8_t)Reg | 0x80;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    BSP_SPI1_Send(&reg_addr, 1);
    int32_t ret = BSP_SPI1_Recv(pData, Len);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    return ret;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  TaskHandle_t tof_task_handler;
  TaskHandle_t main_task_handler;
  TaskHandle_t imu_task_handler;

//  TaskHandle_t vPID_task_handler;
  BaseType_t status;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  DWT_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

    // Enable CYCCNT counter
    DWT_CTRL |= (1 << 0);

    xTofQueue = xQueueCreate(1, sizeof(tof_pack_t));
    xImuQueue = xQueueCreate(1, sizeof(imu_pack_t));
    xUartRxQueue = xQueueCreate(5, 32);
    xJoystickQueue = xQueueCreate(5, 32);
    uartMutex = xSemaphoreCreateMutex();

    status = xTaskCreate(tof_task, "TOFTask", 512, NULL, 2, &tof_task_handler);
    status = xTaskCreate(imu_task, "IMUTask", 512, NULL, 2, &imu_task_handler);
    status = xTaskCreate(main_task, "MainTask", 512, NULL, 2, &main_task_handler);
//    status = xTaskCreate(xOrientationCheckTask, "orietationtask", 128, NULL, 2, &orientation_task_handler);
    status = xTaskCreate(vLocalNavigationTask, "MainTask", 512, NULL, 2, &local_navigation_task_handler);
    status = xTaskCreate(vMotorController, "motorCtrl", 512, NULL, 2, &motor_task_handler);
    status = xTaskCreate(vDebugPrintTask, "debug", 256, NULL, 2, &debug_task_handler);

    printf("Initialised TASKS \r\n");

    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);

    configASSERT(status == pdPASS);

  vTaskStartScheduler();
  /* USER CODE END 2 */

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 6;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 6;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4799;
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 6;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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
  huart1.Init.BaudRate = 460800;
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
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|TOF3XSHUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_Pin|AIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS2_Pin|BIN2_Pin|BIN1_Pin|STBY_Pin
                          |AIN2_Pin|GPIO_PIN_4|TOF1XSHUT_Pin|TOF2XSHUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 TOF3XSHUT_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|TOF3XSHUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin AIN1_Pin */
  GPIO_InitStruct.Pin = CS_Pin|AIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CS2_Pin BIN2_Pin BIN1_Pin STBY_Pin
                           AIN2_Pin PB4 TOF1XSHUT_Pin TOF2XSHUT_Pin */
  GPIO_InitStruct.Pin = CS2_Pin|BIN2_Pin|BIN1_Pin|STBY_Pin
                          |AIN2_Pin|GPIO_PIN_4|TOF1XSHUT_Pin|TOF2XSHUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void VL53L0X_Init_All(void) {
    // 1. Force all into shutdown
    HAL_GPIO_WritePin(sensor1.xshut_port, sensor1.xshut_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(sensor2.xshut_port, sensor2.xshut_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(sensor3.xshut_port, sensor3.xshut_pin, GPIO_PIN_RESET);
    HAL_Delay(30);

    // --- Sensor 1 (Right) ---
    HAL_GPIO_WritePin(sensor1.xshut_port, sensor1.xshut_pin, GPIO_PIN_SET);
    HAL_Delay(10);
    VL53L0X_WrByte(VL53L0X_I2C_ADDR, REG_I2C_SLAVE_ADDR, sensor1.addr >> 1); // Remap address
    VL53L0X_SetBareMetalTimingBudget(sensor1.addr, 20000);// Initialize its object parameters
    VL53L0X_Begin(&sensor1);

    // --- Sensor 2 (Front) ---
    HAL_GPIO_WritePin(sensor2.xshut_port, sensor2.xshut_pin, GPIO_PIN_SET);
    HAL_Delay(10);
    VL53L0X_WrByte(VL53L0X_I2C_ADDR, REG_I2C_SLAVE_ADDR, sensor2.addr >> 1);
    VL53L0X_SetBareMetalTimingBudget(sensor2.addr, 20000);
    VL53L0X_Begin(&sensor2);

    // --- Sensor 3 (Left) ---
    HAL_GPIO_WritePin(sensor3.xshut_port, sensor3.xshut_pin, GPIO_PIN_SET);
    HAL_Delay(10);
    VL53L0X_WrByte(VL53L0X_I2C_ADDR, REG_I2C_SLAVE_ADDR, sensor3.addr >> 1);
    VL53L0X_SetBareMetalTimingBudget(sensor3.addr, 20000);
    VL53L0X_Begin(&sensor3);

    printf("All sensors initialized via object structs!\r\n");
}

// Helper function to auto-recover a locked VL53L0X sensor on the fly via XSHUT
void Reset_Locked_Sensor(VL53L0X_Dev_t *dev) {

//	printf("Resetting sens %d\n\r", dev->addr);
    // 1. Force hardware shutdown using the struct's port and pin
    HAL_GPIO_WritePin(dev->xshut_port, dev->xshut_pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 2. Bring sensor back up
    HAL_GPIO_WritePin(dev->xshut_port, dev->xshut_pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 3. Reassign its I2C slave address
    VL53L0X_WrByte(VL53L0X_I2C_ADDR, REG_I2C_SLAVE_ADDR, dev->addr >> 1);

    // 4. Re-initialize using the proper pointer struct expected by VL53L0X_Begin

    VL53L0X_SetBareMetalTimingBudget(dev->addr, 20000);
    VL53L0X_Begin(dev);
}

static void tof_task(void *parameters) {
    tof_pack_t tof_processed;
    vTaskDelay(pdMS_TO_TICKS(3000));
    VL53L0X_Init_All();

    // Initialize variables as integers
    int filt_left = 100;
    int filt_front = 200;
    int filt_right = 100;

    // Error tracking counters to trigger recovery only if it stays locked/corrupted
    uint8_t err_count_R = 0, err_count_F = 0, err_count_L = 0;

    // Per-sensor read timeout: keep well under the ~60ms task period since
    // all 3 sensors are polled sequentially within one loop iteration.
    const uint32_t READ_TIMEOUT_MS = 15;

    while (1) {
        uint16_t raw_s1 = 0, raw_s2 = 0, raw_s3 = 0;
        HAL_StatusTypeDef st1, st2, st3;

        // --- Read Right Sensor ---
        st1 = VL53L0X_ReadRangePolled(sensor1.addr, &raw_s1, READ_TIMEOUT_MS);
        if (st1 == HAL_OK && raw_s1 <= 500 && raw_s1 != 0) {
            err_count_R = 0;
            filt_right = (int)raw_s1;
        } else {
            filt_right = 20;
            err_count_R++;
            if (st1 == HAL_TIMEOUT) {
                printf("Sen1 timeout (stuck?)\r\n");
            }
        }
        if (err_count_R > 3) {
            printf("Reset Trig sen1\r\n");
            Reset_Locked_Sensor(&sensor1);
            err_count_R = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(5));

        // --- Read Front Sensor ---
        st2 = VL53L0X_ReadRangePolled(sensor2.addr, &raw_s2, READ_TIMEOUT_MS);
        if (st2 == HAL_OK && raw_s2 <= 500 && raw_s2 != 0) {
            err_count_F = 0;
            filt_front = (int)raw_s2;
        } else {
            filt_front = 20;
            err_count_F++;
            if (st2 == HAL_TIMEOUT) {
                printf("Sen2 timeout (stuck?)\r\n");
            }
        }
        if (err_count_F > 3) {
            printf("Reset Trig sen2\r\n");
            Reset_Locked_Sensor(&sensor2);
            err_count_F = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(5));

        // --- Read Left Sensor ---
        st3 = VL53L0X_ReadRangePolled(sensor3.addr, &raw_s3, READ_TIMEOUT_MS);
        if (st3 == HAL_OK && raw_s3 <= 500 && raw_s3 != 0) {
            err_count_L = 0;
            filt_left = (int)raw_s3;
        } else {
            filt_left = 20;
            err_count_L++;
            if (st3 == HAL_TIMEOUT) {
                printf("Sen3 timeout (stuck?)\r\n");
            }
        }
        if (err_count_L > 3) {
            printf("Reset Trig sen3\r\n");
            Reset_Locked_Sensor(&sensor3);
            err_count_L = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(5));

        // Pack values directly into integers (cos(34deg) ≈ 0.829, so we scale it or cast the result to int)
        tof_processed.x_front = filt_front;
        tof_processed.y_right = (int)((float)filt_right * 0.829f); // 0.829 is cos(34°)
        tof_processed.y_left  = (int)((float)filt_left  * 0.829f);

        // Print directly using integers
//        printf("tof_l: %d | tof_f: %d | tof_r: %d\r\n",
//               tof_processed.y_left, tof_processed.x_front, tof_processed.y_right);

        xQueueOverwrite(xTofQueue, &tof_processed);
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

static void imu_task(void *parameters){

    imu_pack_t imu_data;
    ISM330DHCX_Axes_t gyro_data;
    ISM330DHCX_Axes_t acc_data;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // ISM CS
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);  // Mag CS

    ISM330DHCX_IO_t io_ctx;
    io_ctx.BusType  = ISM330DHCX_SPI_4_WIRE;
    io_ctx.Address  = 0;
    io_ctx.Init     = BSP_SPI1_Init;
    io_ctx.DeInit   = BSP_SPI1_DeInit;
    io_ctx.WriteReg = BSP_SPI1_WriteReg;
    io_ctx.ReadReg  = BSP_SPI1_ReadReg;
    io_ctx.GetTick  = BSP_GetTick;

    ISM330DHCX_RegisterBusIO(&ism330dhcx_obj, &io_ctx);

    if (ISM330DHCX_Init(&ism330dhcx_obj) != ISM330DHCX_OK) {
        printf("ISM330DHCX Init Failed!\r\n");
    }

    // Disable I2C on ISM to avoid bus conflicts
    uint8_t ctrl4_val = 0x04;
    BSP_SPI1_WriteReg(0, 0x13, &ctrl4_val, 1);

    ISM330DHCX_ACC_Enable(&ism330dhcx_obj);
    ISM330DHCX_GYRO_Enable(&ism330dhcx_obj);

    printf("Mag Init Started \r\n");
    if (MMC5983_Init(&myMag, &hspi1, CS2_GPIO_Port, CS2_Pin) != MMC5983MA_OK) {
        printf("Magnetometer Init Failed!\r\n");
    }
    MMC5983_SetFilterBandwidth(&myMag, 800);
    MMC5983_PerformResetOperation(&myMag);
    printf("Mag init successful \r\n");

    uint32_t rawX = 0, rawY = 0, rawZ = 0;
    double wrld_heading = 0.0, deviation = 0.0;

    // FreeRTOS pacing variables for 100 Hz (10ms)
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(40);


    while(1){
        // Enforce exact 100Hz execution rate
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // 1. Read IMU (Accel & Gyro)
        if (ISM330DHCX_GYRO_GetAxes(&ism330dhcx_obj, &gyro_data) == ISM330DHCX_OK &&
            ISM330DHCX_ACC_GetAxes(&ism330dhcx_obj, &acc_data) == ISM330DHCX_OK)
        {
            // GYRO (Convert mdps to dps or rad/s if needed)
            imu_data.gyro[0] = gyro_data.x / 1000.0f;
            imu_data.gyro[1] = gyro_data.y / 1000.0f;
            imu_data.gyro[2] = gyro_data.z / 1000.0f;

            // ACCEL (Convert mg to m/s^2)
            imu_data.acc[0] = (acc_data.x / 1000.0f) * 9.80665f;
            imu_data.acc[1] = (acc_data.y / 1000.0f) * 9.80665f;
            imu_data.acc[2] = (acc_data.z / 1000.0f) * 9.80665f;
        }

        // 2. Read Magnetometer
        if (MMC5983_GetMeasurementXYZ(&myMag, &rawX, &rawY, &rawZ) == MMC5983MA_OK)
        {
            // Raw to MicroTesla Conversion - 18bit sensor
            float uT_x = ((float)rawX - 131072.0f) / 131072.0f * 800.0f;
            float uT_y = ((float)rawY - 131072.0f) / 131072.0f * 800.0f;
            float uT_z = ((float)rawZ - 131072.0f) / 131072.0f * 800.0f;

            // Hard Iron Calibration
            float hi_x = uT_x - offset_x;
            float hi_y = uT_y - offset_y;
            float hi_z = uT_z - offset_z;

            // Soft Iron Calibration
            float cal_x = (hi_x * soft_iron[0][0]) + (hi_y * soft_iron[0][1]) + (hi_z * soft_iron[0][2]);
            float cal_y = (hi_x * soft_iron[1][0]) + (hi_y * soft_iron[1][1]) + (hi_z * soft_iron[1][2]);
            float cal_z = (hi_x * soft_iron[2][0]) + (hi_y * soft_iron[2][1]) + (hi_z * soft_iron[2][2]);

            imu_data.mag[0] = cal_x;
            imu_data.mag[1] = cal_y;
            imu_data.mag[2] = cal_z;

            wrld_heading = atan2(cal_y, cal_x);
            wrld_heading = (wrld_heading / M_PI) * 180.0;
            wrld_heading += 180.0;

            if (initial_wrld_heading < -999.0) {
                initial_wrld_heading = wrld_heading;
                printf(">>> Starting Orientation Set: %.1f <<<\r\n", initial_wrld_heading);
            }

            relative_heading = wrld_heading - initial_wrld_heading;
            if (relative_heading < 0.0)     relative_heading += 360.0;
            if (relative_heading >= 360.0)  relative_heading -= 360.0;

            imu_data.heading = (float)relative_heading;

            deviation = relative_heading;
            if (deviation > 180.0) deviation -= 360.0;

//             printf("rawREL: %.1f | DEV: %.1f \r\n", relative_heading, deviation);

        }

        // 3. Broadcast the completed snapshot to the navigation task
        if (xImuQueue != NULL) {
            xQueueOverwrite(xImuQueue, &imu_data);
        }
    }
}

static void vJoystickTask(void *parameters) {
    char buffer[32];
    float speed = 0.0f;
    float turn = 0.0f;

    // Adjustable multiplier to boost turning responsiveness
    float angular_gain = 8.0f;

    while (1) {
        if (xQueueReceive(xJoystickQueue, buffer, portMAX_DELAY) == pdPASS) {
            if (sscanf(buffer, "S:%f,T:%f", &speed, &turn) == 2) {

                // Amplify turn command
                float boosted_turn = turn * angular_gain;

                // Apply Inverse Kinematics for differential drive
                float ang_R, ang_L;
                CalcIK(speed, boosted_turn, &ang_R, &ang_L);

                int16_t target_ticks_A = METERS_TO_TICKS(ang_R);
                int16_t target_ticks_B = METERS_TO_TICKS(ang_L);

                uint32_t packed_value = ((uint32_t)(uint16_t)target_ticks_A << 16) | (uint16_t)target_ticks_B;
                xTaskNotify(motor_task_handler, packed_value, eSetValueWithOverwrite);
                printf("speed: %f | turn: %f | w1: %f| w2: %f \n\r",speed,boosted_turn,ang_L,ang_R);
            }
        }
    }
}

float Compute_PID(PID_Controller_t *pid, float error, float dt)
{
    float p_term = pid->Kp * error;

    pid->integral += error * dt;
    if (pid->integral > pid->max_integral)  pid->integral = pid->max_integral;
    if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
    float i_term = pid->Ki * pid->integral;

    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->Kd * derivative;

    pid->prev_error = error;

    float output = p_term + i_term + d_term;

    if (output > pid->max_output)  output = pid->max_output;
    if (output < -pid->max_output) output = -pid->max_output;

    return output;
}

void compute_lqr_velocity(float y_err, float theta_err, float *v_cmd, float *ang_cmd) {

    // 1. SET BASE VELOCITY
    // The forward speed you want the robot to maintain (e.g., 0.5 m/s
    float v_max = 0.5f;

    // 3. APPLY THE LQR CONTROL LAW: u = -Kx
    // In our case: steering_correction = (K_y * y_err) + (K_theta * theta_err)
    float steering_correction = (K_y * y_err) + (K_theta * theta_err);
//    printf("k_y: %.2f, k_theta: %.2f\r\n", K_y, K_theta);
    // 4. OUTPUT COMMANDS
    float speed_scaling = 1.0f / (1.0f + 2.0f * fabsf(steering_correction));
    float v_ref = v_max * speed_scaling;

        // Optional: Set a minimum floor speed so it doesn't crawl to a complete dead stop
        if (v_ref < 0.1f) {
            v_ref = 0.1f;
        }
    *v_cmd = v_ref;

    // Target angular velocity is 0 (straight) minus the correction
    *ang_cmd = steering_correction;

    // Optional but recommended: Add saturation limits to prevent
//    // the LQR from commanding a turn faster than the motors can physically execute
//    const float MAX_OMEGA = 5.0f; // rad/s
//    if (*ang_cmd > MAX_OMEGA)  *ang_cmd = MAX_OMEGA;
//    if (*ang_cmd < -MAX_OMEGA) *ang_cmd = -MAX_OMEGA;
//
//    printf("steering: %f,v_cmd: %.2f,ang_cmd: %.2f \n\r", steering_correction, *v_cmd, *ang_cmd);
}

void vMotorController(void *parameters){
    uint32_t packed_value = 0;
    int16_t target_ticks_A = 0;
    int16_t target_ticks_B = 0;

    static int32_t prev_cntA = 0;
    static int32_t prev_cntB = 0;

    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    const float dt = 0.010f;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (xTaskNotifyWait(0x00, ULONG_MAX, &packed_value, 0) == pdTRUE)
        {
            target_ticks_A = (int16_t)(packed_value >> 16);
            target_ticks_B = (int16_t)(packed_value & 0xFFFF);
        }

        latest_cntA = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
        latest_cntB = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);

        int16_t deltaA = (int16_t)prev_cntA - latest_cntA;
        int16_t deltaB = (int16_t)prev_cntB - latest_cntB;

        prev_cntA = latest_cntA;
        prev_cntB = latest_cntB;

        float actual_ticks_A = (float)deltaA / dt;
        float actual_ticks_B = (float)deltaB / dt;

        float error_A = (float)target_ticks_A - actual_ticks_A;
        float error_B = (float)target_ticks_B - actual_ticks_B;

        float pwm_A = Compute_PID(&speed_pid_A, error_A, dt);
        float pwm_B = Compute_PID(&speed_pid_B, error_B, dt);

        Move_MotorA((int16_t)pwm_A);
        Move_MotorB((int16_t)pwm_B);

    }
}

// --- GLOBAL SPEED CONTROL VARIABLE ---


//void vMotorController(void *parameters)
//{
//    static int32_t prev_cntA = 0;
//    static int32_t prev_cntB = 0;
//
//    // 1. Wait until startup gate opens
////    while (system_started == 1) {
////        vTaskDelay(pdMS_TO_TICKS(50));
////    }
//
//    // 2. Enable hardware
//    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
//    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
//    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
//    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
//    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
//
//    const float dt = 0.010f; // 10ms loop
//    TickType_t xLastWakeTime = xTaskGetTickCount();
//    const TickType_t xFrequency = pdMS_TO_TICKS(10);
//
//    uint32_t print_counter = 0;
//
//    while (1)
//    {
//        vTaskDelayUntil(&xLastWakeTime, xFrequency);
//
//        // --- Convert your desired m/s speed to target encoder ticks per second ---
//        int16_t target_ticks = (int16_t)((user_target_speed_ms / (M_PI * WHEEL_DIAMETER_M)) * ENCODER_TICKS_PER_REV);
//
//        // Read raw encoder counters
//        latest_cntA = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
//        latest_cntB = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
//
//        // Calculate delta ticks
//        int16_t deltaA = (int16_t)prev_cntA - latest_cntA;
//        int16_t deltaB = (int16_t)prev_cntB - latest_cntB;
//        prev_cntA = latest_cntA;
//        prev_cntB = latest_cntB;
//
//        // Actual ticks per second
//        float actual_ticks_A = (float)deltaA / dt;
//        float actual_ticks_B = (float)deltaB / dt;
//
//        // --- Calculate Actual Measured Speeds (m/s) ---
//        float actual_speed_A_ms = (actual_ticks_A / ENCODER_TICKS_PER_REV) * (M_PI * WHEEL_DIAMETER_M);
//        float actual_speed_B_ms = (actual_ticks_B / ENCODER_TICKS_PER_REV) * (M_PI * WHEEL_DIAMETER_M);
//
//        // Compute PID error using your target ticks
//        float error_A = (float)target_ticks - actual_ticks_A;
//        float error_B = (float)target_ticks - actual_ticks_B;
//
//        float pwm_A = Compute_PID(&speed_pid_A, error_A, dt);
//        float pwm_B = Compute_PID(&speed_pid_B, error_B, dt);
//
//        // Drive motors
//        Move_MotorA((int16_t)pwm_A);
//        Move_MotorB((int16_t)pwm_B);
//
//        // Print target vs actual speed every 100ms
////        if (++print_counter >= 10) {
////            print_counter = 0;
//            printf(">TARGET: %.3f m/s >ACTUAL A: %.3f m/s >ACTUAL B: %.3f m/s >cntA: %lu > cnB: %lu\r\n",
//                   user_target_speed_ms, actual_speed_A_ms, actual_speed_B_ms, (unsigned long)latest_cntA, (unsigned long)latest_cntB);
////        }
//    }
//}

static bool hysteresis_update(bool latched, float value, float threshold, float hyst)
{
    /* Once inside (latched==true), require the value to rise above
     * threshold+hyst to exit. Once outside, require it to fall below
     * threshold-hyst to enter. Prevents chatter right at the boundary. */
    if (latched) {
        return value < (threshold + hyst);
    } else {
        return value < (threshold - hyst);
    }
}

static float rate_limit(float target, float prev, float max_step)
{
    float d = target - prev;
    if (d > max_step)  return prev + max_step;
    if (d < -max_step) return prev - max_step;
    return target;
}

static float wrap_angle_rad(float angle)
{
    while (angle > M_PI)
        angle -= 2.0f * M_PI;

    while (angle < -M_PI)
        angle += 2.0f * M_PI;

    return angle;
}

static float front_speed_scale(float front_mm)
{
    if (front_mm >= 300.0f)
        return 1.0f;

    if (front_mm <= 100.0f)
        return 0.0f;

    return (front_mm - 100.0f) / 200.0f;
}

void vLocalNavigationTask(void *parameters)
{
    (void)parameters;

    tof_pack_t tof_data;
    imu_pack_t imu_data;

    while (system_started == 1) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(40); /* 50Hz nominal (actually ~25Hz at 40ms, matches original) */

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (xQueueReceive(xTofQueue, &tof_data, pdMS_TO_TICKS(50)) != pdPASS ||
            xQueueReceive(xImuQueue, &imu_data, pdMS_TO_TICKS(50)) != pdPASS)
        {
            /* No fresh data this tick - skip rather than act on stale/garbage data. */
            continue;
        }

        float current_yaw_rad = imu_data.heading * (M_PI / 180.0f);
        float y_left_tof  = tof_data.y_left;
        float y_right_tof = tof_data.y_right;
        float x_front_tof = tof_data.x_front;

        /* --- 1. Heading error --- */
        float theta_err = wrap_angle_rad(TARGET_HEADING_RAD - current_yaw_rad);

        /* --- 2. Wall presence with hysteresis --- */
        bool has_left  = hysteresis_update(s_has_left_latched,  y_left_tof,  WALL_THRESHOLD_MM, WALL_HYSTERESIS_MM);
        bool has_right = hysteresis_update(s_has_right_latched, y_right_tof, WALL_THRESHOLD_MM, WALL_HYSTERESIS_MM);
        s_has_left_latched  = has_left;
        s_has_right_latched = has_right;

        /* --- 3. Lateral error, mode-selected --- */
        float y_err = 0.0f;
//        if (has_left && has_right) {
//            /* Both walls: center between them. */
//            y_err = y_right_tof - y_left_tof;
//        } else
//        if (has_left) {
//            y_err = TARGET_WALL_DIST_MM - y_left_tof;
//        } else
        if (has_right) {
            y_err = TARGET_WALL_DIST_MM - y_right_tof;
        }
        /* else: no walls in range, y_err stays 0 (drive straight on heading only) */

        /* Low-pass filter to smooth sensor noise / residual mode-switch jump. */
        s_y_err_filt = Y_ERR_FILTER_ALPHA * y_err + (1.0f - Y_ERR_FILTER_ALPHA) * s_y_err_filt;

        /* --- 4. Front obstacle debounce --- */
        if (x_front_tof < FRONT_OBSTACLE_THRESHOLD_MM) {
            if (s_front_block_count < 255) s_front_block_count++;
        } else {
            s_front_block_count = 0;
        }
        bool front_blocked = (s_front_block_count >= FRONT_BLOCK_DEBOUNCE_COUNT);

        /* --- 5. Nav state machine (Pure wall-following / straight driving without turns) --- */
        float v_cmd, omega_cmd;

        // Always compute regular tracking velocity/steering
        compute_lqr_velocity(s_y_err_filt, theta_err, &v_cmd, &omega_cmd);

        // If front is blocked, force navigation state to tracking mode,
        // but front_speed_scale will naturally zero out the speed anyway.
        s_nav_state = NAV_FOLLOWING;

        float front_scale = front_speed_scale(x_front_tof);
//        v_cmd *= front_scale;

        /* --- 6. Saturate + slew-rate limit omega before it reaches motors --- */
        if (omega_cmd > MAX_OMEGA)  omega_cmd = MAX_OMEGA;
        if (omega_cmd < -MAX_OMEGA) omega_cmd = -MAX_OMEGA;

        omega_cmd = rate_limit(omega_cmd, s_prev_omega, MAX_OMEGA_STEP);
        s_prev_omega = omega_cmd;

        /* --- 7. Inverse kinematics + motor execution --- */
        float ang_R, ang_L;

        CalcIK(v_cmd, omega_cmd, &ang_R, &ang_L);

        int16_t target_ticks_A = METERS_TO_TICKS(ang_R);
        int16_t target_ticks_B = METERS_TO_TICKS(ang_L);

        uint32_t packed_value = ((uint32_t)(uint16_t)target_ticks_A << 16) | (uint16_t)target_ticks_B;
        xTaskNotify(motor_task_handler, packed_value, eSetValueWithOverwrite);

        /* Debug */
         printf("state:%d | l:%.1f f:%.1f r:%.1f | y_err:%.2f(filt:%.2f) th_err:%.2f | v:%.2f w:%.2f | cntA: %lu | cntB: %lu\n\r ",
                s_nav_state, y_left_tof, x_front_tof, y_right_tof, y_err, s_y_err_filt, theta_err, v_cmd, omega_cmd, (unsigned long)latest_cntA, (unsigned long)latest_cntB);
    }
}

//void vLocalNavigationTask(void *parameters)
//{
//    tof_pack_t tof_data;
//    imu_pack_t imu_data;
//
//    int last_has_left;
//    while (system_started == 1) {
//        vTaskDelay(pdMS_TO_TICKS(50));
//    }
//
//    TickType_t xLastWakeTime = xTaskGetTickCount();
//    const TickType_t xFrequency = pdMS_TO_TICKS(40); // 50Hz
//
//    const float TARGET_WALL_DIST_MM = 100.0f;
//    const float TARGET_HEADING_RAD = 0.0f;
//    const float FRONT_OBSTACLE_THRESHOLD_MM = 100.0f; // Stop or slow down if wall is closer than 10cm ahead
//
//    while (1)
//    {
//        vTaskDelayUntil(&xLastWakeTime, xFrequency);
//
//        // Safe 50ms timeout to prevent locking up the loop
//        if (xQueueReceive(xTofQueue, &tof_data, pdMS_TO_TICKS(50)) == pdPASS &&
//            xQueueReceive(xImuQueue, &imu_data, pdMS_TO_TICKS(50)) == pdPASS)
//        {
//            float current_yaw_rad = imu_data.heading * (M_PI / 180.0f);
//            float y_left_tof = tof_data.y_left;
//            float y_right_tof = tof_data.y_right;
//            float x_front_tof = tof_data.x_front; // <--- Your new center sensor data!
//
//            // 1. Heading & Lateral Error Calculation
//            float theta_err = TARGET_HEADING_RAD - current_yaw_rad;
//
//            bool has_left = last_has_left
//                ? (y_left_tof < WALL_THRESHOLD_MM + 20.0f)   // stay locked in
//                : (y_left_tof < WALL_THRESHOLD_MM - 20.0f);  // harder to enter
//            last_has_left = has_left;
//
//
////            y_err = y_right_tof - y_left_tof;
//            float y_err = 0.0f;
//
////            if (has_left && has_right) {
////                y_err = y_right_tof - y_left_tof;
//////                printf("Both walls\n\r");
////            } else
//            if (has_left) {
//                y_err = TARGET_WALL_DIST_MM - y_left_tof;
////                printf("LEft wall\n\r");
//            } else if (has_right) {
//                y_err = y_right_tof - TARGET_WALL_DIST_MM;
////                printf("right Wall\n\r");
//            }
//
////            printf("tof_l: %f | tof_f: %f | tof_r: %f | y_err: %f\n", y_left_tof, x_front_tof, y_right_tof,y_err);
//
//            // 2. Compute normal LQR velocity and steering
//            float v_cmd, omega_cmd;
//            compute_lqr_velocity(y_err, theta_err, &v_cmd, &omega_cmd);
//
//            // 3. FRONT OBSTACLE OVERRIDE (Using your new center sensor)
//            if (x_front_tof < FRONT_OBSTACLE_THRESHOLD_MM) {
//                // If a wall is directly ahead, force forward velocity to zero
//                // (or handle a turn/maze rotation here)
//                v_cmd = 0.0f;
//            }
//
//            // 4. Inverse Kinematics & Motor Execution
//            float ang_R, ang_L;
//            CalcIK(v_cmd, omega_cmd, &ang_R, &ang_L);
//
//            int16_t target_ticks_A = METERS_TO_TICKS(ang_R);
//            int16_t target_ticks_B = METERS_TO_TICKS(ang_L);
//
//            uint32_t packed_value = ((uint32_t)(uint16_t)target_ticks_A << 16) | (uint16_t)target_ticks_B;
//            xTaskNotify(motor_task_handler, packed_value, eSetValueWithOverwrite);
//        }
//    }
//}


static void xOrientationCheckTask(void *parameters)
{
    imu_pack_t imu_data;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // Check at 20Hz

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Peek or receive from xImuQueue without blocking indefinitely
        if (xQueuePeek(xImuQueue, &imu_data, portMAX_DELAY) == pdPASS)
        {
            // When upright, Z-axis acceleration points down against gravity (+9.81 m/s^2).
            // When upside down (flipped), Z-axis acceleration becomes negative (approx -9.81 m/s^2).
            // Threshold set at -5.0 m/s^2 to reliably catch a flip.
            if (imu_data.acc[2] < -6.0f && !is_robot_flipped)
            {
                is_robot_flipped = true;

                // 1. Suspend the navigation task
                if (local_navigation_task_handler != NULL) {
                    vTaskSuspend(local_navigation_task_handler);
                }

                // 2. Hardware cutoff: Kill the motor driver instantly via Standby pin
                HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

                // 3. Print debug statement
                printf(">>> ROBOT FLIPPED! Z-Accel: %.2f m/s^2 (Negative G). Navigation Suspended & Motors Killed. <<<\r\n", imu_data.acc[2]);
            }
            else if (imu_data.acc[2] >= -6.0f && is_robot_flipped)
            {
                is_robot_flipped = false;

                // 1. Resume the navigation task
                if (local_navigation_task_handler != NULL) {
                    vTaskResume(local_navigation_task_handler);
                }

                // 2. Restore hardware standby pin
                HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

                // 3. Print debug statement
                printf(">>> Robot upright. Z-Accel: %.2f m/s^2. Navigation Resumed & Motors Restored. <<<\r\n", imu_data.acc[2]);
            }
        }
    }
}

void CalcIK(float v, float ang_v, float *ang_R, float *ang_L){

	*ang_R = (v + (ang_v * CHASSIS_LENGTH / 2.0)) ;
	*ang_L = (v - (ang_v * CHASSIS_LENGTH / 2.0));
}



static void main_task(void *parameters){
    char local_rx_buffer[32];
    float y, t, val;
    float p, i, d;
    int m = 0;

    while(1) {
        if (xQueueReceive(xUartRxQueue, local_rx_buffer, portMAX_DELAY) == pdPASS) {

            // --- JOYSTICK MODE TOGGLE: "J,1" (ON) or "J,0" (OFF) ---
            if (sscanf(local_rx_buffer, "J,%d", &m) == 1) {
                if (m == 1 && !joystick_mode) {
                    joystick_mode = true;

                    // 1. Suspend autonomous navigation
                    if (local_navigation_task_handler != NULL) {
                        vTaskSuspend(local_navigation_task_handler);

                    }

                    // 2. Create or Resume Joystick Task
                    if (joystick_task_handler == NULL) {
                        xTaskCreate(vJoystickTask, "JoystickTask", 512, NULL, 2, &joystick_task_handler);
                    } else {
                        vTaskResume(joystick_task_handler);

                    }
                    uint32_t zero_pack = 0;
                    xTaskNotify(motor_task_handler, zero_pack, eSetValueWithOverwrite);
                    printf(">>> Joystick Mode ENABLED. Navigation Suspended. <<<\r\n");
                }
                else if (m == 0 && joystick_mode) {
                    joystick_mode = false;

                    // 1. Suspend Joystick Task
                    if (joystick_task_handler != NULL) {
                        vTaskSuspend(joystick_task_handler);

                    }

                    // 2. Resume autonomous navigation
                    if (local_navigation_task_handler != NULL) {
                        vTaskResume(local_navigation_task_handler);
                    }
                    printf(">>> Joystick Mode DISABLED. Navigation Resumed. <<<\r\n");
                }
            }
            // --- ROUTE JOYSTICK PACKETS IF MODE IS ACTIVE ---
            else if (joystick_mode && strncmp(local_rx_buffer, "S:", 2) == 0) {
                xQueueSend(xJoystickQueue, local_rx_buffer, 0);
            }
            // --- EXISTING COMMANDS ---
            else if (strncmp(local_rx_buffer, "usb", 3) == 0) {
                use_usb_logging = true;
                printf("Switched to USB logging\r\n");
            }
            else if (sscanf(local_rx_buffer, "p,%f,%f,%f", &p, &i, &d) == 3) {
                speed_pid_A.Kp = p; speed_pid_A.Ki = i; speed_pid_A.Kd = d;
                printf("PID Motor A updated\r\n");
            }
            else if (sscanf(local_rx_buffer, "q,%f,%f,%f", &p, &i, &d) == 3) {
                speed_pid_B.Kp = p; speed_pid_B.Ki = i; speed_pid_B.Kd = d;
                printf("PID Motor B updated\r\n");
            }
            else if (sscanf(local_rx_buffer, "P,%f,%f,%f", &p, &i, &d) == 3) {
                speed_pid_A.Kp = p; speed_pid_A.Ki = i; speed_pid_A.Kd = d;
                speed_pid_B.Kp = p; speed_pid_B.Ki = i; speed_pid_B.Kd = d;
                printf("PID Both Motors updated\r\n");
            }
            else if (sscanf(local_rx_buffer, "c,%f,%f", &y, &t) == 2) {
                K_y = y; K_theta = t;
                printf("LQR updated\r\n");
            }
            else if (sscanf(local_rx_buffer, "M,%d", &m) == 1) {
                if (m == 1) HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
                else HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
                printf("Motor State: %d\r\n", m);
            }
            else if (sscanf(local_rx_buffer, "s,%f", &val) == 1) {
                user_target_speed_ms = val;
                printf("Target Speed Set: %.3f m/s\r\n", user_target_speed_ms);
            }
            else {
                printf("Invalid format. Use: J,0/1 | P/p/q | c | m | s | usb\r\n");
            }
        }
    }
}

void vDebugPrintTask(void *parameters)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // Print every 50ms (20 Hz)
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Create a local pack to hold queued ToF data safely
    tof_pack_t debug_tof;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Peek at the ToF queue without blocking
//        if (xQueuePeek(xTofQueue, &debug_tof, 0) == pdPASS) {
//            printf("tof_l: %d | tof_f: %d | tof_r: %d | cA: %d | cB: %d\r\n",
//                   debug_tof.y_left, debug_tof.x_front, debug_tof.y_right,
//                   (int)latest_cntA, (int)latest_cntB);
//        } else {
            // Fallback print if queue isn't ready yet, showing at least encoders
//            printf("cA: %d | cB: %d\r\n", (int)latest_cntA, (int)latest_cntB);
//        }
    }
}

void Move_MotorA(int16_t magn) {
    uint32_t pwm_value = 0;
    if (magn > 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET); //GPIO_PIN_RESET
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        pwm_value = (uint32_t)magn;
    } else if (magn < 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET); //GPIO_PIN_SET
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        pwm_value = (uint32_t)(-magn);
    } else {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        pwm_value = 0;
    }
    if (pwm_value > 4799) pwm_value = 4799;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_value);
}

void Move_MotorB(int16_t magn) {
    uint32_t pwm_value = 0;
    if (magn > 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        pwm_value = (uint32_t)magn;
    } else if (magn < 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        pwm_value = (uint32_t)(-magn);
    } else {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        pwm_value = 0;
    }
    if (pwm_value > 4799) pwm_value = 4799;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_value);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Check for end of line/command
        if (rx_data == '\n' || rx_data == '\r') {
            rx_buffer[rx_index] = '\0'; // Null-terminate the string

            if (rx_index > 0) {
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                // Send the received string to the queue
                xQueueSendFromISR(xUartRxQueue, rx_buffer, &xHigherPriorityTaskWoken);
                rx_index = 0;
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // Force context switch if needed
            }
        } else {
            // Append byte to buffer if there's space (avoid overflow)
            if (rx_index < 31) {
                rx_buffer[rx_index++] = rx_data;
            }
        }
        // Re-arm the interrupt for the next byte rapidly
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}

/* USER CODE END 4 */

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
