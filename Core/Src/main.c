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
 * @desc This is a program designed to balance a 2-wheeled robot using a STM32
 * Nucelo board. FreeRTOS is leveraged to efficiently balance the robot by
 * splitting up the tasks into the proirities that follow.
 *
 * HIGH_PIORITY:
 * 	imuDataReadTask: This task is used to read gyroscope and accelerom-
 * 	-meter data in from the MPU6050. This data is then converted into 
 * 	proper units, and then combined using a complementary filter for 
 * 	reliable and accurate data.
 *
 *	selfBalanceTask: This task uses a PID control loop to convert the
 *	IMU data into a output that will drive the motors. There are three
 *	coefficients that can be used to tweak each subsection of the control
 *	loop function.
 ******************************************************************************
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	float accel_pitch;
	float gyro_pitch;
	float comp_pitch;
} Pitch_t;

typedef struct {
	TIM_HandleTypeDef *htim;
	uint8_t channels[2];
} Motor_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_TRIALS 5						// number of attempts to check device readiness
#define MPU6050_Addr (0x68 << 1)
#define MPU6050_WAKE 0x00
#define PWR_MGMT_1_Addr 0x6B
#define PWR_MGMT_2_Addr 0x6C
#define INT_ENABLE_Addr 0x38				// address for imu registers that allows enabling of the interrupt pin
#define STANDBY_ACCEL_Y_AXIS (1UL << 4) 	// mask for accelerometer y-axis standby mode bit
#define STANDBY_GYRO_X_AXIS (1UL << 2) 		// mask for gyroscope x-axis standby mode bit
#define STANDBY_GYRO_Z_AXIS (1UL << 0)  	// mask for gyroscope z-axis standby mode bit
#define MPU6050_DATA_READY_Enable 0x01
#define ACCEL_XOUT_H_Addr 0x3B
#define ACCEL_XOUT_L_Addr 0x3C
#define ACCEL_ZOUT_H_Addr 0x3F
#define ACCEL_ZOUT_L_Addr 0x40
#define GYRO_YOUT_H_Addr 0x45
#define GYRO_YOUT_L_Addr 0x46
#define ACCEL_SENSITIVITY 16384
#define GYRO_SENSITIVITY 131
#define FILTER_COEFF 0.98

#define IMU_CALIBRATION_PRE_MSG_DELAY 1000
#define IMU_CALIBRATION_POST_MSG_DELAY 2000
#define IMU_CONFIG_NUM_SAMPLES 3

#define AUTO_RELOAD_REGISTER 4000
#define TICKS_PER_SECOND 1000

#define PRINT_I2C_ERRORS 0
#define PRINT_IMU_CONFIG_ERRORS 0
#define PRINT_IMU_CALIBRATION_ERRORS 0
#define PRINT_COMPUTED_PITCHES 1
#define PRINT_PID_ERROR 0

#define I2C_TIMEOUT_MS 10 					// milliseconds to wait per attempt
#define UART_TIMEOUT_MS HAL_MAX_DELAY 		// milliseconds to wait per uart transmit
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask", .stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal, };
/* Definitions for imuReadTask */
osThreadId_t imuReadTaskHandle;
const osThreadAttr_t imuReadTask_attributes = { .name = "imuReadTask", .stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityHigh, };
/* Definitions for balanceTask */
osThreadId_t balanceTaskHandle;
const osThreadAttr_t balanceTask_attributes = { .name = "balanceTask", .stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityHigh, };
/* Definitions for imuDataMutex */
osMutexId_t imuDataMutexHandle;
const osMutexAttr_t imuDataMutex_attributes = { .name = "imuDataMutex" };
/* Definitions for imuDataReadySem */
osSemaphoreId_t imuDataReadySemHandle;
const osSemaphoreAttr_t imuDataReadySem_attributes = { .name = "imuDataReadySem" };
/* USER CODE BEGIN PV */
volatile Pitch_t pitches = { // global reference to IMU acceleration and gyroscope estimated pitches (keep initialized to 0)
		.accel_pitch = 0.0, .gyro_pitch = 0.0, .comp_pitch = 0.0 };
float initial_accel_pitch_offset_deg = 0.0f;
Motor_t left_motor;
Motor_t right_motor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);
void startIMUReadTask(void *argument);
void selfBalanceTask(void *argument);

/* USER CODE BEGIN PFP */
// --- IMU Functions ---
static void configure_imu_power(void);
static void calibrate_imu(void);
static void process_imu_data(const uint8_t *data_buffer, Pitch_t *pitch_buffer, float dt);
static void update_global_imu_data(Pitch_t *pitch_buffer);
static void print_i2c_read_error(void);

// --- Motor Driver Functions ---
static void start_pwm_channels(void);
static void set_motor_duty_cycle(Motor_t *motor, float speed_control);

// --- Helper Functions ---
static void uart_motor_speed_polling(void);
static void uart_print(char *buffer);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int - should never return due to the FreeRTOS scheduler
 */
int main(void) {

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
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();

	/* USER CODE BEGIN 2 */
	char nl[] = "\r\n";
	uart_print(nl); // quick newline print to seperate console logs

	// Configure the Motors
	left_motor.htim = &htim2;
	left_motor.channels[0] = TIM_CHANNEL_1;
	left_motor.channels[1] = TIM_CHANNEL_2;
	right_motor.htim = &htim3;
	right_motor.channels[0] = TIM_CHANNEL_3;
	right_motor.channels[1] = TIM_CHANNEL_4;

	HAL_GPIO_WritePin(DRV8833_SLEEP_GPIO_Port, DRV8833_SLEEP_Pin, GPIO_PIN_RESET); // make sure motor is in low power (steadystate)
	start_pwm_channels();
	HAL_GPIO_WritePin(DRV8833_SLEEP_GPIO_Port, DRV8833_SLEEP_Pin, GPIO_PIN_SET); // take motor driver out of low power

	// Prepare imu
	configure_imu_power(); // configure the imu for i2c communication
	calibrate_imu(); // zero out the accelerometer's pitch calculation
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();
	/* Create the mutex(es) */
	/* creation of imuDataMutex */
	imuDataMutexHandle = osMutexNew(&imuDataMutex_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of imuDataReadySem */
	imuDataReadySemHandle = osSemaphoreNew(1, 0, &imuDataReadySem_attributes);

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

	/* creation of imuReadTask */
	imuReadTaskHandle = osThreadNew(startIMUReadTask, NULL, &imuReadTask_attributes);

	/* creation of balanceTask */
	balanceTaskHandle = osThreadNew(selfBalanceTask, NULL, &balanceTask_attributes);

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
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */
	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */
	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x10D19CE4;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
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
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */
	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */
	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 3999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
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
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */
	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */
	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 3999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */
	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(DRV8833_SLEEP_GPIO_Port, DRV8833_SLEEP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DRV8833_SLEEP_Pin */
	GPIO_InitStruct.Pin = DRV8833_SLEEP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DRV8833_SLEEP_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : MPU6050_INT_Pin */
	GPIO_InitStruct.Pin = MPU6050_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(MPU6050_INT_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*===============================================================================================*/
// IMU Functions
/*===============================================================================================*/

/**
 * @brief Prepares I2C communication and configures the MPU6050 to begin comms.
 */
static void configure_imu_power(void) {
	// Wait for the IMU to be ready
	while (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_Addr, I2C_TRIALS, I2C_TIMEOUT_MS) != HAL_OK)
		;
	// we cannot proceed until this device is ready

	uint8_t data_byte;

	// Wake up the MPU6050 from sleep mode
	data_byte = MPU6050_WAKE; // we will clear the sleep bit
	while (HAL_I2C_Mem_Write(&hi2c1, MPU6050_Addr, PWR_MGMT_1_Addr, I2C_MEMADD_SIZE_8BIT,
			&data_byte, 1, I2C_TIMEOUT_MS) != HAL_OK) {
		if (PRINT_IMU_CONFIG_ERRORS) {
			char error_buffer[] = "Failed to wake MPU6050\r\n";
			uart_print(error_buffer);
		}
	}

	// Wait a few ms for the device to stabilize after power changes
	HAL_Delay(10);

	// Set unused IMU peripherals into standby mode
	// Read the current register value to not overwrite other settings
	while (HAL_I2C_Mem_Read(&hi2c1, MPU6050_Addr, PWR_MGMT_2_Addr, I2C_MEMADD_SIZE_8BIT, &data_byte,
			1, I2C_TIMEOUT_MS) != HAL_OK) {
		if (PRINT_IMU_CONFIG_ERRORS) {
			char error_buffer[] = "Failed to read from MPU6050 PWR_MGMT_2 register\r\n";
			uart_print(error_buffer);
		}
	}

	data_byte |= (STANDBY_ACCEL_Y_AXIS | STANDBY_GYRO_X_AXIS | STANDBY_GYRO_Z_AXIS);

	// Write the modified value back to the register
	while (HAL_I2C_Mem_Write(&hi2c1, MPU6050_Addr, PWR_MGMT_2_Addr, I2C_MEMADD_SIZE_8BIT,
			&data_byte, 1,
			I2C_TIMEOUT_MS) != HAL_OK) {
		if (PRINT_IMU_CONFIG_ERRORS) {
			char error_buffer[] = "Failed to write to MPU6050 PWR_MGMT_2 register\r\n";
			uart_print(error_buffer);
		}
	}

	// Enable the Data Ready Interrupt on the MPU6050
	data_byte = MPU6050_DATA_READY_Enable; // sets the DATA_RDY_EN bit
	while (HAL_I2C_Mem_Write(&hi2c1, MPU6050_Addr, INT_ENABLE_Addr, I2C_MEMADD_SIZE_8BIT,
			&data_byte, 1,
			I2C_TIMEOUT_MS) != HAL_OK) {
		if (PRINT_IMU_CONFIG_ERRORS) {
			char error_buffer[] = "Failed to enable MPU6050 interrupt pin\r\n";
			uart_print(error_buffer);
		}
	}
}

/**
 * @brief Establishes a baseline for the IMU to disregard initial pitch.
 */
static void calibrate_imu(void) {
	uint8_t rx_buffer[14];
	float sum_accel_x_g_force = 0;
	float sum_accel_z_g_force = 0;

	char calibration_warning[] = "Calibrating... Keep robot Level!\r\n";
	uart_print(calibration_warning);
	HAL_Delay(IMU_CALIBRATION_PRE_MSG_DELAY);

	// Make sure MPU is ready before starting
	while (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_Addr, I2C_TRIALS, I2C_TIMEOUT_MS) != HAL_OK) {
		if (PRINT_IMU_CALIBRATION_ERRORS) {
			char error_buffer[] = "Warning: IMU not ready yet in configuration\r\n";
			uart_print(error_buffer);
		}
	}

	// Perform a number of reads to get a baseline measurment for zeroing out the accelerometer pitch
	for (int i = 0; i < IMU_CONFIG_NUM_SAMPLES; i++) {
		// Read raw accelerometer data
		while (HAL_I2C_Mem_Read(&hi2c1, MPU6050_Addr, ACCEL_XOUT_H_Addr, I2C_MEMADD_SIZE_8BIT,
				rx_buffer, 14, I2C_TIMEOUT_MS) != HAL_OK) {
			if (PRINT_IMU_CALIBRATION_ERRORS) {
				char error_buffer[] = "Failed to read data from IMU during configuration.\r\n";
				uart_print(error_buffer);
			}
		}

		// Since read was successful, decode it
		int16_t raw_accel_x = (int16_t) (rx_buffer[0] << 8 | rx_buffer[1]);
		int16_t raw_accel_z = (int16_t) (rx_buffer[4] << 8 | rx_buffer[5]);

		float adjusted_accel_x = ((float) raw_accel_x / ACCEL_SENSITIVITY);
		float adjusted_accel_z = ((float) raw_accel_z / ACCEL_SENSITIVITY);

		sum_accel_x_g_force += adjusted_accel_x;
		sum_accel_z_g_force += adjusted_accel_z;
	}

	// Calculate the average offset
	float avg_accel_x_g_force = sum_accel_x_g_force / IMU_CONFIG_NUM_SAMPLES;
	float avg_accel_z_g_force = sum_accel_z_g_force / IMU_CONFIG_NUM_SAMPLES;

	// Calculate the initial angle from the averages and store it
	initial_accel_pitch_offset_deg = atan2(avg_accel_x_g_force, avg_accel_z_g_force)
			* 180.0/ M_PI;

	char calibration_complete[128];
	sprintf(calibration_complete, "Calibration complete! Offset Angle: %f\r\n",
			initial_accel_pitch_offset_deg);
	uart_print(calibration_complete);

	HAL_Delay(IMU_CALIBRATION_POST_MSG_DELAY);
}

/**
 * @brief Converts raw IMU data into the process pitch values for the accelerometer, gyroscope, and complementary filter
 * @param const uint8_t* data_buffer - Buffer that contains the raw data read from the I2C channel
 * @param Pitch_t* pitch_buffer - Buffer to store the computed pitches in for return
 * @param float dt - Change in time between the last data read and the current
 */
static void process_imu_data(const uint8_t *data_buffer, Pitch_t *pitch_buffer, float dt) {
	// Combine high and low bytes
	int16_t raw_accel_x = (int16_t) (data_buffer[0] << 8 | data_buffer[1]);
	int16_t raw_accel_z = (int16_t) (data_buffer[4] << 8 | data_buffer[5]);
	int16_t raw_gyro_y = (int16_t) (data_buffer[10] << 8 | data_buffer[11]);

	float accel_x = ((float) raw_accel_x / ACCEL_SENSITIVITY);
	float accel_z = ((float) raw_accel_z / ACCEL_SENSITIVITY);
	float gyro_y = ((float) raw_gyro_y / GYRO_SENSITIVITY);

	float absolute_accel_pitch_deg = atan2(accel_x, accel_z) * 180.0 / M_PI;
	float relative_accel_pitch_deg = absolute_accel_pitch_deg - initial_accel_pitch_offset_deg;

	pitch_buffer->accel_pitch = relative_accel_pitch_deg;
	pitch_buffer->gyro_pitch = (gyro_y * dt);
	osMutexAcquire(imuDataMutexHandle, osWaitForever);
	pitch_buffer->comp_pitch = FILTER_COEFF * (pitches.comp_pitch - pitch_buffer->gyro_pitch)
			+ (1.0 - FILTER_COEFF) * pitch_buffer->accel_pitch;
	osMutexRelease(imuDataMutexHandle);
}

/**
 * @brief Updates the global references of the objects transform and pitches
 * @param Pitch_t* pitch_buffer - Buffer that contains the computed pitches to write to the gloabal reference
 */
static void update_global_imu_data(Pitch_t *pitch_buffer) {
	osMutexAcquire(imuDataMutexHandle, osWaitForever);

	// Update estimated pitch information
	pitches.accel_pitch = pitch_buffer->accel_pitch;
	pitches.gyro_pitch -= pitch_buffer->gyro_pitch;
	pitches.comp_pitch = pitch_buffer->comp_pitch;
	osMutexRelease(imuDataMutexHandle);

	if (PRINT_COMPUTED_PITCHES) {
		char uartBuffer[128];
		sprintf(uartBuffer, "accel-pitch: %f\tgyro-pitch: %f\tcomp-pitch: %f\r\n",
				pitches.accel_pitch, pitches.gyro_pitch, pitches.comp_pitch);
		uart_print(uartBuffer);
	}
}

/**
 * @brief Prints a descriptive message to UART based on the I2C error code
 */
static void print_i2c_read_error(void) {
	char errorBuffer[128];
	uint32_t i2c_error = HAL_I2C_GetError(&hi2c1);

	if (i2c_error & HAL_I2C_ERROR_BERR) {
		sprintf(errorBuffer,
				"MPU6050 Read Error: Bus Error (BERR). Check wiring/signal integrity.\r\n");
	} else if (i2c_error & HAL_I2C_ERROR_ARLO) {
		sprintf(errorBuffer,
				"MPU6050 Read Error: Arbitration Lost (ARLO). Check for noise or other masters.\r\n");
	} else if (i2c_error & HAL_I2C_ERROR_AF) {
		sprintf(errorBuffer,
				"MPU6050 Read Error: Acknowledge Failure (AF/NACK). Sensor may be busy or disconnected.\r\n");
	} else if (i2c_error & HAL_I2C_ERROR_OVR) {
		sprintf(errorBuffer, "MPU6050 Read Error: Overrun/Underrun (OVR).\r\n");
	} else {
		sprintf(errorBuffer, "MPU6050 Read Error: Unknown I2C Error. Code: 0x%lX\r\n", i2c_error);
	}
	uart_print(errorBuffer);
}

/**
 * @brief Handles hardware level interrupts
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// Check if the interrupt came from the IMU's interrupt pin
	if (GPIO_Pin == MPU6050_INT_Pin) {
		osSemaphoreRelease(imuDataReadySemHandle);
	}
}

/*===============================================================================================*/
// Motor Driver Functions
/*===============================================================================================*/

/**
 * @brief Begins the timer pwm channels and puts them in a stopped position
 */
static void start_pwm_channels(void) {
	// Begin the PWM
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	// Default motors with no movement
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

/**
 * @brief Sets a motor's speed and direction.
 * @param Motor_t motor : Motor structure to be driven.
 * @param uint8_t drive_channel : Indicates which channel to use PWM on -- channel 1, 2, or 0 if neither needs pwm.
 * @param float duty_coeff : The fraction of power to drive the motor. *TODO* figure out margin of accuracy
 */
static void set_motor_duty_cycle(Motor_t *motor, float speed_control) {
	// Clamp the speed
	if (speed_control > 1.0f) {
		speed_control = 1.0f;
	}
	if (speed_control < -1.0f) {
		speed_control = -1.0f;
	}

	uint16_t crr = fabsf(AUTO_RELOAD_REGISTER * speed_control);
	// uint16_t crr = AUTO_RELOAD_REGISTER/2 + fabsf(AUTO_RELOAD_REGISTER/2 * speed_control);

	if (speed_control > 0) {
		__HAL_TIM_SET_COMPARE(motor->htim, motor->channels[0], 0);
		__HAL_TIM_SET_COMPARE(motor->htim, motor->channels[1], crr);
	} else if (speed_control < 0) {
		__HAL_TIM_SET_COMPARE(motor->htim, motor->channels[0], crr);
		__HAL_TIM_SET_COMPARE(motor->htim, motor->channels[1], 0);
	} else {
		__HAL_TIM_SET_COMPARE(motor->htim, motor->channels[0], 0);
		__HAL_TIM_SET_COMPARE(motor->htim, motor->channels[1], 0);
	}
}

/*===============================================================================================*/
// Helper Functions
/*===============================================================================================*/

/**
 * @brief Helper used to constantly poll the usart line to quickly manipulate speed during the robot's runtime
 */
static void uart_motor_speed_polling(void) {
	while (1) {
		uint8_t index = 0;
		uint8_t recieved_byte;
		uint16_t recieved_byte_size = sizeof(recieved_byte);

		char rx_buffer[20];
		uint16_t rx_buffer_size = sizeof(rx_buffer) / sizeof(rx_buffer[0]);
		memset(rx_buffer, 0, sizeof(rx_buffer));

		char msg_buffer[32] = "Enter a speed: ";
		uart_print(msg_buffer);

		while (index < rx_buffer_size - 1) {
			HAL_UART_Receive(&huart2, &recieved_byte, recieved_byte_size, HAL_MAX_DELAY);

			if (recieved_byte == '\r' || recieved_byte == '\n') {
				char newline[] = "\r\n";
				uart_print(newline);
				index = 0;
				break;
			} else if (recieved_byte == 127 || recieved_byte == 8) {
				if (index > 0) {
					index--;
					char backspace_seq[] = "\b \b";
					uart_print(backspace_seq);
				}

			} else {
				HAL_UART_Transmit(&huart2, &recieved_byte, sizeof(recieved_byte),
				HAL_MAX_DELAY); // print the typed character back to user
				char recieved_char = (char) recieved_byte;
				rx_buffer[index++] = recieved_char;
			}
		}

		float speed = atof(rx_buffer);

		// Clamp the value
		speed = (speed > 1) ? 1 : speed;
		speed = (speed < -1) ? -1 : speed;

		sprintf(msg_buffer, "Setting speed to: %.2f\r\n", speed);
		uart_print(msg_buffer);

		set_motor_duty_cycle(&left_motor, speed);
		set_motor_duty_cycle(&right_motor, speed);
	}
}

/**
 * @brief Prints a character buffer over the uart communication channel
 * @param char* buffer - Buffer to be transmitted over the UART line
 */
static void uart_print(char *buffer) {
	HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), UART_TIMEOUT_MS);
}

/*===============================================================================================*/

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Func tion implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startIMUReadTask */
/**
 * @brief Function implementing the imuReadTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startIMUReadTask */
void startIMUReadTask(void *argument) {
	/* USER CODE BEGIN startIMUReadTask */
	uint32_t last_tick = osKernelGetTickCount(); // initialize starting tick

	/* Infinite loop */
	for (;;) {
		// Wait for interrupt pin to signal new data is available
		osSemaphoreAcquire(imuDataReadySemHandle, osWaitForever);

		// Keep track of the system tick so we can integrate gyro rotation
		uint32_t current_tick = osKernelGetTickCount();
		float dt = (float) (current_tick - last_tick) / TICKS_PER_SECOND;
		last_tick = current_tick;

		// Read the accelerometer and gyroscope data into a buffer
		HAL_StatusTypeDef status;
		uint8_t rx_buffer[14];
		status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_Addr, ACCEL_XOUT_H_Addr,
		I2C_MEMADD_SIZE_8BIT, rx_buffer, 14, I2C_TIMEOUT_MS);

		if (status != HAL_OK) {
			// Attempt to recover the I2C bus by re-initializing it
			HAL_I2C_DeInit(&hi2c1);
			HAL_I2C_Init(&hi2c1);

			if (PRINT_I2C_ERRORS) {
				print_i2c_read_error();
			}

			//osDelay(100); // give the i2c line time to recover
		}

		Pitch_t pitch_buffer;
		process_imu_data(rx_buffer, &pitch_buffer, dt);
		update_global_imu_data(&pitch_buffer);

		osDelay(10); // short wait between reads
	}
	/* USER CODE END startIMUReadTask */
}

/* USER CODE BEGIN Header_selfBalanceTask */
/**
 * @brief Function implementing the balanceTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_selfBalanceTask */
void selfBalanceTask(void *argument) {
	/* USER CODE BEGIN selfBalanceTask */
	uint32_t last_tick = osKernelGetTickCount(); // initialize starting tick
	float integralSum = 0;
	float prev_error = 0.0;

	/* Infinite loop */
	for (;;) {
		osMutexAcquire(imuDataMutexHandle, osWaitForever); // aquire imuData mutex before reading complementary pitch
		float error = 0 - pitches.comp_pitch; // get error (setpoint is 0 degrees)
		osMutexRelease(imuDataMutexHandle);

		// Robot is laying on its side - kill motors, reset variables, and skip cycle
		if (abs(error) >= 40) {
			set_motor_duty_cycle(&left_motor, 0);
			set_motor_duty_cycle(&right_motor, 0);
			last_tick = osKernelGetTickCount(); // reset last tick in case robot has been on it's side for a while
			continue;
		}

		uint32_t current_tick = osKernelGetTickCount(); // initialize starting tick
		uint32_t dt = (current_tick - last_tick) / TICKS_PER_SECOND;
		last_tick = current_tick;

		float kp = 0.20;
		float ki = 0.00;
		float kd = 0.00;

		// Calculate p-error
		float p_out = kp * (error);

		// Calculate i-error
		// *** IMPORTANT : Think about clamping the integral sum to prevent huge corrections when first placing robot down ***
		integralSum += error * dt;
		float i_out = ki * (integralSum);

		// Calculate d-error
		float d_out = 0.0f;
		if (dt > 0) { // protect from divide-by-zero error
			d_out = kd * ((error - prev_error) / dt);
		}

		// Final calculation
		float output = p_out + i_out + d_out;

		prev_error = error;

		set_motor_duty_cycle(&left_motor, output);
		set_motor_duty_cycle(&right_motor, output);

		if (PRINT_PID_ERROR) {
			char print_buf[32];
			sprintf(print_buf, "pid error: %f\r\n", output);
			uart_print(print_buf);
		}

		osDelay(10);
	}
	/* USER CODE END selfBalanceTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
