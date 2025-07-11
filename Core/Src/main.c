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
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
} Transform_t;

typedef struct {
	float accel_pitch;
	float gyro_pitch;
	float comp_pitch;
} Pitch_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_TRIALS 5 			    	// number of attempts to check device readiness
#define I2C_TIMEOUT_MS 10 			// milliseconds to wait per attempt
#define MPU6050_Addr (0x68 << 1)
#define PWR_MGMT_1_Addr 0x6B
#define PWR_MGMT_2_Addr 0x6C
#define INT_ENABLE_Addr 0x38
#define STBY_YA (1UL << 4) 			// mask for accelerometer y-axis standby mode bit
#define STBY_XG (1UL << 2) 			// mask for gyroscope x-axis standby mode bit
#define STB_ZG (1UL << 0)  			// mask for gyroscope z-axis standby mode bit

#define ACCEL_XOUT_H_Addr 0x3B
#define ACCEL_XOUT_L_Addr 0x3C
#define ACCEL_ZOUT_H_Addr 0x3F
#define ACCEL_ZOUT_L_Addr 0x40
#define GYRO_YOUT_H_Addr 0x45
#define GYRO_YOUT_L_Addr 0x46

#define ACCEL_SENSITIVITY 16384
#define GYRO_SENSITIVITY 131

#define UART_TIMEOUT_MS HAL_MAX_DELAY 		// milliseconds to wait per uart transmit

#define FILTER_COEFF 0.98
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
/* Definitions for imuDataMutex */
osMutexId_t imuDataMutexHandle;
const osMutexAttr_t imuDataMutex_attributes = { .name = "imuDataMutex" };
/* Definitions for imuDataReadySem */
osSemaphoreId_t imuDataReadySemHandle;
const osSemaphoreAttr_t imuDataReadySem_attributes = { .name = "imuDataReadySem" };
/* USER CODE BEGIN PV */
volatile Transform_t transform; // global reference to IMU acceleration and gyroscope data
volatile Pitch_t pitches = { // global reference to IMU acceleration and gyroscope estimated pitches
		.accel_pitch = 0.0, .gyro_pitch = 0.0, .comp_pitch = 0.0 };
float initial_accel_pitch_offset_deg = 0.0f;
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

/* USER CODE BEGIN PFP */
static void configure_imu_power(void);
static void calibrate_imu(void);
static void process_imu_data(const uint8_t *data_buffer, Transform_t *transform_buffer,
		Pitch_t *pitch_buffer, float dt);
static void update_global_imu_data(Transform_t *transform_buffer, Pitch_t *pitch_buffer);
static void handle_i2c_read_error();
static void uart_print(char *buffer);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
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
	configure_imu_power();
	calibrate_imu();
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // turn led on to indicate safe usage
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
	refaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* creation of imuReadTask */
	imuReadTaskHandle = osThreadNew(startIMUReadTask, NULL, &imuReadTask_attributes);

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

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */
	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 8000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */
	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 8000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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
/*	IMU Functions																				                                         */
/*===============================================================================================*/
/**
 * @brief Configures the power settings of the IMU
 */
static void configure_imu_power() {
	// Wait for the IMU to be ready
	while (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_Addr, I2C_TRIALS,
	I2C_TIMEOUT_MS) != HAL_OK) {
		HAL_Delay(5); // busy-wait for IMU to start up
	}

	HAL_StatusTypeDef status;
	uint8_t data_byte;

	/*** 1. Wake up the MPU6050 from sleep mode ***/
	data_byte = 0x00; // Clears the sleep bit
	status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_Addr, PWR_MGMT_1_Addr,
	I2C_MEMADD_SIZE_8BIT, &data_byte, 1, I2C_TIMEOUT_MS);
	if (status != HAL_OK) {
		// Handle error
	}
	// Wait a few ms for the device to stabilize after power changes
	HAL_Delay(10);

	/*** 2. Set unused IMU peripherals into standby mode ***/
	// Read the current register value to not overwrite other settings
	status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_Addr, PWR_MGMT_2_Addr,
	I2C_MEMADD_SIZE_8BIT, &data_byte, 1, I2C_TIMEOUT_MS);
	if (status != HAL_OK) {
		// Handle error
	}

	// Combine masks
	data_byte |= (STBY_YA | STBY_XG | STB_ZG);

	// Write the modified value back to the register
	status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_Addr, PWR_MGMT_2_Addr,
	I2C_MEMADD_SIZE_8BIT, &data_byte, 1, I2C_TIMEOUT_MS);
	if (status != HAL_OK) {
		// Handle error
	}

	/*** 3. Enable the Data Ready Interrupt on the MPU6050 ***/
	data_byte = (0x01 << 0); // Sets the DATA_RDY_EN bit
	status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_Addr, INT_ENABLE_Addr,
	I2C_MEMADD_SIZE_8BIT, &data_byte, 1, I2C_TIMEOUT_MS);
	if (status != HAL_OK) {
		// Handle error
	}
}

/**
 * @brief Establishes a baseline for the IMU to disregard initial pitch.
 */
static void calibrate_imu(void) {
	uint8_t rx_buffer[14];
	float sum_accel_x_g_force = 0;
	float sum_accel_z_g_force = 0;
	const int num_samples = 3; // take 100 samples for a stable average

	char buffer[256];
	sprintf(buffer, "Calibrating... Keep robot level!\r\n");
	uart_print(buffer);
	HAL_Delay(1000);

	// Make sure MPU is ready before starting
	while (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_Addr, I2C_TRIALS, I2C_TIMEOUT_MS) != HAL_OK) {
		HAL_Delay(10);
	}

	for (int i = 0; i < num_samples; i++) {
		// Read raw accelerometer data
		while (HAL_I2C_Mem_Read(&hi2c1, MPU6050_Addr, ACCEL_XOUT_H_Addr,
		I2C_MEMADD_SIZE_8BIT, rx_buffer, 14, I2C_TIMEOUT_MS) != HAL_OK) {
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
	float avg_accel_x_g_force = sum_accel_x_g_force / num_samples;
	float avg_accel_z_g_force = sum_accel_z_g_force / num_samples;

	// Calculate the initial angle from the averages and store it
	initial_accel_pitch_offset_deg = atan2(avg_accel_x_g_force, avg_accel_z_g_force) * 180.0 / M_PI;

	sprintf(buffer, "Calibration complete! Offset Angle: %f\r\n", initial_accel_pitch_offset_deg);
	uart_print(buffer);
	HAL_Delay(2000);
}

/**
 * @brief Converts raw IMU data into the process pitch values for the accelerometer, gyroscope, and complementary filterv
 */
static void process_imu_data(const uint8_t *data_buffer, Transform_t *transform_buffer,
		Pitch_t *pitch_buffer, float dt) {
	// Combine high and low bytes
	int16_t raw_accel_x = (int16_t) (data_buffer[0] << 8 | data_buffer[1]);
	int16_t raw_accel_z = (int16_t) (data_buffer[4] << 8 | data_buffer[5]);
	int16_t raw_gyro_y = (int16_t) (data_buffer[10] << 8 | data_buffer[11]);

	transform_buffer->accel_x = ((float) raw_accel_x / ACCEL_SENSITIVITY);
	transform_buffer->accel_z = ((float) raw_accel_z / ACCEL_SENSITIVITY);
	transform_buffer->gyro_y = ((float) raw_gyro_y / GYRO_SENSITIVITY);

	float absolute_accel_pitch_deg = atan2(transform_buffer->accel_x, transform_buffer->accel_z)
			* 180.0 / M_PI;
	float relative_accel_pitch_deg = absolute_accel_pitch_deg - initial_accel_pitch_offset_deg;

	pitch_buffer->accel_pitch = relative_accel_pitch_deg;
	pitch_buffer->gyro_pitch = transform_buffer->gyro_y * dt;
	pitch_buffer->comp_pitch = FILTER_COEFF * (pitch_buffer->comp_pitch - pitch_buffer->gyro_pitch)
			+ (1.0 - FILTER_COEFF) * pitch_buffer->accel_pitch;
}

/**
 * @brief Updates the global references of the objects transform and pitches
 */
static void update_global_imu_data(Transform_t *transform_buffer, Pitch_t *pitch_buffer) {
	if (osMutexAcquire(imuDataMutexHandle, I2C_TIMEOUT_MS) == osOK) {
		// Update relevant transform information
		transform.accel_x = transform_buffer->accel_x;
		transform.accel_z = transform_buffer->accel_z;
		transform.gyro_y = transform_buffer->gyro_y;

		// Update estimated pitch information
		pitches.accel_pitch = pitch_buffer->accel_pitch;
		pitches.gyro_pitch -= pitch_buffer->gyro_pitch;
		pitches.comp_pitch = pitch_buffer->comp_pitch;
		osMutexRelease(imuDataMutexHandle);

		char uartBuffer[128];
		sprintf(uartBuffer, "accel-pitch: %f\tgyro-pitch: %f\tcomp-pitch: %f\r\n",
				pitches.accel_pitch, pitches.gyro_pitch, pitches.comp_pitch);
		uart_print(uartBuffer);
	} else {
		// Handle mutex timeout Error
		char errorBuffer[128];
		sprintf(errorBuffer, "Not able to aquire imuDataMutex in time\r\n");
		uart_print(errorBuffer);
	}
}

/**
 * @brief Prints a descriptive message to UART based on the I2C error code
 */
static void handle_i2c_read_error() {
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

/**
 * @brief Prints a character buffer over the uart communication channel
 */
static void uart_print(char* buffer) {
	HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), UART_TIMEOUT_MS);
}
/*===============================================================================================*/
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
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
		float dt = (float) (current_tick - last_tick) / 1000.0f;
		last_tick = current_tick;

		// Read the accelerometer and gyroscope data into a buffer
		HAL_StatusTypeDef status;
		uint8_t rx_buffer[14];
		status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_Addr, ACCEL_XOUT_H_Addr,
		I2C_MEMADD_SIZE_8BIT, rx_buffer, 14, I2C_TIMEOUT_MS);

		if (status != HAL_OK) {
			handle_i2c_read_error();

			// Attempt to recover the I2C bus by re-initializing it
			HAL_I2C_DeInit(&hi2c1);
			HAL_I2C_Init(&hi2c1);
			osDelay(100); // give the i2c line time to recover
		}

		Transform_t transform_buffer;
		Pitch_t pitch_buffer;
		process_imu_data(rx_buffer, &transform_buffer, &pitch_buffer, dt);
		update_global_imu_data(&transform_buffer, &pitch_buffer);
		osDelay(10); // short wait between reads
	}
	/* USER CODE END startIMUReadTask */
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
