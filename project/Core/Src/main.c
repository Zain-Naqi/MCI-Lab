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
#include "pid.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;        

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
char Buffer[100];
float y_acc_offset = 0.0;
float angleY = 0.0;
int16_t y_raw;
PIDController balance_pid;
int16_t motor_direction;
float error;
float tiltAngle = 0.0;
uint16_t pwm_freq_prescaler = 47;
uint16_t duty_cycle = 0;
uint16_t pwm_period = 99; 
float error = 0;
float int_error = 0;
float target_angle = 0.0;
#define IMU_ADDRESS_READ 0x33
#define IMU_ADDRESS_WRITE 0x32
#define CTRL_REG_1 0x20
#define Y_OUT_H 0x2B
#define Y_OUT_L 0x2A
#define SCALE_2G  3.9f
#define RAD_TO_DEG 57.2957795f
#define MOTOR_MAX_PWM 75
#define MOTOR_MIN_PWM 0
#define CTRL_REG1 0x20             // Gyroscope control register 1 address
#define CTRL_REG1_VAL 0b00011100   
#define MAX_ERROR 20// Enables: Power, X/Y/Z axes, and sets Output Data Rate (ODR)

#define CTRL_REG4  0x23
#define CTRL_REG4_VAL  0b00000000  // Full-scale = ±245 dps

float gyro_offset = 0.0;

#define SAMPLE_SIZE 5
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void calibrate_accel_y(void) {

  const int num_readings = 20;
  float acc_sum = 0.0f;
  uint8_t yl, yh;

  for (int i = 0; i < num_readings; i++) {

      // Read X-axis low and high bytes
      HAL_I2C_Mem_Read(&hi2c1, IMU_ADDRESS_READ, Y_OUT_L, I2C_MEMADD_SIZE_8BIT, &yl, 1, HAL_MAX_DELAY);
      HAL_I2C_Mem_Read(&hi2c1, IMU_ADDRESS_READ, Y_OUT_H, I2C_MEMADD_SIZE_8BIT, &yh, 1, HAL_MAX_DELAY);

      int16_t raw_y = (int16_t)((yh << 8) | yl);
      int16_t accel_y = raw_y >> 6;
      float accel_y_mg = accel_y * SCALE_2G;
      float accel_y_g = (accel_y_mg / 1000.0f)*RAD_TO_DEG;

      acc_sum += accel_y_g;
      HAL_Delay(50);  // small delay between readings

  }

  y_acc_offset = acc_sum / num_readings;  // calculate the offset for gyro 

}
float read_accel_y(void) {

  uint8_t yl, yh;

  HAL_I2C_Mem_Read(&hi2c1, IMU_ADDRESS_READ, Y_OUT_L, I2C_MEMADD_SIZE_8BIT, &yl, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c1, IMU_ADDRESS_READ, Y_OUT_H, I2C_MEMADD_SIZE_8BIT, &yh, 1, HAL_MAX_DELAY);

  int16_t raw_y = (int16_t)((yh<<8) | yl);
  int16_t accel_y = raw_y>>6; // for 10 bit
  float accel_y_mg = accel_y * SCALE_2G;  
  return (accel_y_mg / 1000.0f) * RAD_TO_DEG;

}

void init_acc(){

  uint8_t ctrl1 = 0x63; 
  uint8_t ctrl4 = 0x00;   // ±2g, normal resolution

  HAL_I2C_Mem_Write(&hi2c1, IMU_ADDRESS_WRITE, 0x20, I2C_MEMADD_SIZE_8BIT, &ctrl1, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Write(&hi2c1, IMU_ADDRESS_WRITE, 0x23, I2C_MEMADD_SIZE_8BIT, &ctrl4, 1, HAL_MAX_DELAY);


}


void gyro_init()
{
    uint8_t tx[2] = { 0x20, 0b00011111 }; // Write CTRL_REG1 = 0x1F

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); // CS low (start communication)
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);       // Send address + value
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);   // CS high (end communication)
}
uint8_t gyro_read_register(uint8_t reg)
{
    uint8_t tx = 0x80 | reg; // Set MSB to indicate read operation
    uint8_t rx = 0;

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); // CS low
    HAL_SPI_Transmit(&hspi1, &tx, 1, HAL_MAX_DELAY);       // Send register address
    HAL_SPI_Receive(&hspi1, &rx, 1, HAL_MAX_DELAY);        // Receive data byte
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);   // CS high

    return rx; // Return the value read from the register
}
void calibrate_gyro(void) {

  const int num_readings = 20;
  float gyro_sum = 0.0f;
  uint8_t y_l, y_h;
  int16_t y_raw;

  for (int i = 0; i < num_readings; ++i) {

    // Read low and high registers for y-axis:
    y_l = gyro_read_register(0x2A);
    y_h = gyro_read_register(0x2B);

    // Combine low and high bytes into 16-bit signed integers
    y_raw = (int16_t)(y_h << 8 | y_l);
    
    gyro_sum += y_raw;
  }

  gyro_offset = gyro_sum / num_readings;

}
void print_angle() { 

  char msg[128];
  uint8_t y_l = gyro_read_register(0x28);
  uint8_t y_h = gyro_read_register(0x29);

  // Combine low and high bytes into 16-bit signed integers
  y_raw = (int16_t)(y_h << 8 | y_l);
  float final_accel = read_accel_y() - y_acc_offset;
  float gyro_filtered = y_raw - gyro_offset;
  // Read low and high registers for y-axis:
  

    // === Convert to dps (°/s) using sensitivity scale ===
  // According to L3GD20 datasheet, ±245 dps mode → 8.75 mdps/digit = 0.00875 dps/digit
  float final_gyro = -(gyro_filtered * 0.00875f);

  angleY = 0.98 * (angleY + final_gyro * (1 / SAMPLE_SIZE)) + 0.02 * final_accel;

}


void setPWMspeed(void) {
    char debug_buf[100];

    // Calculate error: target - current angle
    error = target_angle - angleY;
    sprintf(debug_buf, "Angle: %.2f\r\n", angleY);
    HAL_UART_Transmit(&huart1, (uint8_t*)debug_buf, strlen(debug_buf), HAL_MAX_DELAY);

    // Apply PID (target as setpoint, angleY as measurement)
    PIDController_Update(&balance_pid, target_angle, angleY);

    // Get PID output
    
    motor_direction = (int16_t)balance_pid.motor_output; //fr direction
    int16_t pwm_dutycycle= abs(motor_direction); //for speed (duty cycle) 
    int16_t final_dutycycle = (pwm_dutycycle/((MAX_ERROR*balance_pid.kp)+150))*MOTOR_MAX_PWM;
    float pwm_percentage = (final_dutycycle * 100.0f) / MOTOR_MAX_PWM;
    if (pwm_percentage <= 3.0f) {
        final_dutycycle = 0;
    }




    // sprintf(debug_buf, "\tPID Output (PWM): %d\r\n", final_dutycycle);
    // HAL_UART_Transmit(&huart1, (uint8_t*)debug_buf, strlen(debug_buf), HAL_MAX_DELAY);

    // Clamp PID output

    //abs value change for compare 
    // if (pwm_dutycycle > MOTOR_MAX_PWM) pwm_dutycycle  = MOTOR_MAX_PWM;
    // if (pwm_dutycycle  < MOTOR_MIN_PWM) pwm_dutycycle  = MOTOR_MIN_PWM;

    // Determine direction and apply PWM
    
    if(angleY>MAX_ERROR || angleY<-MAX_ERROR){ // Stop
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        // HAL_UART_Transmit(&huart1, (uint8_t*)"Direction: Stop\r\n", 17, HAL_MAX_DELAY);
    }else{
      if (motor_direction>=0) { // Forward
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, final_dutycycle );
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, final_dutycycle );
        // HAL_UART_Transmit(&huart1, (uint8_t*)"Direction: Forward\r\n", 20, HAL_MAX_DELAY);
      //check for motor speed and 0><
    } else if (motor_direction< 0) { // Backward
        
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
       

       __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, final_dutycycle );
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, final_dutycycle );
        // HAL_UART_Transmit(&huart1, (uint8_t*)"Direction: Backward\r\n", 22, HAL_MAX_DELAY);
        // angle +-20 stop 

    } 
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  gyro_init();       // Initialize gyro (enable power and axes)
  HAL_Delay(500);    // Delay to allow sensor to stabilize
  calibrate_gyro();  
    // === Gyro End === //
  init_acc();
  HAL_Delay(500);    // Delay to allow sensor to stabilize
  calibrate_accel_y();

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,0);
  balance_pid.kp =20;
  balance_pid.ki = 8;
  balance_pid.kd = 0;
  balance_pid.sampling_time = SAMPLE_SIZE; 
  PIDController_Init(&balance_pid); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_9);
      print_angle(); 
      setPWMspeed();
      HAL_Delay(SAMPLE_SIZE);   
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim3.Init.Prescaler = pwm_freq_prescaler;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = pwm_period;
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
  sConfigOC.Pulse = duty_cycle;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  htim4.Init.Prescaler =pwm_freq_prescaler;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = pwm_period;
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
  sConfigOC.Pulse = duty_cycle;
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
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);


  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
