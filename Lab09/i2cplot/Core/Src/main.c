
/*
IMPORTANT!!!!!!
The following code contains for task 3 which also covers the rest of the task. who am i is presented in the mpu_exists function
*/



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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>  
#include <math.h> 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265359
#define RAD_TO_DEG 57.2957795131 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t accelX_raw, gyroX_raw;
float gForceX, rotX;
char buffer[100];
float angleX = 0.0; 
float dt = 0.1;
uint8_t Buffer[25] = {0};
uint8_t Space[] = "-";
uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
uint8_t EndMSG[] = "Done! \r\n\r\n";
uint8_t ErrorMSG[] = "\t Error in gyro \r\n";
float acc_Offset = 0.0;
float gyro_Offset = 0.0;

#define MPU6050_ADDR 0x68 << 1  // mpu address
#define PWR_MGMT_1   0x6B
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG  0x1B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

//the function is used to check if the connection exists. uncomment it to verify.
// void MPU6050_Exists(void){
//   uint8_t i=0,ret;
//   HAL_Delay(1000);
//   HAL_UART_Transmit(&huart1, StartMSG, sizeof(StartMSG), 10000);
//   for(i=0; i<128;i++){
//   ret = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i<<1),3,5);
//   if(ret!=HAL_OK){
//     HAL_UART_Transmit(&huart1, Space, sizeof(Space), 10000);
//   }else if(ret==HAL_OK){
//     sprintf(Buffer, "0x%X", i);
//     HAL_UART_Transmit(&huart1, Buffer, sizeof(Buffer), 10000);
//   }
//   }
//   HAL_UART_Transmit(&huart1, EndMSG, sizeof(EndMSG), 10000);
//   uint8_t who_am_i = 0;
// HAL_StatusTypeDef status;
// HAL_Delay(2000);
// status = HAL_I2C_Mem_Read(&hi2c2,
//                           0x68 << 1,     // Device address (8-bit)
//                           0x75,          // WHO_AM_I register
//                           I2C_MEMADD_SIZE_8BIT, // 8-bit register address
//                           &who_am_i,     // Buffer to store result
//                           1,             // One byte
//                           100);          // Timeout in ms

// if (status == HAL_OK) {
//     sprintf((char*)Buffer, "\r\nWHO_AM_I: 0x%X\r\n", who_am_i);
//     HAL_UART_Transmit(&huart1, Buffer, strlen((char*)Buffer), 1000);
// } else {
//     HAL_UART_Transmit(&huart1, (uint8_t *)"\r\nWHO_AM_I read failed\r\n", 26, 1000);
// }

//   uint8_t reg_config = 0;
//   uint8_t sample_rate = 0;
  
//   HAL_I2C_Mem_Read(&hi2c2, 0x68<<1, 25, I2C_MEMADD_SIZE_8BIT, &sample_rate, 1, 100);
//   HAL_I2C_Mem_Read(&hi2c2, 0x68<<1, 26, I2C_MEMADD_SIZE_8BIT, &reg_config, 1, 100);
//   sprintf((char*)Buffer, "Configure Register: 0x%X, Sample Rate Register: 0x%X\r\n",reg_config, sample_rate);
//   HAL_UART_Transmit(&huart1, Buffer, strlen((char*)Buffer), 1000);

// }

void MPU6050_Init(void) {
    uint8_t data;
    // wake up MPU6050
    data = 0;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);

    // Set accelerometer to 2g
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY);

    //250deg
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
}

void Read_MPU6050(void) {
    uint8_t Rec_Data[4];
    // Read accelerometer x-axis
    uint8_t isReading=0;
    isReading = HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 2, HAL_MAX_DELAY);
    if(isReading!=HAL_OK){
      HAL_UART_Transmit(&huart1, ErrorMSG, sizeof(ErrorMSG), HAL_MAX_DELAY);
    }
    accelX_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    gForceX = accelX_raw / 16384.0;
    gForceX = gForceX * RAD_TO_DEG;
    gForceX-=acc_Offset;

    // Read gyroscope x-axis
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 2, HAL_MAX_DELAY);
    gyroX_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    rotX = (gyroX_raw / 131.0);
    rotX-=gyro_Offset;
    angleX = 0.98 * (angleX + rotX * dt) + 0.02 * gForceX;
}

void Print_Data(void) {
    sprintf(buffer, "%.2f,%.2f,%.2f\r\n", gForceX, rotX, angleX);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void callibrate_sensor() {
    int num_readings = 20;
    float acc_sum = 0;
    float gyro_sum = 0;
    uint8_t Rec_Data[2];
    for (int i = 0; i < num_readings; i++) {
        HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 2, HAL_MAX_DELAY);
        int16_t raw_accel = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        float accel_g = (raw_accel / 16384.0f)*RAD_TO_DEG;

        HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 2, HAL_MAX_DELAY);
        int16_t raw_gyro = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        float gyro_dps = raw_gyro / 131.0f;

        acc_sum += accel_g;
        gyro_sum += gyro_dps;

        HAL_Delay(100);
    }
    acc_Offset = acc_sum / num_readings;
    gyro_Offset = gyro_sum / num_readings;    
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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  // MPU6050_Exists(); //uncomment it to check whether mpu exists.
  MPU6050_Init();
  callibrate_sensor();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    Read_MPU6050();
    Print_Data();
    HAL_Delay(100);
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
                              |RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00201D2B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();


  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
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