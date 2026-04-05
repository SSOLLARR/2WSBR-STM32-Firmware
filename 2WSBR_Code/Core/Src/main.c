/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* MPU6050 Configuration */
#define MPU6050_ADDR       (0x68 << 1)
#define WHO_AM_I_REG       0x75
#define PWR_MGMT_1_REG     0x6B
#define ACCEL_XOUT_H_REG   0x3B

/* Operational Modes */
#define MODE_BALANCING      0
#define MODE_STEP_RESPONSE  1
#define MODE_SWEEP          2
#define MODE_FALL_TEST      3

/* Filter & Control Constants */
#define ALPHA           0.97f
#define DT              0.005f
#define RAD_TO_DEG      57.29577951f
#define ACCEL_SCALE     16384.0f
#define GYRO_SCALE      131.0f
#define DIVIDER_RATIO   6.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Sensor Data Buffers */
uint8_t rx_data[14];
int16_t raw_acc_x, raw_acc_y, raw_acc_z;
int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
float estimated_pitch = 0.0f;

/* PID Parameters */
float Kp = 500.0f;
float Ki = 0.0f;
float Kd = 11.5f;

float target_angle = -0.814726472f;
float pid_error = 0.0f;
float previous_error = 5.0f;
float pid_integral = 0.0f;
float pid_output = 0.0f;

/* System States */
float battery_voltage = 0.0f;
float current_speed = 0.0f;
int test_mode = MODE_BALANCING;   /* Set to fall test */
uint32_t test_timer = 0;
float test_speed = 0.0f;

static volatile uint8_t uart_tx_busy = 0;
uint16_t telemetry_counter = 0;
uint16_t loop_exec_time_us = 0;
uint16_t loop_period_us = 0;
uint16_t last_loop_stamp_us = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void MPU6050_Init(void)
{
    uint8_t data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
}

HAL_StatusTypeDef i2c_status;

void MPU6050_Read(void)
{
    i2c_status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rx_data, 14, 2);

    if (i2c_status == HAL_OK) {
        raw_acc_x  = (int16_t)(rx_data[0]  << 8 | rx_data[1]);
        raw_acc_y  = (int16_t)(rx_data[2]  << 8 | rx_data[3]);
        raw_acc_z  = (int16_t)(rx_data[4]  << 8 | rx_data[5]);

        raw_gyro_x = (int16_t)(rx_data[8]  << 8 | rx_data[9]);
        raw_gyro_y = (int16_t)(rx_data[10] << 8 | rx_data[11]);
        raw_gyro_z = (int16_t)(rx_data[12] << 8 | rx_data[13]);
    } else {
        /* Kill motor outputs on sensor failure */
        htim1.Instance->CCR1 = 0;
        htim4.Instance->CCR1 = 0;
    }
}

float update_complementary_filter(float acc_pitch, float gyro_rate)
{
    estimated_pitch = ALPHA * (estimated_pitch + gyro_rate * DT) + (1.0f - ALPHA) * acc_pitch;
    return estimated_pitch;
}

float calculate_PID(float current_pitch, float gyro_rate)
{
    pid_error = current_pitch - target_angle;
    float P_out = Kp * pid_error;

    pid_integral += pid_error * DT;
    if (pid_integral > 400.0f)  pid_integral = 400.0f;
    if (pid_integral < -400.0f) pid_integral = -400.0f;
    float I_out = Ki * pid_integral;

    float D_out = Kd * gyro_rate;

    pid_output = P_out + I_out + D_out;
    return pid_output;
}

void set_motor_speed(float target_speed)
{
    if (isnan(target_speed) || isinf(target_speed)) {
        htim1.Instance->CCR1 = 0;
        htim4.Instance->CCR1 = 0;
        return;
    }

    if (target_speed > 3000.0f)  target_speed = 3000.0f;
    if (target_speed < -3000.0f) target_speed = -3000.0f;

    static float slewed_speed = 0.0f;

    float max_change_per_loop = 2000.0f;
    float instant_kick_speed  = 2000.0f;

    if (slewed_speed == 0.0f && target_speed > instant_kick_speed) {
        slewed_speed = instant_kick_speed;
    } else if (slewed_speed == 0.0f && target_speed < -instant_kick_speed) {
        slewed_speed = -instant_kick_speed;
    } else {
        if (target_speed > slewed_speed + max_change_per_loop) {
            slewed_speed += max_change_per_loop;
        } else if (target_speed < slewed_speed - max_change_per_loop) {
            slewed_speed -= max_change_per_loop;
        } else {
            slewed_speed = target_speed;
        }
    }

    float applied_speed = slewed_speed;

    if (applied_speed >= 0) {
        HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR_R_GPIO_Port, DIR_R_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR_R_GPIO_Port, DIR_R_Pin, GPIO_PIN_RESET);
        applied_speed = -applied_speed;
    }

    if (applied_speed < 20.0f) {
        htim1.Instance->CCR1 = 0;
        htim4.Instance->CCR1 = 0;
        return;
    }

    if (applied_speed > 3000.0f) applied_speed = 3000.0f;

    uint32_t timer_arr = (uint32_t)(1000000.0f / applied_speed) - 1;

    __disable_irq();
    htim1.Instance->ARR  = timer_arr;
    htim1.Instance->CCR1 = timer_arr / 2;

    htim4.Instance->ARR  = timer_arr;
    htim4.Instance->CCR1 = timer_arr / 2;
    __enable_irq();
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim6);
  __HAL_TIM_SET_COUNTER(&htim6, 0);
  last_loop_stamp_us = 0;

  HAL_Delay(100);
  MPU6050_Init();

  /* Initialize Motor Driver Pins */
  HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIR_R_GPIO_Port, DIR_R_Pin, GPIO_PIN_RESET);

  /* Default enable state at boot */
  HAL_GPIO_WritePin(EN_L_GPIO_Port, EN_L_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EN_R_GPIO_Port, EN_R_Pin, GPIO_PIN_RESET);

  /* Enable shadow registers */
  htim1.Instance->CR1 |= TIM_CR1_ARPE;
  htim4.Instance->CR1 |= TIM_CR1_ARPE;

  /* Sync counters */
  htim1.Instance->CNT = 0;
  htim4.Instance->CNT = 0;

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
        uint16_t now = __HAL_TIM_GET_COUNTER(&htim6);
        uint16_t elapsed = (uint16_t)(now - last_loop_stamp_us);

        if (elapsed >= 5000) { // 5ms Loop
            loop_period_us = elapsed;
            last_loop_stamp_us = now;

            uint16_t exec_start_us = __HAL_TIM_GET_COUNTER(&htim6);

            /* Battery ADC */
            HAL_ADC_Start(&hadc2);
            if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
                uint32_t raw_adc = HAL_ADC_GetValue(&hadc2);
                battery_voltage = ((float)raw_adc / 4095.0f) * 3.3f * DIVIDER_RATIO;
            }
            HAL_ADC_Stop(&hadc2);

            /* IMU Read */
            MPU6050_Read();

            float acc_pitch = -(atan2f((float)raw_acc_y, (float)raw_acc_z) * RAD_TO_DEG);
            float gyro_rate = -((float)raw_gyro_x / GYRO_SCALE);

            /* Sensor Fusion */
            update_complementary_filter(acc_pitch, gyro_rate);

            /* Mode handling */
            switch (test_mode) {
                case MODE_BALANCING:
                    HAL_GPIO_WritePin(EN_L_GPIO_Port, EN_L_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(EN_R_GPIO_Port, EN_R_Pin, GPIO_PIN_RESET);
                    calculate_PID(estimated_pitch, gyro_rate);
                    set_motor_speed(pid_output);
                    break;

                case MODE_STEP_RESPONSE:
                    HAL_GPIO_WritePin(EN_L_GPIO_Port, EN_L_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(EN_R_GPIO_Port, EN_R_Pin, GPIO_PIN_RESET);
                    test_timer++;
                    if (test_timer < 100) test_speed = 0.0f;
                    else if (test_timer < 1000) test_speed += 5.0f;
                    else { test_speed = 0.0f; test_timer = 0; }
                    set_motor_speed(test_speed);
                    pid_output = test_speed;
                    break;

                case MODE_FALL_TEST:
                    pid_output = 0.0f;
                    htim1.Instance->CCR1 = 0;
                    htim4.Instance->CCR1 = 0;
                    HAL_GPIO_WritePin(EN_L_GPIO_Port, EN_L_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(EN_R_GPIO_Port, EN_R_Pin, GPIO_PIN_SET);
                    break;

                default:
                    break;
            }

            loop_exec_time_us = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim6) - exec_start_us);

            // --- 1. HIGH-SPEED USB TELEMETRY (Every 5ms) ---
            static char tx_buffer[96];
            int len = snprintf(tx_buffer, sizeof(tx_buffer),
                               "$%u,%u,%.3f,%.3f,%.3f,%.3f,%.2f\n",
                               loop_period_us, loop_exec_time_us,
                               acc_pitch, gyro_rate, estimated_pitch, pid_output, battery_voltage);
            if (len > 0) {
                HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, len, 2);
            }

            // --- 2. BLUETOOTH TELEMETRY (Every 50ms) ---
            static uint8_t bt_counter = 0;
            if (++bt_counter >= 10) {
                bt_counter = 0;
                static char bt_buffer[48];
                // Using the compact format to ensure it fits in 1 BLE packet
                int bt_len = snprintf(bt_buffer, sizeof(bt_buffer),
                                      "P:%.0f,G:%.0f,M:%.0f,V:%.1f\n",
                                      estimated_pitch, gyro_rate, pid_output, battery_voltage);
                if (bt_len > 0) {
                    // Using 115200 Baud, this takes ~2ms. 10ms timeout is safe.
                    HAL_UART_Transmit(&huart1, (uint8_t *)bt_buffer, bt_len, 10);
                }
            }
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uart_tx_busy = 0;
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
