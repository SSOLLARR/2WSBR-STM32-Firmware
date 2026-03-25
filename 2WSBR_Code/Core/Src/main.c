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
#define MPU6050_ADDR (0x68 << 1)
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define ACCEL_XOUT_H_REG 0x3B

/* Operational Modes */
#define MODE_BALANCING 0
#define MODE_STEP_RESPONSE 1
#define MODE_SWEEP 2

/* Filter & Control Constants */
#define ALPHA 0.97f
#define DT 0.005f
#define RAD_TO_DEG 57.29577951f
#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE 131.0f
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
float Kp = 100.0f;
float Ki = 0.0f;
float Kd = 100.0f;

float target_angle = -1.35449028f;
float pid_error = 0.0f;
float previous_error = 5.0f;
float pid_integral = 0.0f;
float pid_output = 0.0f;

/* System States */
float battery_voltage = 0.0f;
float current_speed = 0.0f;
int test_mode = MODE_BALANCING;
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
void MPU6050_Init(void) {
    uint8_t data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
}

HAL_StatusTypeDef i2c_status;

void MPU6050_Read(void) {
    // 1. Attempt the read with a strict 2ms timeout
    i2c_status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rx_data, 14, 2);

    // 2. CHECK THE RETURN STATUS
    if (i2c_status == HAL_OK) {
        // Success! We have fresh, valid data. Safe to parse.
        raw_acc_x = (int16_t)(rx_data[0] << 8 | rx_data[1]);
        raw_acc_y = (int16_t)(rx_data[2] << 8 | rx_data[3]);
        raw_acc_z = (int16_t)(rx_data[4] << 8 | rx_data[5]);

        raw_gyro_x = (int16_t)(rx_data[8] << 8 | rx_data[9]);
        raw_gyro_y = (int16_t)(rx_data[10] << 8 | rx_data[11]);
        raw_gyro_z = (int16_t)(rx_data[12] << 8 | rx_data[13]);
    }
    else {
        // FAILURE! The sensor disconnected, timed out, or glitched.
        // Do NOT parse the data.

        // Safety feature: Kill the motors immediately so the robot
        // doesn't run away blindly using old PID data.
        htim1.Instance->CCR1 = 0;
        htim4.Instance->CCR1 = 0;
    }
}
float update_complementary_filter(float acc_pitch, float gyro_rate) {
    estimated_pitch = ALPHA * (estimated_pitch + gyro_rate * DT) + (1.0f - ALPHA) * acc_pitch;
    return estimated_pitch;
}

float calculate_PID(float current_pitch, float gyro_rate) {
    pid_error = current_pitch - target_angle;
    float P_out = Kp * pid_error;

    pid_integral += pid_error * DT;
    if (pid_integral > 400.0f) pid_integral = 400.0f;
    if (pid_integral < -400.0f) pid_integral = -400.0f;
    float I_out = Ki * pid_integral;

    float D_out = Kd * gyro_rate;

    pid_output = P_out + I_out + D_out;
    return pid_output;
}

void set_motor_speed(float target_speed) {
    // 1. Trap NaN or Infinity before they hit the hardware
    if (isnan(target_speed) || isinf(target_speed)) {
        htim1.Instance->CCR1 = 0;
        htim4.Instance->CCR1 = 0;
        return;
    }
    // CLAMP TARGET HERE, BEFORE THE LIMITER
        if (target_speed > 3000.0f) target_speed = 3000.0f;
        if (target_speed < -3000.0f) target_speed = -3000.0f;

        // --- 2. THE ADVANCED SLEW RATE LIMITER ---
            static float current_speed = 0.0f;

            // TUNE THESE TWO VARIABLES:
            float max_change_per_loop = 265.0f; // Push this as high as you can before stalling
            float instant_kick_speed = 500.0f; // The speed the motor can instantly jump to from 0

            // If the motor is currently stopped, allow an instant jump to the kick speed
            if (current_speed == 0.0f && target_speed > instant_kick_speed) {
                current_speed = instant_kick_speed;
            }
            else if (current_speed == 0.0f && target_speed < -instant_kick_speed) {
                current_speed = -instant_kick_speed;
            }
            // Otherwise, apply the normal acceleration ramp
            else {
                if (target_speed > current_speed + max_change_per_loop) {
                    current_speed += max_change_per_loop;
                }
                else if (target_speed < current_speed - max_change_per_loop) {
                    current_speed -= max_change_per_loop;
                }
                else {
                    current_speed = target_speed;
                }
            }

    // Use the smoothed speed for the physical hardware logic
    float applied_speed = current_speed;

    // --- 3. HARDWARE UPDATES ---
    /* Handle Direction States */
    if (applied_speed >= 0) {
        HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR_R_GPIO_Port, DIR_R_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR_R_GPIO_Port, DIR_R_Pin, GPIO_PIN_RESET);
        applied_speed = -applied_speed; // Make positive for timer math
    }

    /* Enforce deadzone to prevent audible motor hum at zero speed */
    if (applied_speed < 20.0f) {
        htim1.Instance->CCR1 = 0;
        htim4.Instance->CCR1 = 0;
        return;
    }

    /* Cap upper limit to prevent 16-bit timer overflow */
    // Note: I kept your original logic, but jumping from 3000 to 20000
    // is a massive leap! You might want to just clamp it to 3000.0f.
    if (applied_speed > 3000.0f) applied_speed = 3000.0f;

    /* Calculate timer Auto-Reload Register (ARR) for target frequency */
    // Assuming your timer clock is 1MHz (170MHz / 170 prescaler)
    uint32_t timer_arr = (uint32_t)(1000000.0f / applied_speed) - 1;

    /* Synchronize dual timer updates */
    __disable_irq();
    htim1.Instance->ARR = timer_arr;
    htim1.Instance->CCR1 = timer_arr / 2; // Keep 50% duty cycle

    htim4.Instance->ARR = timer_arr;
    htim4.Instance->CCR1 = timer_arr / 2; // Keep 50% duty cycle
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
  /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start(&htim6);
  __HAL_TIM_SET_COUNTER(&htim6, 0);
  last_loop_stamp_us = 0;
        /* Hardware Initialization Delay */
        HAL_Delay(100);
        MPU6050_Init();

        /* Initialize Motor Drivers */
        HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR_R_GPIO_Port, DIR_R_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(EN_L_GPIO_Port, EN_L_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(EN_R_GPIO_Port, EN_R_Pin, GPIO_PIN_RESET);

        // --- THE FIX: ENABLE SHADOW REGISTERS ---
        // This forces the timer to wait until the end of its current
        // cycle before applying the new speed, preventing the 65k wrap-around stall.
        htim1.Instance->CR1 |= TIM_CR1_ARPE;
        htim4.Instance->CR1 |= TIM_CR1_ARPE;

        // --- THE FIX: PERFECT SYNCHRONIZATION ---
        // Reset both counters to exactly zero before starting them,
        // so their PWM waves are perfectly aligned in time.
        htim1.Instance->CNT = 0;
        htim4.Instance->CNT = 0;



        /* Start PWM Channels */
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

        uint32_t last_loop_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
        while (1) {
        	uint16_t now = __HAL_TIM_GET_COUNTER(&htim6);
        	uint16_t elapsed = (uint16_t)(now - last_loop_stamp_us);

        	if (elapsed >= 5000) {
        	    loop_period_us = elapsed;
        	    last_loop_stamp_us = now;

        	    uint16_t exec_start_us = __HAL_TIM_GET_COUNTER(&htim6);

                /* 1. ACQUIRE SENSOR DATA */
                MPU6050_Read();

                float acc_pitch = -(atan2f((float)raw_acc_y, (float)raw_acc_z) * RAD_TO_DEG);
                float gyro_rate = -((float)raw_gyro_x / GYRO_SCALE);

                /* 2. SENSOR FUSION */
                update_complementary_filter(acc_pitch, gyro_rate);

                /* 3. EXECUTE CURRENT MODE */
                switch (test_mode) {
                    case MODE_BALANCING:
                        calculate_PID(estimated_pitch, gyro_rate);
                        set_motor_speed(pid_output);
                        break;

                    case MODE_STEP_RESPONSE:
                        // This test ramps speed every 0.5 seconds to find the stall point.
                        // Ensure the robot is on a stand (wheels off the ground) for the first run!

                        test_timer++;

                        if (test_timer < 100) {
                            test_speed = 0;          // Start at zero
                        }
                        else if (test_timer < 1000) { // 4.5 second ramp test
                            // Increment speed by 5 units every 5ms loop
                            // Result: Speed increases by 1000 units per second
                            test_speed += 5.0f;
                        }
                        else {
                            test_speed = 0;
                            test_timer = 0;          // Reset test
                        }

                        // Bypass the PID—send the test ramp directly to the motor logic
                        set_motor_speed(test_speed);

                        // Log test_speed as pid_output for the Telemetry graph
                        pid_output = test_speed;
                        break;

                    case MODE_SWEEP:
                    {
                        static int sweep_direction = 1;
                        float sweep_step = 2.0f;

                        if (sweep_direction == 1) {
                            test_speed += sweep_step;
                            if (test_speed >= 3000.0f) {
                                sweep_direction = -1;
                            }
                        } else {
                            test_speed -= sweep_step;
                            if (test_speed <= 0.0f) {
                                test_speed = 0.0f;
                                sweep_direction = 1;
                            }
                        }

                        set_motor_speed(test_speed);
                        pid_output = test_speed;
                        break;
                    }

                    default:
                        break;
                }

                loop_exec_time_us = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim6) - exec_start_us);

                // 1. Increase Baud Rate in CubeMX to 460800 or 921600
                // 2. Optimization: Use a smaller buffer and shorter float precision

                if (!uart_tx_busy) {
                    static char tx_buffer[96];

                    int len = snprintf(tx_buffer, sizeof(tx_buffer),
                                       "$%u,%u,%.3f,%.3f,%.3f,%.3f\n",
                                       loop_period_us, loop_exec_time_us,
                                       acc_pitch, gyro_rate, estimated_pitch, pid_output);

                    if (len > 0) {
                        HAL_StatusTypeDef tx_status =
                            HAL_UART_Transmit_DMA(&huart2, (uint8_t *)tx_buffer, len);

                        if (tx_status == HAL_OK) {
                            uart_tx_busy = 1;
                        } else {
                            uart_tx_busy = 0;
                        }
                    }
                }}
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
