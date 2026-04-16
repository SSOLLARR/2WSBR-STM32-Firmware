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
#include <stdlib.h>


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
#define ALPHA           0.975f   // Optimized to prevent scale-factor drift leaning
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
static float gyro_bias_x = 0.0f;
static float pitch_zero  = 0.0f;

/* Inner Loop: Pitch PID Parameters */
float Kp = 800.0f;
float Ki = 2.5f;
float Kd = 1000.0f;

float pid_error = 0.0f;
float previous_error = 0.0f;
float pid_integral = 0.0f;
float pid_output = 0.0f;

/* Outer Loop: Velocity Control Parameters */
float target_angle = 0.0f;
float speed_integral = 0.0f;
float speed_Kp = 0.0001f;
float speed_Ki = 0.0004f;
static float chassis_velocity = 0.0f; // Low-pass filtered motor output

/* Fall Detection States */
#define SAFE_ANGLE_LIMIT    8.0f   // Degrees: Beyond this, we consider it a fall
#define FALL_TIME_LIMIT_MS  1500   // 1.5 seconds
uint32_t fall_timer_ms = 0;
uint8_t  is_falling = 0;


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

static void IMU_CalibrateAtRest(void)
{
    const int N = 500;
    float gyro_sum = 0.0f;
    float acc_sum  = 0.0f;

    for (int i = 0; i < N; i++)
    {
        MPU6050_Read();
        gyro_sum += -((float)raw_gyro_x / GYRO_SCALE);
        acc_sum  += -(atan2f((float)raw_acc_y, (float)raw_acc_z) * RAD_TO_DEG);
        HAL_Delay(2); // Wait 2ms between reads
    }

    gyro_bias_x = gyro_sum / N;
    pitch_zero  = acc_sum / N;

    /* The controller now sees this exact resting position as 0.0 degrees */
    estimated_pitch = 0.0f;
}

float update_complementary_filter(float acc_pitch, float gyro_rate, float dynamic_dt)
{
    estimated_pitch = ALPHA * (estimated_pitch + gyro_rate * dynamic_dt) + (1.0f - ALPHA) * acc_pitch;
    return estimated_pitch;
}

float calculate_PID(float current_pitch, float gyro_rate, float dynamic_dt)
{
    pid_error = current_pitch - target_angle;

    /* --- NON-LINEAR PROPORTIONAL TERM --- */
    // As error grows, Kp grows exponentially to violently catch dives
    float aggression_multiplier = 150.0f;
    float dynamic_Kp = Kp + (fabs(pid_error) * aggression_multiplier);

    float P_out = dynamic_Kp * pid_error;

    /* 1. Integral */
    pid_integral += pid_error * dynamic_dt;
    if (pid_integral > 400.0f)  pid_integral = 400.0f;
    if (pid_integral < -400.0f) pid_integral = -400.0f;
    float I_out = Ki * pid_integral;

    /* 2. Derivative */
    static float D_filtered = 0.0f;
    float D_raw = Kd * gyro_rate;
    const float beta = 0.4f;
    D_filtered = (beta * D_filtered) + ((1.0f - beta) * D_raw);

    pid_output = P_out + I_out + D_filtered;
    return pid_output;
}
void set_motor_speed(float target_speed)
{
    if (isnan(target_speed) || isinf(target_speed)) {
        htim1.Instance->CCR1 = 0;
        htim4.Instance->CCR1 = 0;
        return;
    }

    float compensated_speed = target_speed;

    /* Clamp absolute limits */
    if (compensated_speed > 5000.0f)  compensated_speed = 5000.0f;
    if (compensated_speed < -5000.0f) compensated_speed = -5000.0f;

    /* --- 1. SLEW RATE LIMITER --- */
    static float slewed_speed = 0.0f;
    float max_change_per_loop = 150.0f;
    float instant_kick_speed  = 200.0f;

    if (slewed_speed == 0.0f && compensated_speed > instant_kick_speed) {
        slewed_speed = instant_kick_speed;
    } else if (slewed_speed == 0.0f && compensated_speed < -instant_kick_speed) {
        slewed_speed = -instant_kick_speed;
    } else {
        if (compensated_speed > slewed_speed + max_change_per_loop) {
            slewed_speed += max_change_per_loop;
        } else if (compensated_speed < slewed_speed - max_change_per_loop) {
            slewed_speed -= max_change_per_loop;
        } else {
            slewed_speed = compensated_speed;
        }
    }

    float applied_speed = slewed_speed;

    /* --- 2. DIRECTION & HARDWARE TIMERS --- */
    if (applied_speed >= 0) {
        HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR_R_GPIO_Port, DIR_R_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR_R_GPIO_Port, DIR_R_Pin, GPIO_PIN_RESET);
        applied_speed = -applied_speed; // Absolute value for timer calculation
    }

    /* --- 3. CRITICAL 16-BIT TIMER CUTOFF ---
       The 16-bit timer physically CANNOT step slower than 15.26 Hz.
       We cut power cleanly below 15.5 Hz to allow the robot to stand still
       and mathematically prevent an 884 Hz integer overflow spike. */
    if (applied_speed < 15.5f) {
        htim1.Instance->CCR1 = 0;
        htim4.Instance->CCR1 = 0;
        return;
    }

    uint32_t timer_arr = (uint32_t)(1000000.0f / applied_speed) - 1;

    // Failsafe Guard against any residual overflows
    if (timer_arr > 65535) {
        timer_arr = 65535;
    }

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
  IMU_CalibrateAtRest();

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

  // Telemetry Priority
  static uint8_t batt_div = 0;   // every 10 loops = 50 ms
  static char bt_cmd_buf[32];
  static uint8_t bt_cmd_idx = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint16_t now = __HAL_TIM_GET_COUNTER(&htim6);
      uint16_t elapsed = (uint16_t)(now - last_loop_stamp_us);

      if (elapsed >= 5000)   // 5 ms loop
      {
          loop_period_us = elapsed;
          last_loop_stamp_us = now;

          uint16_t exec_start_us = __HAL_TIM_GET_COUNTER(&htim6);

          /* --- Battery ADC --- */
          HAL_ADC_Start(&hadc2);
          if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK)
          {
              uint32_t raw_adc = HAL_ADC_GetValue(&hadc2);
              battery_voltage = ((float)raw_adc / 4095.0f) * 3.3f * DIVIDER_RATIO;
          }
          HAL_ADC_Stop(&hadc2);

          /* --- IMU Read --- */
	      MPU6050_Read();

          // Calculate dynamic dt based on the actual hardware timer
          float dynamic_dt = loop_period_us * 1e-6f;

          // Subtract the offsets calculated during startup
          float acc_pitch = -(atan2f((float)raw_acc_y, (float)raw_acc_z) * RAD_TO_DEG) - pitch_zero;
          float gyro_rate = -((float)raw_gyro_x / GYRO_SCALE) - gyro_bias_x;

          /* --- Sensor Fusion --- */
          update_complementary_filter(acc_pitch, gyro_rate, dynamic_dt);

          /* --- Mode Handling --- */
          switch (test_mode)
          {
          case MODE_BALANCING:
                        {
                            /* --- FALL DETECTION LOGIC --- */
                            // If the robot leans past the safe limit, start the timer
                            if (fabs(estimated_pitch) > SAFE_ANGLE_LIMIT) {
                                fall_timer_ms += (uint32_t)(dynamic_dt * 1000.0f); // Add ~5ms
                            } else {
                                // If it recovers into the safe zone, reset the timer
                                fall_timer_ms = 0;
                                is_falling = 0;
                            }

                            // If it has been falling for longer than the limit, trigger the failsafe
                            if (fall_timer_ms >= FALL_TIME_LIMIT_MS) {
                                is_falling = 1;
                            }

                            /* --- BEHAVIOR --- */
                            if (is_falling == 1) {
                                // EMERGENCY STATE: Arrest the dive or kill power

                                // Option A: Kill Power (Drop to the floor safely)
                                pid_output = 0.0f;
                                htim1.Instance->CCR1 = 0;
                                htim4.Instance->CCR1 = 0;
                                HAL_GPIO_WritePin(EN_L_GPIO_Port, EN_L_Pin, GPIO_PIN_SET);
                                HAL_GPIO_WritePin(EN_R_GPIO_Port, EN_R_Pin, GPIO_PIN_SET);

                                // (Optional) Automatically reset to balancing mode if you pick it back up
                                // if (fabs(estimated_pitch) < 1.0f) {
                                //     fall_timer_ms = 0;
                                //     is_falling = 0;
                                //     // Reset integral accumulators so it doesn't instantly jump
                                //     pid_integral = 0;
                                //     speed_integral = 0;
                                // }

                            } else {
                                // NORMAL BALANCING STATE
                                HAL_GPIO_WritePin(EN_L_GPIO_Port, EN_L_Pin, GPIO_PIN_RESET);
                                HAL_GPIO_WritePin(EN_R_GPIO_Port, EN_R_Pin, GPIO_PIN_RESET);

                                /* --- 1. OUTER LOOP: Velocity Cascade Control --- */
                                chassis_velocity = (0.85f * chassis_velocity) + (0.15f * pid_output);

                                float target_speed = 0.0f;
                                float speed_error = target_speed - chassis_velocity;

                                speed_integral += speed_error * dynamic_dt;

                                if (speed_integral > 8000.0f)  speed_integral = 8000.0f;
                                if (speed_integral < -8000.0f) speed_integral = -8000.0f;

                                target_angle = (speed_error * speed_Kp) + (speed_integral * speed_Ki);

                                // Outer loop authority clamp
                                if (target_angle > 1.7f)  target_angle = 1.7f;
                                if (target_angle < -1.7f) target_angle = -1.7f;

                                /* --- 2. INNER LOOP: Pitch Control --- */
                                calculate_PID(estimated_pitch, gyro_rate, dynamic_dt);
                                set_motor_speed(pid_output);
                            }
                            break;
                        }

              case MODE_STEP_RESPONSE:
              {
                  HAL_GPIO_WritePin(EN_L_GPIO_Port, EN_L_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(EN_R_GPIO_Port, EN_R_Pin, GPIO_PIN_RESET);

                  test_timer++;
                  if (test_timer < 100)          test_speed = 0.0f;
                  else if (test_timer < 1000)    test_speed += 5.0f;
                  else
                  {
                      test_speed = 0.0f;
                      test_timer = 0;
                  }

                  set_motor_speed(test_speed);
                  pid_output = test_speed;
                  break;
              }

              case MODE_FALL_TEST:
              {
                  pid_output = 0.0f;
                  htim1.Instance->CCR1 = 0;
                  htim4.Instance->CCR1 = 0;
                  HAL_GPIO_WritePin(EN_L_GPIO_Port, EN_L_Pin, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(EN_R_GPIO_Port, EN_R_Pin, GPIO_PIN_SET);
                  break;
              }

              default:
                  break;
          }

          loop_exec_time_us = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim6) - exec_start_us);

          /* =========================================================
             USB / MATLAB telemetry on USART2 @ 460800
             ========================================================= */

          /* A packet: every 5 ms (critical) */
          {
              char usbA[64];
              int len = snprintf(usbA, sizeof(usbA),
                                 "A,%.3f,%.3f,%.3f,%.3f\n",
                                 acc_pitch, gyro_rate, estimated_pitch, pid_output);
              if (len > 0)
              {
                  HAL_UART_Transmit(&huart2, (uint8_t *)usbA, len, 2);
              }
          }

          /* B packet: every 5 ms (timing) */
          {
              char usbB[32];
              int len = snprintf(usbB, sizeof(usbB),
                                 "B,%u,%u\n",
                                 loop_period_us, loop_exec_time_us);
              if (len > 0)
              {
                  HAL_UART_Transmit(&huart2, (uint8_t *)usbB, len, 2);
              }
          }

          /* C packet: every 50 ms (battery) */
          if (++batt_div >= 10)
          {
              batt_div = 0;

              char usbC[20];
              int len = snprintf(usbC, sizeof(usbC),
                                 "C,%.2f\n",
                                 battery_voltage);
              if (len > 0)
              {
                  HAL_UART_Transmit(&huart2, (uint8_t *)usbC, len, 2);
              }
          }

          /* =========================================================
             USART1 (Bluetooth Tuning)
             ========================================================= */

          uint8_t ch;

          /* Drain ALL available characters from the UART RX register */
          while (HAL_UART_Receive(&huart1, &ch, 1, 0) == HAL_OK)
          {
              if (ch == '\n' || ch == '\r')
              {
                  bt_cmd_buf[bt_cmd_idx] = '\0'; // Null terminate

                  if (bt_cmd_buf[0] == 'P') {
                      Kp = atof(&bt_cmd_buf[1]);
                      pid_integral = 0.0f; // Reset integral to prevent violent jumps
                  }
                  else if (bt_cmd_buf[0] == 'I') {
                      Ki = atof(&bt_cmd_buf[1]);
                      pid_integral = 0.0f;
                  }
                  else if (bt_cmd_buf[0] == 'D') {
                      Kd = atof(&bt_cmd_buf[1]);
                  }

                  bt_cmd_idx = 0; // Reset index for next command
              }
              else if (bt_cmd_idx < sizeof(bt_cmd_buf) - 1)
              {
                  bt_cmd_buf[bt_cmd_idx++] = (char)ch;
              }
              else
              {
                  bt_cmd_idx = 0; // Buffer overflow protection, reset
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
