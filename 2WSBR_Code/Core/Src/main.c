/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Hybrid cascade-control starting point
  *                   - keeps the clean dissertation-friendly structure
  *                   - keeps the useful parts of the Gemini draft
  *
  * Control structure:
  *   Outer loop : pitch angle theta_hat  -> q_ref
  *   Inner loop : pitch rate  gyro_rate  -> motor pulse rate (PPS)
  *
  * Notes:
  *   1) This is NOT a true wheel-velocity inner loop, because the current robot
  *      has IMU feedback and STEP/DIR stepper actuation but no measured wheel
  *      speed feedback.
  *   2) The pseudo-velocity supervisor from the Gemini draft has deliberately
  *      been removed, because it used filtered control effort as if it were a
  *      plant measurement. That is bad for black-box identification.
  *   3) An optional Gemini-style nonlinear outer-loop gain punch is retained,
  *      but set conservatively so it can be turned on later if needed.
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
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* MPU6050 registers */
#define MPU6050_ADDR         (0x68 << 1)
#define WHO_AM_I_REG         0x75
#define PWR_MGMT_1_REG       0x6B
#define CONFIG_REG           0x1A
#define GYRO_CONFIG_REG      0x1B
#define ACCEL_CONFIG_REG     0x1C
#define ACCEL_XOUT_H_REG     0x3B

/* Modes */
#define MODE_BALANCING       0
#define MODE_RATE_STEP       1   /* closed-loop inner-loop excitation */
#define MODE_MOTOR_STEP      2   /* open-loop motor excitation for ID */
#define MODE_FALL_TEST       3

/* Scaling */
#define RAD_TO_DEG           57.29577951f
#define GYRO_SCALE           131.0f
#define DIVIDER_RATIO        6.0f

/* Complementary filter */
#define CF_ALPHA             0.975f

/* Angle-loop limits */
#define THETA_REF_DEFAULT_DEG    0.0f
#define THETA_TRIM_DEFAULT_DEG   0.0f
#define Q_REF_LIMIT_DPS          120.0f

/* Motor / actuation limits */
#define MOTOR_CMD_LIMIT_PPS      3500.0f
#define MOTOR_MIN_PPS            15.5f
#define MOTOR_KICK_PPS           200.0f
#define MOTOR_SLEW_PPS_PER_S     30000.0f
#define MOTOR_SIGN               -1.0f   /* flip to -1.0f if the polarity is wrong */

/* Safety */
#define SAFE_ANGLE_LIMIT_DEG     8.0f
#define FALL_TIME_LIMIT_MS       1500U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Raw IMU buffer */
uint8_t rx_data[14];
int16_t raw_acc_x, raw_acc_y, raw_acc_z;
int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;

/* Sensor / estimated states */
float acc_pitch_deg = 0.0f;
float gyro_rate_dps = 0.0f;
float estimated_pitch_deg = 0.0f;
static float gyro_bias_x_dps = 0.0f;
static float pitch_zero_deg  = 0.0f;

/* Outer loop: theta -> q_ref */
float angle_Kp         = 7.0f;
float angle_Ki         = 2.00f;
float angle_aggression = 0.00f;
float angle_integral   = 0.0f;
float theta_ref_deg    = THETA_REF_DEFAULT_DEG;
float theta_trim_deg   = THETA_TRIM_DEFAULT_DEG;
float q_ref_dps        = 0.0f;

/* Inner loop: q -> motor PPS */
float rate_Kp       = 37.0f;
float rate_Ki       = 15.0f;
float rate_integral = 0.0f;
float motor_cmd_pps = 0.0f;  /* controller output before actuator shaping */
float motor_applied_pps = 0.0f; /* actual post-limit / post-slew plant input */
static float motor_slewed_pps = 0.0f;

/* Mode and test signals */
int test_mode = MODE_BALANCING;
uint32_t test_timer = 0;
float test_rate_ref_dps = 0.0f;
float test_motor_pps    = 0.0f;

/* Fall detection */
uint32_t fall_timer_ms = 0;
uint8_t  is_falling = 0;

/* Battery / comms / timing */
float battery_voltage = 0.0f;
HAL_StatusTypeDef i2c_status;
static volatile uint8_t uart_tx_busy = 0;
uint16_t loop_exec_time_us = 0;
uint16_t loop_period_us = 0;
uint16_t last_loop_stamp_us = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static float clampf(float x, float lo, float hi);
static void reset_controllers(void);
static void motor_outputs_enable(void);
static void motor_outputs_disable(void);
static void MPU6050_WriteReg(uint8_t reg, uint8_t value);
static uint8_t MPU6050_ReadReg(uint8_t reg);
static void MPU6050_Init(void);
static void MPU6050_Read(void);
static void IMU_CalibrateAtRest(void);
static float update_complementary_filter(float acc_pitch, float gyro_rate, float dt);
static float angle_outer_loop(float theta_ref, float theta_hat, float dt);
static float rate_inner_loop(float q_ref, float q_hat, float dt);
static void set_motor_speed(float target_speed_pps, float dt);
static void process_bluetooth_command(const char *cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void reset_controllers(void)
{
    angle_integral = 0.0f;
    rate_integral  = 0.0f;
    q_ref_dps      = 0.0f;
    motor_cmd_pps  = 0.0f;
}

static void motor_outputs_enable(void)
{
    HAL_GPIO_WritePin(EN_L_GPIO_Port, EN_L_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_R_GPIO_Port, EN_R_Pin, GPIO_PIN_RESET);
}

static void motor_outputs_disable(void)
{
    htim1.Instance->CCR1 = 0;
    htim4.Instance->CCR1 = 0;
    HAL_GPIO_WritePin(EN_L_GPIO_Port, EN_L_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_R_GPIO_Port, EN_R_Pin, GPIO_PIN_SET);
    motor_applied_pps = 0.0f;
    motor_slewed_pps  = 0.0f;
}

static void MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, 1, &value, 1, 100);
}

static uint8_t MPU6050_ReadReg(uint8_t reg)
{
    uint8_t value = 0;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, 1, &value, 1, 100);
    return value;
}

static void MPU6050_Init(void)
{
    /* Wake sensor */
    MPU6050_WriteReg(PWR_MGMT_1_REG, 0x00);
    HAL_Delay(50);

    /* Enable DLPF -> gyro bandwidth about 42 Hz */
    MPU6050_WriteReg(CONFIG_REG, 0x03);

    /* Gyro = ±250 dps, accel = ±2g */
    MPU6050_WriteReg(GYRO_CONFIG_REG, 0x00);
    MPU6050_WriteReg(ACCEL_CONFIG_REG, 0x00);

    (void)MPU6050_ReadReg(WHO_AM_I_REG);
}

static void MPU6050_Read(void)
{
    i2c_status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rx_data, 14, 2);

    if (i2c_status == HAL_OK)
    {
        raw_acc_x  = (int16_t)((rx_data[0]  << 8) | rx_data[1]);
        raw_acc_y  = (int16_t)((rx_data[2]  << 8) | rx_data[3]);
        raw_acc_z  = (int16_t)((rx_data[4]  << 8) | rx_data[5]);

        raw_gyro_x = (int16_t)((rx_data[8]  << 8) | rx_data[9]);
        raw_gyro_y = (int16_t)((rx_data[10] << 8) | rx_data[11]);
        raw_gyro_z = (int16_t)((rx_data[12] << 8) | rx_data[13]);
    }
    else
    {
        motor_outputs_disable();
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
        HAL_Delay(2);
    }

    gyro_bias_x_dps = gyro_sum / (float)N;
    pitch_zero_deg  = acc_sum / (float)N;
    estimated_pitch_deg = 0.0f;
}

static float update_complementary_filter(float acc_pitch, float gyro_rate, float dt)
{
    estimated_pitch_deg = (CF_ALPHA * (estimated_pitch_deg + gyro_rate * dt))
                        + ((1.0f - CF_ALPHA) * acc_pitch);
    return estimated_pitch_deg;
}

static float angle_outer_loop(float theta_ref, float theta_hat, float dt)
{
    const float theta_err = theta_ref - theta_hat;
    const float dynamic_Kp = angle_Kp + (angle_aggression * fabsf(theta_err));
    float unsat_q_ref;
    float sat_q_ref;

    /* Integrate only near the operating region */
    if (fabsf(theta_err) < 6.0f)
    {
        angle_integral += theta_err * dt;
        angle_integral = clampf(angle_integral, -40.0f, 40.0f);
    }
    else
    {
        angle_integral *= 0.98f;
    }

    unsat_q_ref = (dynamic_Kp * theta_err) + (angle_Ki * angle_integral);
    sat_q_ref   = clampf(unsat_q_ref, -Q_REF_LIMIT_DPS, Q_REF_LIMIT_DPS);

    /* Simple anti-windup backout */
    if (unsat_q_ref != sat_q_ref)
    {
        angle_integral *= 0.995f;
    }

    return sat_q_ref;
}

static float rate_inner_loop(float q_ref, float q_hat, float dt)
{
    const float q_err = q_ref - q_hat;
    float unsat_u;
    float sat_u;

    rate_integral += q_err * dt;
    rate_integral = clampf(rate_integral, -600.0f, 600.0f);

    unsat_u = (rate_Kp * q_err) + (rate_Ki * rate_integral);
    sat_u   = clampf(unsat_u, -MOTOR_CMD_LIMIT_PPS, MOTOR_CMD_LIMIT_PPS);

    if (unsat_u != sat_u)
    {
        rate_integral *= 0.995f;
    }

    return sat_u;
}

static void set_motor_speed(float target_speed_pps, float dt)
{
    float max_delta_pps;
    float applied;
    uint32_t timer_arr;

    if (isnan(target_speed_pps) || isinf(target_speed_pps))
    {
        htim1.Instance->CCR1 = 0;
        htim4.Instance->CCR1 = 0;
        motor_applied_pps = 0.0f;
        return;
    }

    target_speed_pps = clampf(target_speed_pps, -MOTOR_CMD_LIMIT_PPS, MOTOR_CMD_LIMIT_PPS);
    max_delta_pps = MOTOR_SLEW_PPS_PER_S * dt;

    /* Software ramping is essential for steppers */
    if ((motor_slewed_pps == 0.0f) && (target_speed_pps > MOTOR_KICK_PPS))
    {
        motor_slewed_pps = MOTOR_KICK_PPS;
    }
    else if ((motor_slewed_pps == 0.0f) && (target_speed_pps < -MOTOR_KICK_PPS))
    {
        motor_slewed_pps = -MOTOR_KICK_PPS;
    }
    else
    {
        if (target_speed_pps > motor_slewed_pps + max_delta_pps)
        {
            motor_slewed_pps += max_delta_pps;
        }
        else if (target_speed_pps < motor_slewed_pps - max_delta_pps)
        {
            motor_slewed_pps -= max_delta_pps;
        }
        else
        {
            motor_slewed_pps = target_speed_pps;
        }
    }

    motor_applied_pps = motor_slewed_pps;
    applied = MOTOR_SIGN * motor_slewed_pps;

    if (applied >= 0.0f)
    {
        HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR_R_GPIO_Port, DIR_R_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR_R_GPIO_Port, DIR_R_Pin, GPIO_PIN_RESET);
        applied = -applied;
    }

    if (applied < MOTOR_MIN_PPS)
    {
        htim1.Instance->CCR1 = 0;
        htim4.Instance->CCR1 = 0;
        motor_applied_pps = 0.0f;
        return;
    }

    timer_arr = (uint32_t)(1000000.0f / applied) - 1U;
    if (timer_arr > 65535U)
    {
        timer_arr = 65535U;
    }

    __disable_irq();
    htim1.Instance->ARR  = timer_arr;
    htim1.Instance->CCR1 = timer_arr / 2U;
    htim4.Instance->ARR  = timer_arr;
    htim4.Instance->CCR1 = timer_arr / 2U;
    __enable_irq();
}

static void process_bluetooth_command(const char *cmd)
{
    /* Explicit two-letter commands */
    if (strncmp(cmd, "OP", 2) == 0)
    {
        angle_Kp = atof(&cmd[2]);
        angle_integral = 0.0f;
    }
    else if (strncmp(cmd, "OI", 2) == 0)
    {
        angle_Ki = atof(&cmd[2]);
        angle_integral = 0.0f;
    }
    else if (strncmp(cmd, "OG", 2) == 0)
    {
        angle_aggression = atof(&cmd[2]);
    }
    else if (strncmp(cmd, "IP", 2) == 0)
    {
        rate_Kp = atof(&cmd[2]);
        rate_integral = 0.0f;
    }
    else if (strncmp(cmd, "II", 2) == 0)
    {
        rate_Ki = atof(&cmd[2]);
        rate_integral = 0.0f;
    }
    else if (strncmp(cmd, "RE", 2) == 0)
    {
        theta_ref_deg = atof(&cmd[2]);
    }
    else if (strncmp(cmd, "TR", 2) == 0)
    {
        theta_trim_deg = atof(&cmd[2]);
    }
    else if (strncmp(cmd, "MO", 2) == 0)
    {
        int requested_mode = atoi(&cmd[2]);
        if ((requested_mode >= MODE_BALANCING) && (requested_mode <= MODE_FALL_TEST))
        {
            test_mode = requested_mode;
            test_timer = 0U;
            reset_controllers();
        }
    }
    else if (strcmp(cmd, "ZE") == 0)
    {
        reset_controllers();
    }

    else if (cmd[0] == 'P')
    {
        angle_Kp = atof(&cmd[1]);
        angle_integral = 0.0f;
    }
    else if (cmd[0] == 'I')
    {
        angle_Ki = atof(&cmd[1]);
        angle_integral = 0.0f;
    }
    else if (cmd[0] == 'A')
    {
        angle_aggression = atof(&cmd[1]);
    }
    else if (cmd[0] == 'p')
    {
        rate_Kp = atof(&cmd[1]);
        rate_integral = 0.0f;
    }
    else if (cmd[0] == 'i')
    {
        rate_Ki = atof(&cmd[1]);
        rate_integral = 0.0f;
    }
    else if (cmd[0] == 'R')
    {
        theta_ref_deg = atof(&cmd[1]);
    }
    else if (cmd[0] == 'T')
    {
        theta_trim_deg = atof(&cmd[1]);
    }
    else if (cmd[0] == 'M')
    {
        int requested_mode = atoi(&cmd[1]);
        if ((requested_mode >= MODE_BALANCING) && (requested_mode <= MODE_FALL_TEST))
        {
            test_mode = requested_mode;
            test_timer = 0U;
            reset_controllers();
        }
    }
    else if (cmd[0] == 'Z')
    {
        reset_controllers();
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
  HAL_Init();
  SystemClock_Config();

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

  /* Default motor direction */
  HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIR_R_GPIO_Port, DIR_R_Pin, GPIO_PIN_RESET);

  /* Enable drivers by default */
  motor_outputs_enable();

  /* Enable shadow registers */
  htim1.Instance->CR1 |= TIM_CR1_ARPE;
  htim4.Instance->CR1 |= TIM_CR1_ARPE;

  /* Sync counters */
  htim1.Instance->CNT = 0;
  htim4.Instance->CNT = 0;

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  static uint8_t batt_div = 0;
  static char bt_cmd_buf[32];
  static uint8_t bt_cmd_idx = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint16_t now = __HAL_TIM_GET_COUNTER(&htim6);
      uint16_t elapsed = (uint16_t)(now - last_loop_stamp_us);

      if (elapsed >= 5000U)
      {
          const float dt = elapsed * 1.0e-6f;
          const float theta_command_deg = theta_ref_deg + theta_trim_deg;
          uint16_t exec_start_us;

          loop_period_us = elapsed;
          last_loop_stamp_us = now;
          exec_start_us = __HAL_TIM_GET_COUNTER(&htim6);

          /* Battery ADC */
          HAL_ADC_Start(&hadc2);
          if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK)
          {
              const uint32_t raw_adc = HAL_ADC_GetValue(&hadc2);
              battery_voltage = ((float)raw_adc / 4095.0f) * 3.3f * DIVIDER_RATIO;
          }
          HAL_ADC_Stop(&hadc2);

          /* IMU */
          MPU6050_Read();
          acc_pitch_deg = -(atan2f((float)raw_acc_y, (float)raw_acc_z) * RAD_TO_DEG) - pitch_zero_deg;
          gyro_rate_dps = -((float)raw_gyro_x / GYRO_SCALE) - gyro_bias_x_dps;
          update_complementary_filter(acc_pitch_deg, gyro_rate_dps, dt);

          /* Fall detection */
          if (fabsf(estimated_pitch_deg) > SAFE_ANGLE_LIMIT_DEG)
          {
              fall_timer_ms += (uint32_t)(dt * 1000.0f);
              if (fall_timer_ms >= FALL_TIME_LIMIT_MS)
              {
                  is_falling = 1U;
              }
          }
          else
          {
              fall_timer_ms = 0U;
              is_falling = 0U;
          }

          switch (test_mode)
          {
              case MODE_BALANCING:
              {
                  if (is_falling)
                  {
                      motor_outputs_disable();
                      reset_controllers();
                  }
                  else
                  {
                      motor_outputs_enable();
                      q_ref_dps = angle_outer_loop(theta_command_deg, estimated_pitch_deg, dt);
                      motor_cmd_pps = rate_inner_loop(q_ref_dps, gyro_rate_dps, dt);
                      set_motor_speed(motor_cmd_pps, dt);
                  }
                  break;
              }

              case MODE_RATE_STEP:
              {
                  /* Closed-loop inner-loop identification via q_ref step */
                  motor_outputs_enable();
                  test_timer++;

                  if (test_timer < 200U)         test_rate_ref_dps = 0.0f;
                  else if (test_timer < 600U)    test_rate_ref_dps = 35.0f;
                  else if (test_timer < 1000U)   test_rate_ref_dps = -35.0f;
                  else
                  {
                      test_timer = 0U;
                      test_rate_ref_dps = 0.0f;
                  }

                  q_ref_dps = clampf(test_rate_ref_dps, -Q_REF_LIMIT_DPS, Q_REF_LIMIT_DPS);
                  motor_cmd_pps = rate_inner_loop(q_ref_dps, gyro_rate_dps, dt);
                  set_motor_speed(motor_cmd_pps, dt);
                  break;
              }

              case MODE_MOTOR_STEP:
              {
                  /* Open-loop single step for plant identification.
                     Phase 1 (0-499ms): motors off, robot settles upright.
                     Phase 2 (500ms+):  fixed step, holds until you catch it.
                     Power cycle to repeat. */
                  motor_outputs_enable();
                  reset_controllers();
                  test_timer++;

                  if (test_timer < 100U)
                  {
                      /* Phase 1: settle */
                      test_motor_pps = 0.0f;
                  }
                  else
                  {
                      /* Phase 2: step and hold */
                      test_motor_pps = 450.0f;
                  }

                  q_ref_dps     = 0.0f;
                  motor_cmd_pps = test_motor_pps;
                  set_motor_speed(motor_cmd_pps, dt);
                  break;
              }

              case MODE_FALL_TEST:
              default:
              {
                  motor_outputs_disable();
                  reset_controllers();
                  break;
              }
          }

          loop_exec_time_us = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim6) - exec_start_us);

          /* =========================================================
             USART2 telemetry for MATLAB
             A packet every 5 ms:
             A,acc_pitch,gyro_rate,theta_hat,theta_cmd,q_ref,u_cmd,u_applied,mode
             ========================================================= */
          {
              char usbA[160];
              const int len = snprintf(usbA, sizeof(usbA),
                                       "A,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n",
                                       acc_pitch_deg,
                                       gyro_rate_dps,
                                       estimated_pitch_deg,
                                       theta_command_deg,
                                       q_ref_dps,
                                       motor_cmd_pps,
                                       motor_applied_pps,
                                       test_mode);
              if (len > 0)
              {
                  HAL_UART_Transmit(&huart2, (uint8_t *)usbA, (uint16_t)len, 2);
              }
          }

          {
              char usbB[48];
              const int len = snprintf(usbB, sizeof(usbB),
                                       "B,%u,%u\n",
                                       loop_period_us,
                                       loop_exec_time_us);
              if (len > 0)
              {
                  HAL_UART_Transmit(&huart2, (uint8_t *)usbB, (uint16_t)len, 2);
              }
          }

          if (++batt_div >= 10U)
          {
              batt_div = 0U;
              char usbC[48];
              const int len = snprintf(usbC, sizeof(usbC),
                                       "C,%.2f,%u,%u\n",
                                       battery_voltage,
                                       fall_timer_ms,
                                       (unsigned)is_falling);
              if (len > 0)
              {
                  HAL_UART_Transmit(&huart2, (uint8_t *)usbC, (uint16_t)len, 2);
              }
          }

          /* Bluetooth commands on USART1 */
          {
              uint8_t ch;
              while (HAL_UART_Receive(&huart1, &ch, 1, 0) == HAL_OK)
              {
                  if ((ch == '\n') || (ch == '\r'))
                  {
                      bt_cmd_buf[bt_cmd_idx] = '\0';
                      if (bt_cmd_idx > 0U)
                      {
                          process_bluetooth_command(bt_cmd_buf);
                      }
                      bt_cmd_idx = 0U;
                  }
                  else if (bt_cmd_idx < (sizeof(bt_cmd_buf) - 1U))
                  {
                      bt_cmd_buf[bt_cmd_idx++] = (char)ch;
                  }
                  else
                  {
                      bt_cmd_idx = 0U;
                  }
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

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

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
    if (huart->Instance == USART2)
    {
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
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
