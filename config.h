#pragma once
#include <stdint.h>

#define PIN_SDA            21
#define PIN_SCL            22
#define I2C_FREQ           400000

enum { TOF_L = 0, TOF_F, TOF_R, TOF_COUNT };
static const uint8_t PIN_XSHUT[TOF_COUNT] = { 16, 18, 23 };
static const uint8_t TOF_ADDR[TOF_COUNT]  = { 0x10, 0x12, 0x14 };

#define PIN_ML_IN1         25
#define PIN_ML_IN2         26
#define PIN_MR_IN1         27
#define PIN_MR_IN2         14
#define MOTOR_L_INVERT     0
#define MOTOR_R_INVERT     1

#define PIN_START          34
#define PIN_KILL           35
#define USE_KILL_PIN       1

#define USE_ENCODERS       1
#define PIN_ENC_L_A        32
#define PIN_ENC_L_B        33
#define PIN_ENC_R_A        4
#define PIN_ENC_R_B        13
#define ENC_L_INVERT       0
#define ENC_R_INVERT       1

#define USE_VBAT           1
#define PIN_VBAT           36
#define VBAT_DIV           ((100.0f + 47.0f) / 47.0f)
#define VBAT_NOMINAL       7.6f

#define PIN_LED            2
#define PIN_BUTTON         0

#define TEST_START_BUTTON  0

#define CELL_MM                300.0f
#define SIDE_SENSOR_OFFSET_MM   38.0f
#define SIDE_SENSOR_X_MM        30.0f
#define FRONT_SENSOR_X_MM       45.0f
#define WHEEL_DIAMETER_MM       34.0f
#define WHEEL_BASE_MM           78.0f
#define ENC_PPR                 7.0f
#define GEAR_RATIO              50.0f
#define ENC_COUNTS_PER_REV      (ENC_PPR * 4.0f * GEAR_RATIO)

#define SIDE_TARGET_MM      (CELL_MM * 0.5f - SIDE_SENSOR_OFFSET_MM)
#define WALL_PRESENT_MM     (SIDE_TARGET_MM + 75.0f)
#define OPEN_MM             (SIDE_TARGET_MM + 115.0f)
#define FRONT_TURN_MM       (CELL_MM * 0.5f - FRONT_SENSOR_X_MM)
#define FRONT_SLOW_MM       420.0f
#define OPEN_DETECT_LAG_MM  25.0f
#define ADVANCE_TO_TURN_MM  (CELL_MM * 0.5f + SIDE_SENSOR_X_MM - OPEN_DETECT_LAG_MM)
#define ENTER_AFTER_TURN_MM (CELL_MM * 0.5f - SIDE_SENSOR_X_MM + 20.0f)
#define ENTER_SHORT_MM      50.0f

#define MOTOR_MAX_DUTY      0.85f
#define MOTOR_DEADBAND      0.07f
#define V_CRUISE            0.80f
#define V_MIN               0.28f
#define V_ADVANCE           0.55f
#define V_TURN              0.60f
#define V_TURN_MIN          0.25f
#define V_EXIT              0.70f
#define ACCEL_PER_S         7.0f
#define PWM_FREQ            20000
#define PWM_BITS            10

#define KP_LAT              0.0045f
#define KP_SLOPE            0.50f
#define SLOPE_MIN_MM        8.0f
#define SLOPE_FILT          0.5f
#define K_ENC_HOLD          0.0080f
#define MAX_STEER           0.45f
#define TURN_KP             0.012f
#define TURN_SCALE          1.00f

#define SPEED_LOOP          1
#define SPEED_KP            1.2f
#define SPEED_KI            6.0f
#define SPEED_I_MAX         0.35f
#define SPEED_FILT          0.35f

#define TURN_MS_90          190
#define TURN_MS_180         335
#define MM_PER_S_AT_FULL    600.0f
#define MOTOR_TRIM          0.00f

#define TOF_TIMING_BUDGET_US 20000
#define TOF_FAR_MM           2000
#define TOF_STALE_MS         150

#define EXIT_FAR_MM          900
#define EXIT_CONFIRM_MM      100
#define EXIT_MIN_TIME_MS     2500
#define EXIT_RUNOUT_MM       180

#define OPP_MOTION_MM        45
#define OPP_JUMP_MM          150
#define OPP_MEMORY_MS        700
#define OPP_WAIT_MS          2200
#define OPP_BACKOFF_MM       60

#define STUCK_WINDOW_MS      800
#define STUCK_MIN_MOVE_MM    12

#define CONTROL_PERIOD_MS    5
#define DEBUG_SERIAL         1
