#pragma once
#include <stdint.h>

// ---------------- Pini (după „Maze Electronica Schema Finala.pdf”) ----------------
#define PIN_SDA            21
#define PIN_SCL            22
#define I2C_FREQ           400000

// Senzorii fizici, în ordinea XSHUT: 0 = GPIO15 (sus în schemă), 1 = GPIO2 (mijloc), 2 = GPIO23 (jos).
// Care e L/F/R se alege din Serial cu comanda 'p' (se salvează), nu trebuie reîncărcat codul.
static const uint8_t PIN_XSHUT[3] = { 15, 2, 23 };
static const uint8_t TOF_ADDR[3]  = { 0x10, 0x12, 0x14 };

#define PIN_ML_IN1         25
#define PIN_ML_IN2         26
#define PIN_MR_IN1         27
#define PIN_MR_IN2         14

#define PIN_START          18
#define PIN_KILL           19
#define USE_KILL_PIN       1

// GPIO2 (LED-ul plăcii) e XSHUT pentru un senzor, deci LED-ul rămâne oprit.
#define USE_LED            0
#define PIN_LED            2
#define PIN_BUTTON         0

// 1 = pornire din butonul BOOT (ținut 1 s) în loc de modulul IR. 0 la competiție!
#define TEST_START_BUTTON  0

// ---------------- Geometrie (mm) ----------------
#define CELL_MM                300.0f
#define SIDE_SENSOR_OFFSET_MM   38.0f   // de la centrul robotului la fața senzorului lateral
#define SIDE_SENSOR_X_MM        30.0f   // cât e senzorul lateral în fața axului roților
#define FRONT_SENSOR_X_MM       45.0f   // de la axul roților la fața senzorului frontal

#define SIDE_TARGET_MM      (CELL_MM * 0.5f - SIDE_SENSOR_OFFSET_MM)
#define WALL_PRESENT_MM     (SIDE_TARGET_MM + 75.0f)
#define OPEN_MM             (SIDE_TARGET_MM + 115.0f)
#define FRONT_TURN_MM       (CELL_MM * 0.5f - FRONT_SENSOR_X_MM)
#define FRONT_SLOW_MM       350.0f
#define OPEN_DETECT_LAG_MM  25.0f
#define ADVANCE_TO_TURN_MM  (CELL_MM * 0.5f + SIDE_SENSOR_X_MM - OPEN_DETECT_LAG_MM)
#define ENTER_AFTER_TURN_MM (CELL_MM * 0.5f - SIDE_SENSOR_X_MM + 20.0f)
#define ENTER_SHORT_MM      50.0f

// ---------------- Viteze (fracție din PWM maxim, mici intenționat) ----------------
#define MOTOR_MAX_DUTY      1.00f
#define MOTOR_DEADBAND      0.10f
#define V_RUN               0.45f   // mers pe culoar
#define V_SLOW              0.25f   // aproape de peretele din față
#define V_ADVANCE           0.35f   // avans la un gol / intrare după viraj
#define V_TURN              0.45f   // pivotare pe loc
#define ACCEL_PER_S         4.0f
#define PWM_FREQ            20000
#define PWM_BITS            10

// ---------------- Regulator lateral ----------------
#define KP_LAT              0.0040f // steer pe mm abatere
#define KD_LAT              0.0015f // steer pe mm/s (amortizare)
#define MAX_STEER           0.30f

// ---------------- Valori implicite pentru parametrii salvați (se reglează din Serial) ----------------
#define DEF_TURN_MS_90      300
#define DEF_TURN_MS_180     560
#define DEF_MM_PER_S_RUN    200.0f  // viteza reală la V_RUN; comanda 'v' o măsoară

// ---------------- ToF ----------------
#define TOF_TIMING_BUDGET_US 20000
#define TOF_FAR_MM           2000
#define TOF_STALE_MS         150

// ---------------- Ieșire, adversar, blocare ----------------
#define EXIT_FAR_MM          900
#define EXIT_CONFIRM_MS      400
#define EXIT_MIN_TIME_MS     2500
#define EXIT_RUNOUT_MM       180
#define OPP_WAIT_MS          2000
#define STUCK_WINDOW_MS      1200
#define STUCK_MIN_CHANGE_MM  10

#define CONTROL_PERIOD_MS    5
#define DEBUG_SERIAL         1
