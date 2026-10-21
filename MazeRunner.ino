#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <vl53lx_class.h>
#include <soc/gpio_reg.h>
#include "config.h"

static inline float clampf(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }
static inline float signf(float x) { return x < 0 ? -1.0f : 1.0f; }
static const float COS45 = 0.70710678f;
static const float MM_PER_COUNT = (float)PI * WHEEL_DIAMETER_MM / ENC_COUNTS_PER_REV;

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 1
#endif
static const uint32_t PWM_MAX = (1u << PWM_BITS) - 1u;

static void pwmSetup(uint8_t pin, uint8_t ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch; ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(ch, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, ch);
#endif
}
static void pwmOut(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch; ledcWrite(pin, duty);
#else
  (void)pin; ledcWrite(ch, duty);
#endif
}

static float vbat = VBAT_NOMINAL, vbatScale = 1.0f;
static uint32_t lastVbatMs = 0;
static void updateVbat() {
#if USE_VBAT
  uint32_t now = millis();
  if (now - lastVbatMs < 20) return;
  lastVbatMs = now;
  float v = analogReadMilliVolts(PIN_VBAT) * 0.001f * VBAT_DIV;
  vbat += 0.1f * (v - vbat);
  vbatScale = (vbat > 5.5f && vbat < 9.0f) ? clampf(VBAT_NOMINAL / vbat, 0.80f, 1.25f) : 1.0f;
#endif
}

static void driveOne(uint8_t in1, uint8_t ch1, uint8_t in2, uint8_t ch2, float u, bool brake) {
  if (brake || fabsf(u) < 0.005f) {
    pwmOut(in1, ch1, PWM_MAX); pwmOut(in2, ch2, PWM_MAX); return;
  }
  float a = (MOTOR_DEADBAND + (1.0f - MOTOR_DEADBAND) * fabsf(u)) * vbatScale;
  a = clampf(a, 0, 1) * MOTOR_MAX_DUTY;
  uint32_t d = (uint32_t)(a * PWM_MAX + 0.5f);
  if (u > 0) { pwmOut(in1, ch1, PWM_MAX);     pwmOut(in2, ch2, PWM_MAX - d); }
  else       { pwmOut(in1, ch1, PWM_MAX - d); pwmOut(in2, ch2, PWM_MAX);     }
}
static void motorsCoast() {
  pwmOut(PIN_ML_IN1, 0, 0); pwmOut(PIN_ML_IN2, 1, 0);
  pwmOut(PIN_MR_IN1, 2, 0); pwmOut(PIN_MR_IN2, 3, 0);
}
static float tgtL = 0, tgtR = 0;
static float outL = 0, outR = 0;
static bool  brakeReq = false;

static void writeMotors(float l, float r) {
  if (MOTOR_L_INVERT) l = -l;
  if (MOTOR_R_INVERT) r = -r;
  driveOne(PIN_ML_IN1, 0, PIN_ML_IN2, 1, l, brakeReq);
  driveOne(PIN_MR_IN1, 2, PIN_MR_IN2, 3, r, brakeReq);
}
static void setTarget(float l, float r) {
  brakeReq = false;
  tgtL = clampf(l, -1, 1); tgtR = clampf(r, -1, 1);
}
static void setDrive(float v, float steer) {
  float t = MOTOR_TRIM * v;
  setTarget(v + steer + t, v - steer - t);
}

volatile int32_t encL = 0, encR = 0;
#if USE_ENCODERS
static volatile uint8_t encLs = 0, encRs = 0;
static const int8_t QDEC[16] = { 0,-1, 1, 0,  1, 0, 0,-1,  -1, 0, 0, 1,  0, 1,-1, 0 };
static inline uint8_t IRAM_ATTR fastRead(uint8_t pin) {
  return pin < 32 ? (REG_READ(GPIO_IN_REG) >> pin) & 1 : (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1;
}
void IRAM_ATTR isrEncL() {
  uint8_t s = (fastRead(PIN_ENC_L_A) << 1) | fastRead(PIN_ENC_L_B);
  int8_t d = QDEC[(encLs << 2) | s]; encLs = s;
  encL += ENC_L_INVERT ? -d : d;
}
void IRAM_ATTR isrEncR() {
  uint8_t s = (fastRead(PIN_ENC_R_A) << 1) | fastRead(PIN_ENC_R_B);
  int8_t d = QDEC[(encRs << 2) | s]; encRs = s;
  encR += ENC_R_INVERT ? -d : d;
}
#endif
static inline int32_t rdEncL() { return encL; }
static inline int32_t rdEncR() { return encR; }

static float velL = 0, velR = 0;
static bool  speedLoopOn = SPEED_LOOP && USE_ENCODERS;
#if USE_ENCODERS
static int32_t prevEncL = 0, prevEncR = 0;
static float   intL = 0, intR = 0;
static void updateWheelSpeed(float dt) {
  int32_t l = rdEncL(), r = rdEncR();
  velL += SPEED_FILT * ((l - prevEncL) * MM_PER_COUNT / dt - velL);
  velR += SPEED_FILT * ((r - prevEncR) * MM_PER_COUNT / dt - velR);
  prevEncL = l; prevEncR = r;
}
static float speedCtl(float u, float vel, float& integ, float dt) {
  if (!speedLoopOn || brakeReq || fabsf(u) < 0.005f) { integ = 0; return u; }
  float e = u - vel / MM_PER_S_AT_FULL;
  integ = clampf(integ + SPEED_KI * e * dt, -SPEED_I_MAX, SPEED_I_MAX);
  return clampf(u + SPEED_KP * e + integ, -1, 1);
}
#endif
static void resetWheelSpeed() {
#if USE_ENCODERS
  prevEncL = rdEncL(); prevEncR = rdEncR(); intL = intR = 0;
#endif
  velL = velR = 0;
}

static float odomModelMm = 0;
static float odomMm() {
#if USE_ENCODERS
  return (rdEncL() + rdEncR()) * 0.5f * MM_PER_COUNT;
#else
  return odomModelMm;
#endif
}

static VL53LX* tof[TOF_COUNT];
struct TofSnap { int16_t mm[TOF_COUNT]; uint32_t t[TOF_COUNT]; uint32_t seq[TOF_COUNT]; };
static TofSnap tofShared;
static portMUX_TYPE tofMux = portMUX_INITIALIZER_UNLOCKED;
static TofSnap S;
static bool tofOk[TOF_COUNT];

static bool initTof() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  Wire.setBufferSize(256);
#endif
  Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQ);
  for (int i = 0; i < TOF_COUNT; i++) {
    tof[i] = new VL53LX(&Wire, PIN_XSHUT[i]);
    tof[i]->begin();
    tof[i]->VL53LX_Off();
  }
  delay(10);
  bool all = true;
  for (int i = 0; i < TOF_COUNT; i++) {
    int st = tof[i]->InitSensor(TOF_ADDR[i]);
    tofOk[i] = (st == 0);
    if (!tofOk[i]) { Serial.printf("[ToF] senzor %d EROARE init (%d)\n", i, st); all = false; continue; }
    tof[i]->VL53LX_SetMeasurementTimingBudgetMicroSeconds(TOF_TIMING_BUDGET_US);
    tof[i]->VL53LX_StartMeasurement();
    Serial.printf("[ToF] senzor %d OK la 0x%02X\n", i, TOF_ADDR[i]);
  }
  for (int i = 0; i < TOF_COUNT; i++) { tofShared.mm[i] = TOF_FAR_MM; tofShared.t[i] = 0; tofShared.seq[i] = 0; }
  return all;
}

static void tofTask(void*) {
  int16_t lastValid[TOF_COUNT]; uint32_t lastValidT[TOF_COUNT];
  for (int i = 0; i < TOF_COUNT; i++) { lastValid[i] = TOF_FAR_MM; lastValidT[i] = 0; }
  for (;;) {
    for (int i = 0; i < TOF_COUNT; i++) {
      if (!tofOk[i]) continue;
      uint8_t ready = 0;
      if (tof[i]->VL53LX_GetMeasurementDataReady(&ready) != 0 || !ready) continue;
      VL53LX_MultiRangingData_t d;
      if (tof[i]->VL53LX_GetMultiRangingData(&d) == 0) {
        int best = TOF_FAR_MM; bool valid = false;
        for (int j = 0; j < d.NumberOfObjectsFound; j++) {
          if (d.RangeData[j].RangeStatus == 0) {
            int mm = d.RangeData[j].RangeMilliMeter;
            if (mm >= 0 && mm < best) { best = mm; valid = true; }
          }
        }
        uint32_t now = millis();
        if (valid) { lastValid[i] = best; lastValidT[i] = now; }
        else if (lastValid[i] < 80 && now - lastValidT[i] < 200) best = lastValid[i];
        portENTER_CRITICAL(&tofMux);
        tofShared.mm[i] = (int16_t)best; tofShared.t[i] = now; tofShared.seq[i]++;
        portEXIT_CRITICAL(&tofMux);
      }
      tof[i]->VL53LX_ClearInterruptAndStartMeasurement();
    }
    vTaskDelay(1);
  }
}
static void readSnapshot() {
  portENTER_CRITICAL(&tofMux); S = tofShared; portEXIT_CRITICAL(&tofMux);
  uint32_t now = millis();
  for (int i = 0; i < TOF_COUNT; i++) if (now - S.t[i] > TOF_STALE_MS) S.mm[i] = 60;
}

static int8_t HAND = +1;
static Preferences prefs;
static bool running = false, finished = false, exited = false;
static uint32_t tRun0 = 0;
static TickType_t lastWake;

static inline int dL()  { return S.mm[TOF_L]; }
static inline int dR()  { return S.mm[TOF_R]; }
static inline int dF()  { return S.mm[TOF_F]; }
static inline int dFL() { return S.mm[TOF_FL]; }
static inline int dFR() { return S.mm[TOF_FR]; }
static inline int dHand()  { return HAND > 0 ? dL() : dR(); }
static inline int dOpp()   { return HAND > 0 ? dR() : dL(); }

static void haltForever(const char* why) {
  setTarget(0, 0); outL = outR = 0; brakeReq = true; writeMotors(0, 0); delay(60);
  motorsCoast();
  Serial.printf("[STOP] %s – repornește alimentarea\n", why);
  for (;;) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(500); }
}
static uint8_t stopCnt = 0;
static void safety() {
  if (!running) return;
#if !TEST_START_BUTTON
  bool stop = digitalRead(PIN_START) == LOW;
#if USE_KILL_PIN
  stop = stop || digitalRead(PIN_KILL) == LOW;
#endif
  stopCnt = stop ? stopCnt + 1 : 0;
  if (stopCnt >= 3) haltForever("semnal STOP de la arbitru");
#endif
}

struct FSample { int16_t f; float odo; };
static FSample fRing[8]; static uint8_t fHead = 0, fCount = 0;
static uint32_t lastFrontSeq = 0, lastMovingMs = 0, trackerMuteUntil = 0;

static void updateOpponentTracker() {
  if (S.seq[TOF_F] == lastFrontSeq) return;
  lastFrontSeq = S.seq[TOF_F];
  uint32_t now = millis();
  if (now < trackerMuteUntil) { fCount = 0; return; }
  FSample s = { (int16_t)dF(), odomMm() };
  if (fCount > 0) {
    const FSample& p = fRing[(fHead + 7) & 7];
    if (p.f - s.f > OPP_JUMP_MM && s.f < 500) lastMovingMs = now;
  }
  fRing[fHead] = s; fHead = (fHead + 1) & 7; if (fCount < 8) fCount++;
  if (fCount == 8) {
    const FSample& o = fRing[fHead];
    float closing = (o.f - s.f) - (s.odo - o.odo);
    if (s.f < 600 && closing > OPP_MOTION_MM) lastMovingMs = now;
  }
}
static inline bool opponentRecently() { return millis() - lastMovingMs < OPP_MEMORY_MS; }

static uint32_t lastDbg = 0;
static const char* stateName = "IDLE";
static void step() {
  vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  const float dt = CONTROL_PERIOD_MS / 1000.0f;
  safety();
  readSnapshot();
  updateVbat();
  float maxd = ACCEL_PER_S * dt;
  outL += clampf(tgtL - outL, -maxd, maxd);
  outR += clampf(tgtR - outR, -maxd, maxd);
#if USE_ENCODERS
  updateWheelSpeed(dt);
  writeMotors(speedCtl(outL, velL, intL, dt), speedCtl(outR, velR, intR, dt));
#else
  writeMotors(outL, outR);
#endif
  odomModelMm += (outL + outR) * 0.5f * MM_PER_S_AT_FULL * dt;
  updateOpponentTracker();
#if DEBUG_SERIAL
  if (millis() - lastDbg > 100) {
    lastDbg = millis();
    Serial.printf("%-8s L%4d FL%4d F%4d FR%4d R%4d | odo %6.0f | out %.2f %.2f | v %4.0f %4.0f\n",
                  stateName, dL(), dFL(), dF(), dFR(), dR(), odomMm(), outL, outR, velL, velR);
  }
#endif
}
static void stopHard(uint16_t ms = 40) {
  setTarget(0, 0); outL = outR = 0; brakeReq = true;
  for (uint16_t t = 0; t < ms; t += CONTROL_PERIOD_MS) step();
  brakeReq = false;
}
static void settle(uint16_t ms) { setTarget(0, 0); for (uint16_t t = 0; t < ms; t += CONTROL_PERIOD_MS) step(); }

struct Travel { float odo0; int front0; bool useFront; };
static Travel travelStart() { Travel t; t.odo0 = odomMm(); t.front0 = dF(); t.useFront = t.front0 < 1200; return t; }
static float travelMm(const Travel& t) {
  float o = odomMm() - t.odo0;
#if !USE_ENCODERS
  if (t.useFront && dF() < 1300) {
    float f = (float)(t.front0 - dF());
    if (f > o * 0.4f - 20 && f < o * 2.0f + 40) return f;
  }
#endif
  return o;
}

static float lastLat = 0; static int lastMode = -1;
#if USE_ENCODERS
static int32_t holdRef = 0;
#endif

static float wallSteer() {
  const float dt = CONTROL_PERIOD_MS / 1000.0f;
  int l = dL(), r = dR(), fl = dFL(), fr = dFR();
  bool wl = l < WALL_PRESENT_MM, wr = r < WALL_PRESENT_MM;
  bool hl = wl && fl < l * 1.4142f + DIAG_VALID_EXTRA_MM;
  bool hr = wr && fr < r * 1.4142f + DIAG_VALID_EXTRA_MM;
  float headL = hl ? (l - fl * COS45) : 0;
  float headR = hr ? (r - fr * COS45) : 0;

  int mode; float lat, head;
  if (wl && wr)  { mode = 0; lat = (r - l) * 0.5f;           head = (headL - headR) * 0.5f; }
  else if (wl)   { mode = 1; lat = SIDE_TARGET_MM - l;       head = headL; }
  else if (wr)   { mode = 2; lat = r - SIDE_TARGET_MM;       head = -headR; }
  else {
#if USE_ENCODERS
    int32_t diff = rdEncL() - rdEncR();
    if (lastMode == -1) holdRef = diff;
    lastMode = 3;
    return clampf(-K_ENC_HOLD * (diff - holdRef) * MM_PER_COUNT, -MAX_STEER, MAX_STEER);
#else
    lastMode = 3; return 0;
#endif
  }
#if USE_ENCODERS
  holdRef = rdEncL() - rdEncR();
#endif
  float dlat = (mode == lastMode) ? (lat - lastLat) / dt : 0;
  lastLat = lat; lastMode = mode;
  float s = KP_LAT * lat + KD_LAT * dlat + KP_HEAD * head;
  return clampf(s, -MAX_STEER, MAX_STEER);
}
static void followStep(float vBase) {
  float f = (float)dF();
  float k = clampf((f - FRONT_TURN_MM) / (FRONT_SLOW_MM - FRONT_TURN_MM), 0, 1);
  float v = V_MIN + (vBase - V_MIN) * k;
  float s = wallSteer();
  v *= 1.0f - 0.6f * fabsf(s);
  setDrive(v, s);
}

static uint32_t exitSince = 0;
static bool exitDetected() {
  if (millis() - tRun0 < EXIT_MIN_TIME_MS) { exitSince = 0; return false; }
  bool cand = dFL() > EXIT_FAR_MM && dFR() > EXIT_FAR_MM && dF() > EXIT_FAR_MM &&
              (dL() > EXIT_FAR_MM || dR() > EXIT_FAR_MM);
  if (!cand) { exitSince = 0; return false; }
  if (!exitSince) exitSince = millis();
  return millis() - exitSince >= EXIT_CONFIRM_MS;
}
static void doExit() {
  stateName = "EXIT";
  Travel t = travelStart(); t.useFront = false;
#if USE_ENCODERS
  holdRef = rdEncL() - rdEncR();
#endif
  while (travelMm(t) < EXIT_RUNOUT_MM) {
#if USE_ENCODERS
    int32_t diff = rdEncL() - rdEncR();
    setDrive(V_EXIT, clampf(-K_ENC_HOLD * (diff - holdRef) * MM_PER_COUNT, -0.2f, 0.2f));
#else
    setDrive(V_EXIT, 0);
#endif
    step();
  }
  stopHard(80);
  exited = true; finished = true;
  Serial.printf("[EXIT] Afară în %.2f s!\n", (millis() - tRun0) / 1000.0f);
}

static void pivot(float deg) {
  stateName = "PIVOT";
  stopHard(25);
  trackerMuteUntil = millis() + 100000;
  float s = signf(deg);
#if USE_ENCODERS
  float arc = (float)PI * WHEEL_BASE_MM * fabsf(deg) / 360.0f * TURN_SCALE;
  int32_t l0 = rdEncL(), r0 = rdEncR();
  uint32_t t0 = millis();
  for (;;) {
    float done = (fabsf((float)(rdEncL() - l0)) + fabsf((float)(rdEncR() - r0))) * 0.5f * MM_PER_COUNT;
    float rem = arc - done;
    if (rem <= 0 || millis() - t0 > 1500) break;
    float w = clampf(rem * TURN_KP + V_TURN_MIN, V_TURN_MIN, V_TURN);
    setTarget(-s * w, s * w);
    step();
  }
#else
  const float msPerDeg = (TURN_MS_180 - TURN_MS_90) / 90.0f;
  float msF = TURN_MS_90 + (fabsf(deg) - 90.0f) * msPerDeg;
  uint32_t ms = msF > 0 ? (uint32_t)msF : 0;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { setTarget(-s * V_TURN, s * V_TURN); step(); }
#endif
  stopHard(30);
  trackerMuteUntil = millis() + 120;
  lastMode = -1;
}

static bool driveDistance(float mm, float v, bool stopAtFront) {
  Travel t = travelStart();
  for (;;) {
    float rem = mm - travelMm(t);
    if (rem <= 0) return true;
    if (exitDetected()) { doExit(); return false; }
    if (stopAtFront && dF() <= FRONT_TURN_MM + 3) return true;
    float vv = clampf(V_MIN + rem * 0.006f, V_MIN, v);
    float s = wallSteer();
    setDrive(vv, s);
    step();
  }
}

static void alignFront() {
  uint32_t t0 = millis();
  while (millis() - t0 < 180) {
    float e = (float)dF() - FRONT_TURN_MM;
    if (fabsf(e) < 6 || dF() > FRONT_TURN_MM + 60) break;
    float v = clampf(e * 0.006f, -0.30f, 0.30f);
    if (fabsf(v) < 0.18f) v = 0.18f * signf(v);
    setDrive(v, 0);
    step();
  }
  stopHard(20);
}

static float stuckOdo = 0; static int stuckF = 0; static uint32_t stuckT = 0;
static void stuckCheck() {
  uint32_t now = millis();
  if (now - stuckT < STUCK_WINDOW_MS) return;
  bool pushing = (outL + outR) * 0.5f > 0.25f;
#if USE_ENCODERS
  float moved = odomMm() - stuckOdo;
  bool still = fabsf(moved) < STUCK_MIN_MOVE_MM ||
               (moved > 60 && dF() < 400 && abs(dF() - stuckF) < 8);
#else
  bool still = dF() < 1200 && abs(dF() - stuckF) < 8;
#endif
  if (pushing && still) {
    stateName = "UNSTUCK";
    Serial.println("[STUCK] mă deblochez");
    uint32_t t0 = millis();
    float turn = (dL() < dR()) ? 0.25f : -0.25f;
    while (millis() - t0 < 250) { setDrive(-0.45f, turn); step(); }
    stopHard(30);
    lastMode = -1;
  }
  stuckOdo = odomMm(); stuckF = dF(); stuckT = millis();
}

enum WallSt { WS_OPEN, WS_WALL };
static WallSt handPrev = WS_OPEN;
static uint32_t handOpenSince = 0;

static bool handOpeningEvent() {
  int d = dHand();
  if (d < WALL_PRESENT_MM) { handPrev = WS_WALL; handOpenSince = 0; return false; }
  if (d > OPEN_MM && handPrev == WS_WALL) {
    if (!handOpenSince) handOpenSince = millis();
    if (millis() - handOpenSince >= 15) { handPrev = WS_OPEN; handOpenSince = 0; return true; }
  }
  return false;
}

static bool waitForClear() {
  stateName = "OPP_WAIT";
  uint32_t t0 = millis(), clearSince = 0;
  while (millis() - t0 < OPP_WAIT_MS) {
    if (dF() < OPP_BACKOFF_MM) setDrive(-0.25f, 0); else setTarget(0, 0);
    step();
    if (dF() > FRONT_TURN_MM + 120) {
      if (!clearSince) clearSince = millis();
      if (millis() - clearSince > 120) return true;
    } else clearSince = 0;
  }
  return false;
}

static void frontDecision() {
  stateName = "DECIDE";
  stopHard(30);
  if (opponentRecently() && waitForClear()) return;
  settle(45);
  alignFront();
  if (dF() > FRONT_TURN_MM + 100) return;
  bool handOpen = dHand() > OPEN_MM;
  bool oppOpen  = dOpp()  > OPEN_MM;
  if (handOpen) {
    pivot(90.0f * HAND);
    handPrev = WS_WALL;
    driveDistance(ENTER_AFTER_TURN_MM, V_ADVANCE, true);
  } else if (oppOpen) {
    pivot(-90.0f * HAND);
    driveDistance(ENTER_SHORT_MM, V_ADVANCE, true);
    handPrev = dHand() < WALL_PRESENT_MM ? WS_WALL : WS_OPEN;
  } else {
    stateName = "DEADEND";
    pivot(180.0f * HAND);
    driveDistance(ENTER_SHORT_MM, V_ADVANCE, true);
    handPrev = dHand() < WALL_PRESENT_MM ? WS_WALL : WS_OPEN;
  }
}

static void runMaze() {
  lastWake = xTaskGetTickCount();
  resetWheelSpeed(); lastMode = -1;
  tRun0 = millis(); handPrev = WS_OPEN;
  stuckT = millis(); stuckOdo = odomMm(); stuckF = dF();
  while (!finished) {
    step();
    if (exitDetected()) { doExit(); break; }

    if (handOpeningEvent()) {
      stateName = "ADVANCE";
      if (!driveDistance(ADVANCE_TO_TURN_MM, V_ADVANCE, true)) break;
      if (dHand() > OPEN_MM) {
        pivot(90.0f * HAND);
        handPrev = WS_WALL;
        stateName = "ENTER";
        if (!driveDistance(ENTER_AFTER_TURN_MM, V_ADVANCE, true)) break;
      }
      stuckT = millis(); stuckOdo = odomMm();
      continue;
    }
    if (dF() < FRONT_TURN_MM + 4) {
      frontDecision();
      if (finished) break;
      stuckT = millis(); stuckOdo = odomMm();
      continue;
    }
    stateName = "FOLLOW";
    followStep(V_CRUISE);
    stuckCheck();
  }
  stateName = "DONE";
  for (;;) { setTarget(0, 0); brakeReq = true; step(); digitalWrite(PIN_LED, (millis() / 100) & 1); }
}

static void showHand() { Serial.printf("[CFG] Regula: mâna %s\n", HAND > 0 ? "STÂNGĂ" : "DREAPTĂ"); }
static void ledPattern() {
  uint32_t p = millis() % 1200;
  bool on = (p < 120) || (HAND < 0 && p >= 250 && p < 370);
  digitalWrite(PIN_LED, on);
}
static void errorBlinkForever(const char* why) {
  motorsCoast(); Serial.println(why);
  for (;;) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(80); }
}

static void encoderTest() {
#if USE_ENCODERS
  const float U = 0.6f;
  float spd[2];
  speedLoopOn = false;
  for (int w = 0; w < 2; w++) {
    const char* name = w == 0 ? "stâng" : "drept";
    uint32_t t0 = millis();
    int32_t a0 = 0, b0 = 0, a1, b1; uint32_t tm = 0;
    while (millis() - t0 < 600) {
      if (w == 0) setTarget(U, 0); else setTarget(0, U);
      step();
      if (!tm && millis() - t0 >= 300) { tm = millis(); a0 = rdEncL(); b0 = rdEncR(); }
    }
    a1 = rdEncL(); b1 = rdEncR();
    float sec = (millis() - tm) / 1000.0f;
    int32_t own = w == 0 ? a1 - a0 : b1 - b0, other = w == 0 ? b1 - b0 : a1 - a0;
    spd[w] = fabsf(own) * MM_PER_COUNT / sec;
    stopHard(60); settle(200);
    Serial.printf("[ENC] motor %s: %+ld impulsuri proprii, %+ld pe celălalt encoder, %.0f mm/s -> ",
                  name, (long)own, (long)other, spd[w]);
    if (abs(own) < 20 && abs(other) > 20) Serial.println("encoderele sunt INVERSATE între ele (schimbă pinii L/R)");
    else if (abs(own) < 20) Serial.println("NICIUN impuls: verifică alimentarea (3,3 V) și firele A/B");
    else if (own < 0) Serial.printf("sens INVERS: schimbă ENC_%c_INVERT\n", w == 0 ? 'L' : 'R');
    else Serial.println("OK");
  }
  speedLoopOn = SPEED_LOOP;
  Serial.printf("[ENC] MM_PER_S_AT_FULL aprox. %.0f (roți în aer; pe podea pune ~90%% din valoare)\n",
                (spd[0] + spd[1]) * 0.5f / U);
#else
  Serial.println("[ENC] USE_ENCODERS = 0");
#endif
}

static void serialCommands() {
  if (!Serial.available()) return;
  char c = Serial.read();
  running = false;
  lastWake = xTaskGetTickCount();
  resetWheelSpeed();
  switch (c) {
    case 'l': HAND = +1; prefs.putChar("hand", HAND); showHand(); break;
    case 'r': HAND = -1; prefs.putChar("hand", HAND); showHand(); break;
    case 's': readSnapshot();
      Serial.printf("L%d FL%d F%d FR%d R%d enc %ld %ld (%.0f / %.0f mm) | Vbat %.2f V (x%.2f)\n",
                    dL(), dFL(), dF(), dFR(), dR(), (long)rdEncL(), (long)rdEncR(),
                    rdEncL() * MM_PER_COUNT, rdEncR() * MM_PER_COUNT, vbat, vbatScale); break;
    case '9': pivot(90); settle(100); break;
    case '8': pivot(180); settle(100); break;
    case '0': pivot(360); settle(100); break;
    case 'e': encoderTest(); break;
    case 'm': { Travel t = travelStart(); t.useFront = false; lastMode = -1;
      while (travelMm(t) < CELL_MM) { setDrive(0.5f, wallSteer()); step(); }
      stopHard(60); settle(100); break; }
    default: break;
  }
  setTarget(0, 0); outL = outR = 0; motorsCoast();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_START, INPUT);
#if USE_VBAT
  vbat = analogReadMilliVolts(PIN_VBAT) * 0.001f * VBAT_DIV;
#endif
#if USE_KILL_PIN
  pinMode(PIN_KILL, INPUT);
#endif
  pwmSetup(PIN_ML_IN1, 0); pwmSetup(PIN_ML_IN2, 1);
  pwmSetup(PIN_MR_IN1, 2); pwmSetup(PIN_MR_IN2, 3);
  motorsCoast();

#if USE_ENCODERS
  pinMode(PIN_ENC_L_A, INPUT_PULLUP); pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP); pinMode(PIN_ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), isrEncR, CHANGE);
#endif

  prefs.begin("maze", false);
  HAND = prefs.getChar("hand", +1) >= 0 ? +1 : -1;
  Serial.println("\n=== MazeRunner – Robochallenge Maze ===");
  showHand();

#if !TEST_START_BUTTON
  if (digitalRead(PIN_START) == HIGH)
    errorBlinkForever("[ERR] START=1 la pornire! Cere arbitrului STOP, apoi repornește robotul.");
#endif

  if (!initTof()) errorBlinkForever("[ERR] Nu toți senzorii ToF au pornit. Verifică XSHUT/I2C.");
  xTaskCreatePinnedToCore(tofTask, "tof", 16384, nullptr, 3, nullptr, 0);
  delay(100);
}

void loop() {
  static uint32_t btnDown = 0;
  ledPattern();
  readSnapshot();
  updateVbat();
  serialCommands();

  bool btn = digitalRead(PIN_BUTTON) == LOW;
  if (btn && !btnDown) btnDown = millis();
  if (!btn && btnDown) {
    uint32_t held = millis() - btnDown; btnDown = 0;
    if (held > 30 && held < 800) { HAND = -HAND; prefs.putChar("hand", HAND); showHand(); }
  }

  bool go = false;
#if TEST_START_BUTTON
  if (btn && btnDown && millis() - btnDown > 1000) {
    Serial.println("[TEST] start în 2 s..."); delay(2000); go = true; btnDown = 0;
  }
#else
  static uint8_t hi = 0;
  hi = digitalRead(PIN_START) == HIGH ? hi + 1 : 0;
  go = hi >= 3;
#endif
  if (go) {
    Serial.println("[GO] Start!");
    digitalWrite(PIN_LED, HIGH);
    running = true;
    runMaze();
  }
  delay(5);
}
