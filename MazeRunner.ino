#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <VL53L0X.h>
#include "config.h"

struct Travel;  // prototipurile generate automat pentru .ino apar înaintea definiției

static inline void ledSet(bool on) {
#if USE_LED
  digitalWrite(PIN_LED, on);
#else
  (void)on;
#endif
}

static inline float clampf(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }
static inline float signf(float x) { return x < 0 ? -1.0f : 1.0f; }

// ---------------- Parametri salvați în flash ----------------
struct Params {
  int8_t hand;          // +1 = mâna stângă, -1 = mâna dreaptă
  uint8_t invL, invR;   // 1 = motorul respectiv e inversat
  uint8_t perm;         // ce senzor fizic e L, F, R (index în PERMS)
  uint16_t t90, t180;   // timpii de pivotare (ms) la V_TURN
  float mmps;           // viteza reală la V_RUN (mm/s)
  float trim;           // corecție dacă robotul trage într-o parte fără pereți
};
static Params P;
static Preferences prefs;
static const uint8_t PERMS[6][3] = { {0,1,2}, {0,2,1}, {1,0,2}, {1,2,0}, {2,0,1}, {2,1,0} };

static void loadParams() {
  P.hand = prefs.getChar("hand", +1) >= 0 ? +1 : -1;
  P.invL = prefs.getUChar("invL", 0) ? 1 : 0;
  P.invR = prefs.getUChar("invR", 1) ? 1 : 0;
  P.perm = prefs.getUChar("perm", 0) % 6;
  P.t90  = prefs.getUShort("t90", DEF_TURN_MS_90);
  P.t180 = prefs.getUShort("t180", DEF_TURN_MS_180);
  P.mmps = prefs.getFloat("mmps", DEF_MM_PER_S_RUN);
  P.trim = prefs.getFloat("trim", 0.0f);
}
static void saveParams() {
  prefs.putChar("hand", P.hand);
  prefs.putUChar("invL", P.invL); prefs.putUChar("invR", P.invR);
  prefs.putUChar("perm", P.perm);
  prefs.putUShort("t90", P.t90); prefs.putUShort("t180", P.t180);
  prefs.putFloat("mmps", P.mmps); prefs.putFloat("trim", P.trim);
}
static void resetParams() {
  P.hand = +1; P.invL = 0; P.invR = 1; P.perm = 0;
  P.t90 = DEF_TURN_MS_90; P.t180 = DEF_TURN_MS_180;
  P.mmps = DEF_MM_PER_S_RUN; P.trim = 0;
  saveParams();
}
static void printParams() {
  const uint8_t* m = PERMS[P.perm];
  Serial.printf("[CFG] mâna %s | senzori L=GPIO%d F=GPIO%d R=GPIO%d (perm %d) | invL %d invR %d\n",
                P.hand > 0 ? "STÂNGĂ" : "DREAPTĂ", PIN_XSHUT[m[0]], PIN_XSHUT[m[1]], PIN_XSHUT[m[2]],
                P.perm, P.invL, P.invR);
  Serial.printf("[CFG] t90 %u ms | t180 %u ms | viteză %.0f mm/s | trim %+.2f\n",
                P.t90, P.t180, P.mmps, P.trim);
}

// ---------------- Motoare ----------------
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

// DRV8835 în mod PHASE/ENABLE: PH = sensul, EN = PWM; EN = 0 frânează (ieșirile trec pe LOW).
static void driveOne(uint8_t ph, uint8_t en, uint8_t ch, float u, bool brake) {
  if (brake || fabsf(u) < 0.005f) { pwmOut(en, ch, 0); return; }
  float a = clampf(MOTOR_DEADBAND + (1.0f - MOTOR_DEADBAND) * fabsf(u), 0, 1) * MOTOR_MAX_DUTY;
  digitalWrite(ph, u < 0 ? HIGH : LOW);
  pwmOut(en, ch, (uint32_t)(a * PWM_MAX + 0.5f));
}
static void motorsCoast() {
  pwmOut(PIN_ML_EN, 0, 0); pwmOut(PIN_MR_EN, 1, 0);
}
static float tgtL = 0, tgtR = 0;
static float outL = 0, outR = 0;
static bool  brakeReq = false;

static void writeMotors(float l, float r) {
  if (P.invL) l = -l;
  if (P.invR) r = -r;
  driveOne(PIN_ML_PH, PIN_ML_EN, 0, l, brakeReq);
  driveOne(PIN_MR_PH, PIN_MR_EN, 1, r, brakeReq);
}
static void setTarget(float l, float r) {
  brakeReq = false;
  tgtL = clampf(l, -1, 1); tgtR = clampf(r, -1, 1);
}
static void setDrive(float v, float steer) {
  float t = P.trim * v;
  setTarget(v + steer + t, v - steer - t);
}

// Fără encodere distanța se estimează din comanda dată motoarelor și viteza măsurată cu 'v'.
static float odomModelMm = 0;
static inline float odomMm() { return odomModelMm; }

// ---------------- Senzori ToF ----------------
static VL53L0X tof[3];
struct TofSnap { int16_t mm[3]; uint32_t t[3]; uint32_t seq[3]; };
static TofSnap tofShared;
static portMUX_TYPE tofMux = portMUX_INITIALIZER_UNLOCKED;
static TofSnap S;
static bool tofOk[3];

// Toți senzorii pornesc pe aceeași adresă (0x29): îi trezim pe rând din XSHUT și le dăm adrese noi.
static bool initTof() {
  for (int i = 0; i < 3; i++) { pinMode(PIN_XSHUT[i], OUTPUT); digitalWrite(PIN_XSHUT[i], LOW); }
  delay(20);
  bool all = true;
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_XSHUT[i], HIGH);
    delay(15);
    tof[i] = VL53L0X();
    tof[i].setBus(&Wire);
    tof[i].setTimeout(100);
    tofOk[i] = tof[i].init();
    if (!tofOk[i]) {
      Serial.printf("[ToF] GPIO%d EROARE init\n", PIN_XSHUT[i]);
      digitalWrite(PIN_XSHUT[i], LOW);   // altfel ar ocupa 0x29 pentru următorul senzor
      all = false; continue;
    }
    tof[i].setAddress(TOF_ADDR[i]);
    tof[i].setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
    tof[i].startContinuous();
    Serial.printf("[ToF] GPIO%d OK\n", PIN_XSHUT[i]);
  }
  return all;
}

static void tofTask(void*) {
  for (;;) {
    for (int i = 0; i < 3; i++) {
      if (!tofOk[i]) continue;
      if ((tof[i].readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) continue;
      uint16_t mm = tof[i].readRangeContinuousMillimeters();
      if (tof[i].timeoutOccurred()) continue;
      int16_t v = mm >= TOF_FAR_MM ? TOF_FAR_MM : (int16_t)mm;   // 8190/8191 = nimic în rază
      uint32_t now = millis();
      portENTER_CRITICAL(&tofMux);
      tofShared.mm[i] = v; tofShared.t[i] = now; tofShared.seq[i]++;
      portEXIT_CRITICAL(&tofMux);
    }
    vTaskDelay(1);
  }
}
static void readSnapshot() {
  portENTER_CRITICAL(&tofMux); S = tofShared; portEXIT_CRITICAL(&tofMux);
  uint32_t now = millis();
  // Un senzor care nu mai răspunde e tratat ca perete foarte aproape: robotul se oprește, nu intră în el.
  for (int i = 0; i < 3; i++) if (now - S.t[i] > TOF_STALE_MS) S.mm[i] = 60;
}

static inline int dL() { return S.mm[PERMS[P.perm][0]]; }
static inline int dF() { return S.mm[PERMS[P.perm][1]]; }
static inline int dR() { return S.mm[PERMS[P.perm][2]]; }
static inline uint32_t seqSide() { return S.seq[PERMS[P.perm][0]] + S.seq[PERMS[P.perm][2]]; }
static inline int dHand() { return P.hand > 0 ? dL() : dR(); }
static inline int dOpp()  { return P.hand > 0 ? dR() : dL(); }

// ---------------- Bucla de control (200 Hz) ----------------
static bool running = false, finished = false;
static bool testRun = false;    // pornit cu 'g' din Serial (ignoră modulul IR)
static bool irBlocked = false;  // START era 1 la pornire: modulul IR e ignorat până la repornire
static bool autoArmed = !USE_START_MODULE;   // pornire automată după AUTO_START_MS
static uint32_t autoT0 = 0;
static uint32_t tRun0 = 0;
static TickType_t lastWake;

static void haltForever(const char* why) {
  setTarget(0, 0); outL = outR = 0; brakeReq = true; writeMotors(0, 0); delay(60);
  motorsCoast();
  Serial.printf("[STOP] %s – repornește alimentarea\n", why);
  for (;;) { ledSet((millis() / 500) & 1); delay(10); }
}
static uint8_t stopCnt = 0;
static void safety() {
  if (!running) return;
  if (digitalRead(PIN_BUTTON) == LOW || Serial.available()) haltForever("oprit manual (BOOT sau tastă)");
#if USE_START_MODULE
  if (testRun) return;
  bool stop = digitalRead(PIN_START) == LOW;
#if PIN_KILL >= 0
  stop = stop || digitalRead(PIN_KILL) == LOW;
#endif
  stopCnt = stop ? stopCnt + 1 : 0;
  if (stopCnt >= 3) haltForever("semnal STOP de la arbitru");
#endif
}

static uint32_t lastDbg = 0;
static const char* stateName = "IDLE";
static void step() {
  vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  const float dt = CONTROL_PERIOD_MS / 1000.0f;
  safety();
  readSnapshot();
  float maxd = ACCEL_PER_S * dt;
  outL += clampf(tgtL - outL, -maxd, maxd);
  outR += clampf(tgtR - outR, -maxd, maxd);
  writeMotors(outL, outR);
  odomModelMm += (outL + outR) * 0.5f * (P.mmps / V_RUN) * dt;
#if DEBUG_SERIAL
  if (running && millis() - lastDbg > 100) {
    lastDbg = millis();
    Serial.printf("%-8s L%4d F%4d R%4d | odo %6.0f | out %.2f %.2f\n",
                  stateName, dL(), dF(), dR(), odomMm(), outL, outR);
  }
#endif
}
static void stopHard(uint16_t ms = 40) {
  setTarget(0, 0); outL = outR = 0; brakeReq = true;
  for (uint16_t t = 0; t < ms; t += CONTROL_PERIOD_MS) step();
  brakeReq = false;
}
static void settle(uint16_t ms) { setTarget(0, 0); for (uint16_t t = 0; t < ms; t += CONTROL_PERIOD_MS) step(); }

// Distanța parcursă: din peretele din față când se vede și e plauzibilă, altfel din model.
struct Travel { float odo0; int front0; bool useFront; };
static Travel travelStart() { Travel t; t.odo0 = odomMm(); t.front0 = dF(); t.useFront = t.front0 < 1200; return t; }
static float travelMm(const Travel& t) {
  float o = odomMm() - t.odo0;
  if (t.useFront && dF() < 1300) {
    float f = (float)(t.front0 - dF());
    if (f > o * 0.4f - 20 && f < o * 2.0f + 40) return f;
  }
  return o;
}

// ---------------- Urmărire de perete (P + amortizare) ----------------
static int lastMode = -1;
static float prevLat = 0, latRate = 0;
static uint32_t prevLatMs = 0, lastSideSeq = 0;

static float wallSteer() {
  int l = dL(), r = dR();
  bool wl = l < WALL_PRESENT_MM, wr = r < WALL_PRESENT_MM;
  int mode; float lat;
  if (wl && wr)  { mode = 0; lat = (r - l) * 0.5f; }
  else if (wl)   { mode = 1; lat = SIDE_TARGET_MM - l; }
  else if (wr)   { mode = 2; lat = r - SIDE_TARGET_MM; }
  else           { lastMode = 3; return 0; }

  uint32_t now = millis(), seq = seqSide();
  if (mode != lastMode) {
    latRate = 0; prevLat = lat; prevLatMs = now;
  } else if (seq != lastSideSeq && now - prevLatMs >= 15) {
    float rate = (lat - prevLat) * 1000.0f / (now - prevLatMs);
    latRate += 0.5f * (clampf(rate, -300, 300) - latRate);
    prevLat = lat; prevLatMs = now;
  }
  lastSideSeq = seq; lastMode = mode;
  return clampf(KP_LAT * lat + KD_LAT * latRate, -MAX_STEER, MAX_STEER);
}
static void followStep() {
  float k = clampf(((float)dF() - FRONT_TURN_MM) / (FRONT_SLOW_MM - FRONT_TURN_MM), 0, 1);
  float v = V_SLOW + (V_RUN - V_SLOW) * k;
  float s = wallSteer();
  setDrive(v * (1.0f - 0.5f * fabsf(s)), s);
}

// ---------------- Ieșire ----------------
static uint32_t exitSince = 0;
static inline bool allFar(int mm) { return dL() > mm && dF() > mm && dR() > mm; }
static bool exitDetected() {
  if (millis() - tRun0 < EXIT_MIN_TIME_MS || !allFar(EXIT_FAR_MM)) { exitSince = 0; return false; }
  if (!exitSince) exitSince = millis();
  return millis() - exitSince >= EXIT_CONFIRM_MS;
}
static bool doExit() {
  stateName = "EXIT";
  Travel t = travelStart(); t.useFront = false;
  while (travelMm(t) < EXIT_RUNOUT_MM) { setDrive(V_ADVANCE, 0); step(); }
  stopHard(80);
  settle(100);
  exitSince = 0;
  if (!allFar((int)(EXIT_FAR_MM * 0.8f))) {
    Serial.println("[EXIT] fals (văd pereți), continui");
    lastMode = -1;
    return false;
  }
  finished = true;
  Serial.printf("[EXIT] Afară în %.2f s!\n", (millis() - tRun0) / 1000.0f);
  return true;
}

// ---------------- Manevre ----------------
static void pivot(int deg) {   // +90 = stânga, -90 = dreapta, ±180 = întoarcere
  stateName = "PIVOT";
  stopHard(30);
  float s = deg > 0 ? 1.0f : -1.0f;
  uint32_t ms = abs(deg) >= 135 ? P.t180 : P.t90;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { setTarget(-s * V_TURN, s * V_TURN); step(); }
  stopHard(40);
  settle(40);
  lastMode = -1;
}

static bool driveDistance(float mm, float v, bool stopAtFront) {
  Travel t = travelStart();
  uint32_t t0 = millis();
  for (;;) {
    float rem = mm - travelMm(t);
    if (rem <= 0) return true;
    if (millis() - t0 > 4000) return true;   // plasă de siguranță dacă modelul de viteză e greșit
    if (exitDetected() && doExit()) return false;
    if (stopAtFront && dF() <= FRONT_TURN_MM + 3) return true;
    setDrive(v, wallSteer());
    step();
  }
}

static void alignFront() {
  uint32_t t0 = millis();
  while (millis() - t0 < 400) {
    float e = (float)dF() - FRONT_TURN_MM;
    if (fabsf(e) < 8 || dF() > FRONT_TURN_MM + 80) break;
    float v = clampf(e * 0.005f, -0.25f, 0.25f);
    if (fabsf(v) < 0.18f) v = 0.18f * signf(v);
    setDrive(v, 0);
    step();
  }
  stopHard(30);
}

static int stuckF = 0; static uint32_t stuckT = 0;
static void stuckReset() { stuckT = millis(); stuckF = dF(); }
static void stuckCheck() {
  if (millis() - stuckT < STUCK_WINDOW_MS) return;
  bool pushing = (outL + outR) * 0.5f > 0.2f;
  bool still = dF() < 1200 && abs(dF() - stuckF) < STUCK_MIN_CHANGE_MM;
  if (pushing && still) {
    stateName = "UNSTUCK";
    Serial.println("[STUCK] mă deblochez");
    float turn = (dL() < dR()) ? 0.2f : -0.2f;
    uint32_t t0 = millis();
    while (millis() - t0 < 300) { setDrive(-0.4f, turn); step(); }
    stopHard(30);
    lastMode = -1;
  }
  stuckReset();
}

enum WallSt { WS_OPEN, WS_WALL };
static WallSt handPrev = WS_OPEN;
static uint32_t handOpenSince = 0;
static bool handOpeningEvent() {
  int d = dHand();
  if (d < WALL_PRESENT_MM) { handPrev = WS_WALL; handOpenSince = 0; return false; }
  if (d > OPEN_MM && handPrev == WS_WALL) {
    if (!handOpenSince) handOpenSince = millis();
    if (millis() - handOpenSince >= 20) { handPrev = WS_OPEN; handOpenSince = 0; return true; }
  }
  return false;
}

// Perete în față: dacă obstacolul se mișcă e adversarul, așteptăm să plece; altfel virăm.
static bool frontBlockedByOpponent() {
  stateName = "CHECK";
  int f0 = dF();
  bool moving = false;
  for (uint32_t t0 = millis(); millis() - t0 < 250;) { setTarget(0, 0); step(); if (abs(dF() - f0) > 40) moving = true; }
  if (dF() > FRONT_TURN_MM + 100) return true;   // a dispărut deja
  if (!moving) return false;
  stateName = "OPP_WAIT";
  uint32_t t0 = millis(), clearSince = 0;
  while (millis() - t0 < OPP_WAIT_MS) {
    setTarget(0, 0); step();
    if (dF() > FRONT_TURN_MM + 120) {
      if (!clearSince) clearSince = millis();
      if (millis() - clearSince > 150) return true;
    } else clearSince = 0;
  }
  return false;
}

static void frontDecision() {
  stateName = "DECIDE";
  stopHard(40);
  if (frontBlockedByOpponent()) return;
  alignFront();
  if (dF() > FRONT_TURN_MM + 100) return;
  if (dHand() > OPEN_MM) {
    pivot(90 * P.hand);
    handPrev = WS_WALL;
    driveDistance(ENTER_AFTER_TURN_MM, V_ADVANCE, true);
  } else if (dOpp() > OPEN_MM) {
    pivot(-90 * P.hand);
    driveDistance(ENTER_SHORT_MM, V_ADVANCE, true);
    handPrev = dHand() < WALL_PRESENT_MM ? WS_WALL : WS_OPEN;
  } else {
    stateName = "DEADEND";
    pivot(180 * P.hand);
    driveDistance(ENTER_SHORT_MM, V_ADVANCE, true);
    handPrev = dHand() < WALL_PRESENT_MM ? WS_WALL : WS_OPEN;
  }
}

static void runMaze() {
  lastWake = xTaskGetTickCount();
  lastMode = -1; finished = false;
  tRun0 = millis(); handPrev = WS_OPEN; exitSince = 0;
  stuckReset();
  while (!finished) {
    step();
    if (exitDetected()) {
      if (doExit()) break;
      handPrev = dHand() < WALL_PRESENT_MM ? WS_WALL : WS_OPEN;
      stuckReset();
      continue;
    }
    if (dF() < FRONT_TURN_MM + 4) {
      frontDecision();
      stuckReset();
      continue;
    }
    if (handOpeningEvent()) {
      stateName = "ADVANCE";
      if (!driveDistance(ADVANCE_TO_TURN_MM, V_ADVANCE, true)) break;
      if (dHand() > OPEN_MM) {
        pivot(90 * P.hand);
        handPrev = WS_WALL;
        stateName = "ENTER";
        if (!driveDistance(ENTER_AFTER_TURN_MM, V_ADVANCE, true)) break;
      }
      stuckReset();
      continue;
    }
    stateName = "FOLLOW";
    followStep();
    stuckCheck();
  }
  stateName = "DONE";
  for (;;) { setTarget(0, 0); brakeReq = true; step(); }
}

// ---------------- Calibrare din Serial ----------------
static int lastTurnTest = 90;

// LED: 1 clipire = mâna stângă, 2 clipiri = mâna dreaptă.
static bool handBlink() {
  uint32_t p = millis() % 1200;
  return (p < 120) || (P.hand < 0 && p >= 250 && p < 370);
}

static void printHelp() {
  Serial.println(
    "\n--- Comenzi (robotul pe masă sau în labirint) ---\n"
    " s   afișează L F R timp de 3 s (pune mâna în fața fiecărui senzor)\n"
    " p   următoarea ordine a senzorilor (până L, F, R arată corect)\n"
    " 1/2 motorul stâng / drept înainte 0,5 s\n"
    " a/b inversează sensul motorului stâng / drept\n"
    " 9/8 pivotare 90 / 180 grade spre stânga\n"
    " +/- timpul ultimei pivotări testate +10 / -10 ms\n"
    " v   măsoară viteza (robot cu fața la un perete aflat la 60-110 cm)\n"
    " f   mergi o celulă (30 cm) pe culoar\n"
    " q/w trim stânga / dreapta (dacă trage într-o parte)\n"
    " l/r mâna stângă / dreaptă\n"
    " g   cursă (pornește în 2 s; oprire: BOOT sau orice tastă)\n"
    " x   afișează parametrii   z  revino la valorile implicite   h  ajutor\n");
}

static void showSensors(uint32_t ms) {
  for (uint32_t t0 = millis(); millis() - t0 < ms; delay(200)) {
    readSnapshot();
    Serial.printf("L %4d   F %4d   R %4d\n", dL(), dF(), dR());
  }
}

static void motorTest(bool left) {
  for (uint32_t t0 = millis(); millis() - t0 < 500;) { setTarget(left ? 0.4f : 0, left ? 0 : 0.4f); step(); }
  stopHard(60);
  Serial.printf("[MOT] %s: dacă roata s-a rotit înapoi apasă '%c'\n", left ? "stâng" : "drept", left ? 'a' : 'b');
}

static void measureSpeed() {
  readSnapshot();
  if (dF() < 500 || dF() > 1150) { Serial.printf("[VIT] F=%d: pune robotul la 60-110 cm de un perete\n", dF()); return; }
  int f0 = 0; uint32_t tA = 0, t0 = millis();
  while (millis() - t0 < 1500 && dF() > 300) {
    setDrive(V_RUN, 0); step();
    if (!tA && millis() - t0 >= 400) { tA = millis(); f0 = dF(); }
  }
  int f1 = dF(); uint32_t tB = millis();
  stopHard(80);
  if (!tA || tB - tA < 300) { Serial.println("[VIT] prea scurt, mai încearcă de mai departe"); return; }
  float v = (f0 - f1) * 1000.0f / (tB - tA);
  if (v < 30 || v > 3000) { Serial.printf("[VIT] valoare ciudată (%.0f mm/s), nesalvată\n", v); return; }
  P.mmps = v; saveParams();
  Serial.printf("[VIT] %.0f mm/s la V_RUN, salvat\n", v);
}

static void serialCommands() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n' || c == '\r' || c == ' ') return;
  if (autoArmed) { autoArmed = false; Serial.println("[AUTO] pornirea automată e anulată; pornești cu 'g'"); }
  running = false;
  lastWake = xTaskGetTickCount();
  switch (c) {
    case 's': showSensors(3000); break;
    case 'p': P.perm = (P.perm + 1) % 6; saveParams(); printParams(); showSensors(1000); break;
    case '1': motorTest(true); break;
    case '2': motorTest(false); break;
    case 'a': P.invL ^= 1; saveParams(); printParams(); break;
    case 'b': P.invR ^= 1; saveParams(); printParams(); break;
    case '9': lastTurnTest = 90; pivot(90); break;
    case '8': lastTurnTest = 180; pivot(180); break;
    case '+': case '-': {
      uint16_t& t = lastTurnTest == 90 ? P.t90 : P.t180;
      t = c == '+' ? t + 10 : (t > 60 ? t - 10 : t);
      saveParams(); Serial.printf("[VIRAJ] t%d = %u ms (testează din nou cu '%c')\n",
                                  lastTurnTest, t, lastTurnTest == 90 ? '9' : '8');
      break;
    }
    case 'v': measureSpeed(); break;
    case 'f': {
      readSnapshot(); lastMode = -1;
      Travel t = travelStart();
      while (travelMm(t) < CELL_MM && dF() > FRONT_TURN_MM) { setDrive(V_RUN, wallSteer()); step(); }
      stopHard(60);
      break;
    }
    case 'q': P.trim -= 0.01f; saveParams(); printParams(); break;
    case 'w': P.trim += 0.01f; saveParams(); printParams(); break;
    case 'l': P.hand = +1; saveParams(); printParams(); break;
    case 'r': P.hand = -1; saveParams(); printParams(); break;
    case 'g':
      Serial.println("[TEST] start în 2 s...");
      delay(2000);
      while (Serial.available()) Serial.read();
      testRun = true; running = true;
      runMaze();
      break;
    case 'x': printParams(); break;
    case 'z': resetParams(); printParams(); break;
    default: printHelp(); break;
  }
  setTarget(0, 0); outL = outR = 0; motorsCoast();
}

void setup() {
  Serial.begin(115200);
#if USE_LED
  pinMode(PIN_LED, OUTPUT);
#endif
  pinMode(PIN_BUTTON, INPUT_PULLUP);
#if USE_START_MODULE
  // Pull-down: fără modul IR conectat robotul nu pornește singur.
  pinMode(PIN_START, INPUT_PULLDOWN);
#if PIN_KILL >= 0
  pinMode(PIN_KILL, INPUT_PULLDOWN);
#endif
#endif
  pinMode(PIN_ML_PH, OUTPUT); pinMode(PIN_MR_PH, OUTPUT);
  pwmSetup(PIN_ML_EN, 0); pwmSetup(PIN_MR_EN, 1);
  motorsCoast();

  prefs.begin("maze3", false);
  loadParams();
  Serial.println("\n=== MazeRunner (simplu) – Robochallenge Maze ===");
  printParams();

#if USE_START_MODULE
  if (digitalRead(PIN_START) == HIGH) {
    irBlocked = true;
    Serial.println("[ERR] START=1 la pornire! Modulul IR e ignorat. Cere arbitrului STOP, apoi repornește robotul.");
  }
#endif

  Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQ);
  for (int i = 0; i < 3; i++) { tofShared.mm[i] = TOF_FAR_MM; tofShared.t[i] = 0; tofShared.seq[i] = 0; }
  // Reîncearcă până pornesc toți senzorii (un fir mișcat se poate repara fără reset).
  while (!initTof()) {
    Serial.println("[ERR] Nu toți senzorii ToF au pornit. Verifică XSHUT/I2C. Reîncerc...");
    delay(1000);
  }
  xTaskCreatePinnedToCore(tofTask, "tof", 16384, nullptr, 3, nullptr, 0);
  delay(100);
  printHelp();
  autoT0 = millis();
}

void loop() {
  static uint32_t btnDown = 0;
  readSnapshot();
  serialCommands();

  bool btn = digitalRead(PIN_BUTTON) == LOW;
  if (btn && !btnDown) btnDown = millis();
  if (!btn && btnDown) {
    uint32_t held = millis() - btnDown; btnDown = 0;
    if (held > 30 && held < 800) {
      P.hand = -P.hand; saveParams(); printParams();
      autoT0 = millis();   // după schimbarea mâinii, numărătoarea reîncepe
    }
  }

  bool go = false;
#if USE_START_MODULE
  static uint8_t hi = 0;
  hi = digitalRead(PIN_START) == HIGH ? hi + 1 : 0;
  go = hi >= 3 && !irBlocked;
#else
  if (autoArmed) {
    uint32_t el = millis() - autoT0;
    static uint32_t lastSec = 0xFFFFFFFF;
    uint32_t sec = (AUTO_START_MS - min(el, (uint32_t)AUTO_START_MS) + 999) / 1000;
    if (sec != lastSec) { lastSec = sec; Serial.printf("[AUTO] start în %lu s (orice tastă = anulează)\n", (unsigned long)sec); }
    go = el >= AUTO_START_MS;
    if (go) autoArmed = false;
  }
#endif
  ledSet(autoArmed ? (millis() / 250) & 1 : handBlink());
  if (go) {
    Serial.println("[GO] Start!");
    ledSet(true);
    running = true;
    runMaze();
  }
  delay(5);
}
