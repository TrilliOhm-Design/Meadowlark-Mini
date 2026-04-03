// -------------------------------------------------------------------
//  Meadowlark Mini Flight Computer
//  Board:      XIAO ESP32-C6 (Arduino IDE — esp32 by Espressif core)
//  IMU:        LSM6DSO32TR  (I2C)
//  Barometer:  MS5611-01BA03 (I2C)
//  Libraries:  SparkFun_LSM6DSO, MS5611 by Rob Tillaart
//
//  Persistent: NVS — max altitude + max G (wear-leveled flash)
//  In-flight:  RAM ring buffer — full log, dumped over Serial
// -------------------------------------------------------------------

//#include <BluetoothSerial.h>
#include <Wire.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <SparkFunLSM6DSO.h>
#include <MS5611.h>

// -- Pins -----------------------------------------------------------
#define PIN_PYRO_1   21
#define PIN_PYRO_2   16
#define PIN_PYRO_3   1
#define PIN_BUZZER   0
#define PIN_BATMON   2
#define PIN_LED      15

// -- Flight thresholds ----------------------------------------------
#define LAUNCH_ACCEL_G        2.5f
#define BURNOUT_ACCEL_G       0.3f
#define APOGEE_DROP_M         5.0f
#define DROUGE_DELAY_S        2.0f
#define MAIN_DEPLOY_ALT_M   150.0f
#define PYRO_FIRE_MS          500

// -- Minimum time guards (ms) ---------------------------------------
#define MIN_TIME_POWERED_MS   500
#define MIN_TIME_COAST_MS     200
#define MIN_TIME_DROGUE_MS   1000
#define MIN_TIME_LANDED_MS   3000

// -- Watchdog timeout -----------------------------------------------
#define WDT_TIMEOUT_S           3

// -- Bluetooth Setup ------------------------------------------------
String device_name = "Meadowlark Mini";

// -------------------------------------------------------------------
//  Kalman filter structs — declared first so visible everywhere
// -------------------------------------------------------------------

struct KalmanState { float x, v, P, Q, R; };

KalmanState kAlt  = {0, 0, 1.0f, 0.1f, 2.0f};
KalmanState kAccZ = {0, 0, 1.0f, 0.05f, 1.5f};

// -------------------------------------------------------------------
//  Flight state enum
// -------------------------------------------------------------------

enum FlightState : uint8_t {
  PRE_LAUNCH     = 0,
  POWERED_ASCENT = 1,
  COAST          = 2,
  DROGUE_DEPLOY  = 3,
  MAIN_DEPLOY    = 4,
  LANDED         = 5
};

// -------------------------------------------------------------------
//  All global variables
// -------------------------------------------------------------------

LSM6DSO         imu;
MS5611          baro;
Preferences     prefs;

FlightState   state            = PRE_LAUNCH;
unsigned long stateEntry       = 0;
bool          landedActionDone = false;

float altBase     = 0.0f;
float altFiltered = 0.0f;
float altApogee   = 0.0f;
float accZFilt    = 0.0f;
float flightMaxG  = 0.0f;

float    nvsMaxAlt      = 0.0f;
float    nvsMaxG        = 0.0f;
uint32_t nvsFlightCount = 0;

unsigned long lastLoopUs = 0;
uint8_t       logSkip    = 0;
uint8_t       logFlags   = 0;

// -- IMU calibration ------------------------------------------------
float imuOffsetX    = 0.0f;
float imuOffsetY    = 0.0f;
float imuOffsetZ    = 0.0f;
float imuScaleX     = 1.0f;
float imuScaleY     = 1.0f;
float imuScaleZ     = 1.0f;
bool  imuCalibrated = false;

// -- Barometer calibration ------------------------------------------
float baroKnownAltM   = 0.0f;
float baroRefPressure = 101325.0f;
bool  baroCalibrated  = false;

// -- IMU calibration sample storage ---------------------------------
struct AccelSample { float x, y, z; };

AccelSample calPos1     = {0, 0, 0};
AccelSample calPos2     = {0, 0, 0};
bool        calPos1Done = false;
bool        calPos2Done = false;

// -------------------------------------------------------------------
//  RAM ring buffer
// -------------------------------------------------------------------

#define LOG_CAPACITY 2048

struct LogRecord {
  uint32_t timestamp_ms;
  float    altitude_m;
  float    accelZ_G;
  uint8_t  state;
  uint8_t  flags;
};

LogRecord logBuffer[LOG_CAPACITY];
uint16_t  logWriteIdx = 0;
uint16_t  logCount    = 0;

// -------------------------------------------------------------------
//  Kalman functions
// -------------------------------------------------------------------

float kalmanUpdate(KalmanState &k, float z, float dt) {
  k.x += k.v * dt;
  k.P += k.Q;
  float K = k.P / (k.P + k.R);
  k.x    += K * (z - k.x);
  k.v    += K * ((z - k.x) / dt);
  k.P    *= (1.0f - K);
  return k.x;
}

float kalmanUpdateAccel(KalmanState &k, float z) {
  k.P += k.Q;
  float K = k.P / (k.P + k.R);
  k.x    += K * (z - k.x);
  k.P    *= (1.0f - K);
  return k.x;
}

// -------------------------------------------------------------------
//  Log functions
// -------------------------------------------------------------------

void logWrite(uint8_t currentState) {
  LogRecord &r   = logBuffer[logWriteIdx];
  r.timestamp_ms = millis();
  r.altitude_m   = altFiltered;
  r.accelZ_G     = accZFilt;
  r.state        = currentState;
  r.flags        = logFlags;
  logWriteIdx    = (logWriteIdx + 1) % LOG_CAPACITY;
  if (logCount < LOG_CAPACITY) logCount++;
}

void logDump() {
  uint16_t total    = logCount;
  uint16_t startIdx = (logCount >= LOG_CAPACITY) ? logWriteIdx : 0;

  Serial.println(F("=== FLIGHT LOG START ==="));
  Serial.println(F("time_ms,alt_m,accel_G,state,flags"));

  for (uint16_t i = 0; i < total; i++) {
    uint16_t slot = (startIdx + i) % LOG_CAPACITY;
    LogRecord &r  = logBuffer[slot];
    Serial.print(r.timestamp_ms);  Serial.print(',');
    Serial.print(r.altitude_m, 2); Serial.print(',');
    Serial.print(r.accelZ_G,   3); Serial.print(',');
    Serial.print(r.state);         Serial.print(',');
    Serial.println(r.flags, BIN);
    esp_task_wdt_reset();
  }

  Serial.println(F("=== FLIGHT LOG END ==="));
  Serial.print(F("Records logged: ")); Serial.println(total);
}

// -------------------------------------------------------------------
//  NVS — flight records
// -------------------------------------------------------------------

void nvsLoad() {
  prefs.begin("flight", false);
  nvsMaxAlt      = prefs.getFloat("maxAlt",  0.0f);
  nvsMaxG        = prefs.getFloat("maxG",    0.0f);
  nvsFlightCount = prefs.getUInt("flights",  0);
  prefs.end();
}

void nvsSave(float maxAlt, float maxG) {
  prefs.begin("flight", false);
  if (maxAlt > nvsMaxAlt) { prefs.putFloat("maxAlt", maxAlt); nvsMaxAlt = maxAlt; }
  if (maxG   > nvsMaxG)   { prefs.putFloat("maxG",   maxG);   nvsMaxG   = maxG;   }
  nvsFlightCount++;
  prefs.putUInt("flights", nvsFlightCount);
  prefs.end();
}

void nvsPrint() {
  Serial.println(F("-- Persistent records ------------------"));
  Serial.print(F("  Flights logged : ")); Serial.println(nvsFlightCount);
  Serial.print(F("  All-time max alt: ")); Serial.print(nvsMaxAlt, 1); Serial.println(F(" m"));
  Serial.print(F("  All-time max G  : ")); Serial.print(nvsMaxG,   2); Serial.println(F(" G"));
  Serial.println(F("----------------------------------------"));
}

void nvsClear() {
  prefs.begin("flight", false);
  prefs.clear();
  prefs.end();
  nvsMaxAlt = 0; nvsMaxG = 0; nvsFlightCount = 0;
  Serial.println(F("NVS records cleared."));
}

// -------------------------------------------------------------------
//  Serial command handler
// -------------------------------------------------------------------

void handleSerialCommands() {
  if (!Serial.available()) return;

  char buf[4] = {0};
  uint8_t len = 0;
  while (Serial.available() && len < 3) {
    buf[len++] = toupper(Serial.read());
    delay(2);
  }

  if (buf[0] == 'D') { logDump();  return; }
  if (buf[0] == 'R') { nvsPrint(); return; }
  if (buf[0] == 'X') { nvsClear(); return; }

  if (buf[0] == 'T') {
    if (state != PRE_LAUNCH) {
      Serial.println(F("Test commands only available in PRE_LAUNCH."));
      return;
    }
    if (buf[1] == '\0') { testPrintMenu();         return; }
    if (buf[1] == '1')  { testSensorReadout();     return; }
    if (buf[1] == '2')  { testBuzzerLed();         return; }
    if (buf[1] == '3')  { testPyro(buf[2] - '0'); return; }
    if (buf[1] == '4')  { testSimFlight();         return; }
    testPrintMenu();
  }

  if (buf[0] == 'C') {
    //if (state != PRE_LAUNCH) {
    //  Serial.println(F("Calibration commands only available in PRE_LAUNCH."));
    //  return;
    //}
    if (buf[1] == 'I') {
      if (buf[2] == '1') imuCalStep1();
      if (buf[2] == '2') imuCalStep2();
      if (buf[2] == '3') imuCalPrint();
      if (buf[2] == '4') imuCalClear();
    }
    if (buf[1] == 'B') {
      if (buf[2] == '1') baroCalRun();
      if (buf[2] == '2') baroCalPrint();
      if (buf[2] == '3') baroCalClear();
    }
    calPrintMenu();
  }
}

void testPrintMenu() {
  Serial.println(F("-- Ground test menu --------------------"));
  Serial.println(F("  T1 = live sensor readout (Q to stop)"));
  Serial.println(F("  T2 = buzzer and LED test"));
  Serial.println(F("  T3 = fire pyro channel (T31 / T32 / T33)"));
  Serial.println(F("  T4 = simulated flight"));
  Serial.println(F("----------------------------------------"));
}

void calPrintMenu() {
  Serial.println(F("-- Calibration menu --------------------"));
  Serial.println(F("  IMU calibration:"));
  Serial.println(F("    CI1 = step 1 (board flat, nose horizontal)"));
  Serial.println(F("    CI2 = step 2 (board upside down)"));
  Serial.println(F("    CI3 = print IMU calibration values"));
  Serial.println(F("    CI4 = clear IMU calibration"));
  Serial.println(F("  Barometer calibration:"));
  Serial.println(F("    CB1 = run barometer calibration"));
  Serial.println(F("    CB2 = print barometer calibration values"));
  Serial.println(F("    CB3 = clear barometer calibration"));
  Serial.println(F("----------------------------------------"));
}

// -------------------------------------------------------------------
//  Sensor init
// -------------------------------------------------------------------

void initSensors() {
  Wire.begin();

  if (!imu.begin()) {
    Serial.println(F("LSM6DSO32 not found!"));
    while (true) { esp_task_wdt_reset(); delay(100); }
  }
  imu.setAccelRange(32);
  imu.setAccelDataRate(104);
  imu.setGyroRange(2000);
  imu.setGyroDataRate(104);

  if (!baro.begin()) {
    Serial.println(F("MS5611 not found!"));
    while (true) { esp_task_wdt_reset(); delay(100); }
  }
  baro.setOversampling(OSR_ULTRA_HIGH);

  Serial.println(F("Calibrating ground altitude..."));
  /*
  double pressSum = 0;
  for (int i = 0; i < 50; i++) {
    esp_task_wdt_reset();
    baro.read();
    pressSum += baro.getPressure();
    delay(40);
  }
  float avgPressure = (float)(pressSum / 50.0);
  float P0          = baroCalibrated ? baroRefPressure : 101325.0f;
  altBase           = 44330.0f * (1.0f - powf(avgPressure / P0, 0.1902949f));
  */
  double altSum = 0;
  for (int i = 0; i < 50; i++) {
    esp_task_wdt_reset();
    baro.read();
    altSum += baro.getAltitude();
    delay(40);
  }
  float altBase = (float)(altSum / 50.0); // Pad Altitude in meters
  kAlt.x            = 0.0f;

  Serial.print(F("Ground MSL : ")); Serial.print(altBase, 3); Serial.println(F(" m"));
  Serial.println();
  //Serial.print(F("Using P0   : ")); Serial.print(P0, 2);      Serial.println(F(" Pa"));
}

// -------------------------------------------------------------------
//  Sensor read
// -------------------------------------------------------------------

void readSensors() {
  float dt = (micros() - lastLoopUs) / 1e6f;
  if (dt <= 0 || dt > 1.0f) dt = 0.01f;
  lastLoopUs = micros();

  baro.read();
  //float P0     = baroCalibrated ? baroRefPressure : 101325.0f;
  //float altMSL = 44330.0f * (1.0f - powf(baro.getPressure() / P0, 0.1902949f));
  float altMSL = baro.getAltitude();
  altFiltered  = kalmanUpdate(kAlt, altMSL - altBase, dt);

  float rawX = imu.readFloatAccelX() * 2;
  float rawY = imu.readFloatAccelY() * 2;
  float rawZ = imu.readFloatAccelZ() * 2;
  float calX, calY, calZ;
  imuApplyCalibration(rawX, rawY, rawZ, calX, calY, calZ);
  float magnitude = sqrtf(calX*calX + calY*calY + calZ*calZ);
  float netAccel  = magnitude - 1.0f;
  accZFilt        = kalmanUpdateAccel(kAccZ, netAccel);

  if (accZFilt > flightMaxG) flightMaxG = accZFilt;
}

// -------------------------------------------------------------------
//  Helpers
// -------------------------------------------------------------------

void enterState(FlightState s) {
  state      = s;
  stateEntry = millis();
  Serial.print(F("[STATE] ")); Serial.println(s);
}

bool timeInState(unsigned long minMs) {
  return (millis() - stateEntry) >= minMs;
}

void firePyro(uint8_t pin, uint8_t flagBit) {
  digitalWrite(pin, HIGH);
  delay(PYRO_FIRE_MS);
  digitalWrite(pin, LOW);
  logFlags |= flagBit;
}

void startChime() {
  tone(PIN_BUZZER, 3150);
  delay(175);
  noTone(PIN_BUZZER);
  delay(75);
  tone(PIN_BUZZER, 2350);
  delay(175);
  noTone(PIN_BUZZER);
  delay(25);
  tone(PIN_BUZZER, 2650);
  delay(175);
  noTone(PIN_BUZZER);
  delay(25);
  tone(PIN_BUZZER, 1975);
  delay(175);
  noTone(PIN_BUZZER);
  delay(25);
  tone(PIN_BUZZER, 1325);
  delay(175);
  noTone(PIN_BUZZER);
}


// -------------------------------------------------------------------
//  State machine
// -------------------------------------------------------------------

void runStateMachine() {
  readSensors();

  switch (state) {

    case PRE_LAUNCH:
      digitalWrite(PIN_LED, (millis() % 1000) < 100);
      handleSerialCommands();
      if (accZFilt > LAUNCH_ACCEL_G) {
        flightMaxG       = 0.0f;
        altApogee        = 0.0f;
        logFlags         = 0;
        logWriteIdx      = 0;
        logCount         = 0;
        logSkip          = 0;
        landedActionDone = false;
        enterState(POWERED_ASCENT);
      }
      break;

    case POWERED_ASCENT:
      digitalWrite(PIN_LED, HIGH);
      if (timeInState(MIN_TIME_POWERED_MS) && accZFilt < BURNOUT_ACCEL_G)
        enterState(COAST);
      break;

    case COAST:
      if (altFiltered > altApogee) altApogee = altFiltered;
      if (timeInState(MIN_TIME_COAST_MS) && (altApogee - altFiltered) > APOGEE_DROP_M)
        enterState(DROGUE_DEPLOY);
      break;

    case DROGUE_DEPLOY:
      firePyro(PIN_PYRO_1, 0x01);
      enterState(MAIN_DEPLOY);
      break;

    case MAIN_DEPLOY:
      if (timeInState(MIN_TIME_DROGUE_MS) && altFiltered < MAIN_DEPLOY_ALT_M) {
        firePyro(PIN_PYRO_2, 0x02);
        enterState(LANDED);
      }
      break;

    case LANDED:
      handleSerialCommands();
      if (timeInState(MIN_TIME_LANDED_MS) && !landedActionDone) {
        landedActionDone = true;
        nvsSave(altApogee, flightMaxG);
        Serial.println(F("Landed. Send D to dump log, R for records."));
        Serial.print(F("  Apogee : ")); Serial.print(altApogee, 1); Serial.println(F(" m"));
        Serial.print(F("  Max G  : ")); Serial.print(flightMaxG, 2); Serial.println(F(" G"));
        tone(PIN_BUZZER, 2200, 150);
        delay(400);
        tone(PIN_BUZZER, 1800, 150);
        delay(600);
      }
      break;
  }

  if (++logSkip >= 4) {
    logSkip = 0;
    logWrite((uint8_t)state);
  }
}

// -------------------------------------------------------------------
//  Setup
// -------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(PIN_PYRO_1, OUTPUT); digitalWrite(PIN_PYRO_1, LOW);
  pinMode(PIN_PYRO_2, OUTPUT); digitalWrite(PIN_PYRO_2, LOW);
  pinMode(PIN_PYRO_3, OUTPUT); digitalWrite(PIN_PYRO_3, LOW);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED,    OUTPUT);

  nvsLoad();
  nvsPrint();

  imuCalLoadFromNvs();
  baroCalLoadFromNvs();

  Serial.println();
  Serial.println(F("Commands available before flight and after landing:"));
  Serial.println(F("  D  = dump flight log as CSV"));
  Serial.println(F("  R  = print all-time records"));
  Serial.println(F("  X  = clear NVS records"));
  Serial.println(F("  T  = ground test menu"));
  Serial.println(F("  C  = calibration menu"));

  esp_task_wdt_config_t wdt_cfg = {
    .timeout_ms     = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  esp_task_wdt_reconfigure(&wdt_cfg);
  esp_task_wdt_add(NULL);

  initSensors();
  startChime();
  lastLoopUs = micros();

  Serial.println(F("Watchdog armed. Ready for flight."));
}

// -------------------------------------------------------------------
//  Loop
// -------------------------------------------------------------------

void loop() {
  esp_task_wdt_reset();
  runStateMachine();
}