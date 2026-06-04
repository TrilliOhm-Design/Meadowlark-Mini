/* ----------------------------------------------------------------------------
 *  Meadowlark Mini Flight Computer
 *  Target  : Seeed Studio XIAO RP2040  (arduino-pico / Earle Philhower core)
 *  Sensors : LSM6DSO32  6-axis IMU (+/-32 g)   -  flight event triggering
 *            MS561101BA03-50 (MS5611) barometer -  altitude / apogee
 *  Outputs : Buzzer, 3 pyro channels, user LED
 *
 *  This firmware:
 *    1. Runs a 1-D Kalman filter (altitude / velocity / accel-bias) driven by
 *       the IMU for low-latency event timing. The barometer is decoupled
 *       during boost and high-speed flight (it lags and is Mach-corrupted) and
 *       only corrects the filter once slow - i.e. near apogee and on descent.
 *       See Estimator.ino.
 *    2. Normalises the IMU at power-up so the board may be mounted in ANY
 *       orientation - it learns the gravity vector on the pad and works in the
 *       rocket's "up" axis from then on.
 *    3. Runs the RP2040 hardware watchdog so a hang in flight forces a reboot.
 *    4. Plays a chime once the unit has calibrated, is armed, and is sitting in
 *       STANDBY ready for flight.
 *    5. Enters an interactive CONFIG mode over USB (send any key within 3 s of
 *       power-up) to calibrate sensors, edit & save flight variables to flash,
 *       and ground-test the buzzer / LED / pyro outputs before flight.
 *    6. Logs each flight: the last three flights' apogee / max-g / duration are
 *       kept in flash, and a full flight profile is recorded in RAM and can be
 *       downloaded as CSV over USB after recovery (see DataLog.ino).
 *
 *  Sketch tabs:
 *    Calibration.ino       - on-pad sensor / orientation calibration
 *    Estimator.ino         - IMU + Kalman filter (altitude / velocity)
 *    ConfigMode.ino        - USB console, settings persistence, ground tests
 *    FlightStateMachine.ino- flight state transitions & event triggering
 *    DataLog.ino           - flight summaries (flash) + RAM profile / CSV
 *
 *  Pinout:
 *    D4  = SDA
 *    D5  = SCL
 *    D3  = Pyro 1   (drogue / apogee)
 *    D6  = Pyro 2   (main  / altitude)
 *    D1  = Pyro 3   (apogee backup)
 *    A0  = Buzzer
 *    A2  = Battery mon
 *    D10 = User LED
 *
 * --------------------------------------------------------------------------*/

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_LSM6DSO32.h>
#include "MS5611.h"

// ----------------------------------------------------------------------------
//  Pin assignments
// ----------------------------------------------------------------------------
#define PIN_SDA       D4
#define PIN_SCL       D5
#define PIN_PYRO_1     D3      // Drogue   - fires at apogee
#define PIN_PYRO_2     D6      // Main     - fires at MAIN_DEPLOY_ALT on descent
#define PIN_PYRO_3     D1      // Backup   - apogee redundancy
#define PIN_BUZZER    A0
#define PIN_VBAT      A2
#define PIN_USER_LED  D10     // external user LED (note: core's PIN_LED is the
                              // XIAO's onboard LED on GP17 - don't reuse it)

const uint8_t PYRO_PINS[3] = { PIN_PYRO_1, PIN_PYRO_2, PIN_PYRO_3 };

// ----------------------------------------------------------------------------
//  Flight configuration  -  TUNE THESE FOR YOUR VEHICLE
// ----------------------------------------------------------------------------
const float    G0                  = 9.80665f;  // standard gravity (m/s^2)

// --- Loop timing. 100 Hz for fast event reaction; the barometer is decoupled
//     during high-speed flight (see Estimator.ino) so it never bottlenecks the
//     loop when it matters. The estimator uses the measured dt, not this value.
const uint32_t LOOP_HZ             = 100;                // main control rate
const uint32_t LOOP_DT_US         = 1000000UL / LOOP_HZ;
const float    DT                  = 1.0f / LOOP_HZ;     // nominal step (s)

// --- Watchdog (RP2040 hardware WDT, max ~8.3 s)
const uint32_t WDT_TIMEOUT_MS     = 4000;

// --- Calibration (must be stationary & vertical on the pad)
const uint16_t CAL_SAMPLES         = 400;    // samples averaged for bias/gravity
const float    CAL_MOTION_TOL      = 1.5f;   // m/s^2 spread allowed during cal

// --- Detection tuning that is NOT user-editable (compile-time)
//     Sample counts are sized for the 100 Hz loop (~10 ms/sample).
const uint8_t  LAUNCH_CONFIRM      = 8;          // ~80 ms sustained over threshold
const uint8_t  BURNOUT_CONFIRM     = 12;         // ~120 ms
const uint32_t MIN_BOOST_MS        = 150;        // ms ignore burnout before this
const float    APOGEE_VEL_MS       = 0.0f;       // m/s descending when v < this
const uint8_t  APOGEE_CONFIRM      = 16;         // ~160 ms 

// --- Landing detection
const float    LANDED_VEL_MS       = 1.0f;       // |v| under this ...
const uint32_t LANDED_HOLD_MS      = 5000;       // ... for this long => landed

// --- Battery ADC scaling (the divider ratio itself is a user setting)
const float    ADC_VREF            = 3.30f;      // ADC reference (V)
const uint16_t ADC_MAX             = 4095;       // 12-bit

// --- IMU low-pass cut-offs (Hz). Light filtering only: the Kalman filter does
//     the heavy lifting, and we keep IMU latency low for fast event detection.
const float    LP_FC_ACCEL         = 30.0f;
const float    LP_FC_GYRO          = 30.0f;
// (Altitude/velocity are produced by the Kalman filter in Estimator.ino, which
//  fuses the IMU with the barometer - the latter only at low speed.)

// --- Barometer oversampling: balance precision vs. conversion time.
//     OSR_STANDARD (~4 ms/read) fits the 10 ms loop budget at 100 Hz.
const osr_t    BARO_OSR            = OSR_STANDARD;

// ----------------------------------------------------------------------------
//  User-tunable flight settings
//  These are editable over USB in CONFIG mode and persisted to flash (EEPROM
//  emulation) so they survive power cycles. Defaults are applied on first boot
//  or whenever the stored MAGIC/VERSION does not match.
// ----------------------------------------------------------------------------
struct Config {
  uint32_t magic;
  uint16_t version;
  float    mainDeployAltAGL;   // m AGL, main chute fires on descent below this
  float    launchAccelG;       // g, |a| above this => liftoff
  float    burnoutAccelG;      // g, |a| below this => motor burnout
  float    apogeeAltMargin;    // m below peak required to confirm apogee
  uint32_t apogeeBackupMs;     // delay after apogee before backup channel fires
  uint32_t pyroPulseMs;        // e-match firing pulse length
  bool     enableDrogue;       // Pyro 1 at apogee
  bool     enableMain;         // Pyro 2 at main-deploy altitude
  bool     enableBackup;       // Pyro 3 apogee backup
  float    vbatDivider;        // (R1+R2)/R2 of the battery divider
  float    vbatLow;            // low-battery warning threshold (V)
  float    baroGateVel;        // m/s; baro is ignored above this speed (Mach/lag)
};

#define CFG_MAGIC    0x4D4C4B31UL   // 'MLK1'
#define CFG_VERSION  2              // bumped when the Config layout changes

Config cfg;
// The struct and the live `cfg` instance are declared here (the main tab) so
// the flight code can read them. loadDefaults()/loadConfig()/saveConfig() live
// in ConfigMode.ino.

// ----------------------------------------------------------------------------
//  Sensors
// ----------------------------------------------------------------------------
Adafruit_LSM6DSO32 imu;
MS5611             baro(0x77, &Wire);   // 0x76 if CSB tied high

// ----------------------------------------------------------------------------
//  Flight state machine
// ----------------------------------------------------------------------------
enum FlightState {
  ST_BOOT,        // power-up, hardware init
  ST_CONFIG,      // USB connected: calibrate / set variables / ground test
  ST_CALIBRATE,   // learning bias / gravity vector / ground level
  ST_STANDBY,     // armed, on the pad, waiting for launch  (chime here)
  ST_BOOST,       // motor burning
  ST_COAST,       // burnout -> apogee
  ST_DROGUE,      // apogee passed, drogue out, descending
  ST_MAIN,        // main chute deployed
  ST_LANDED       // touchdown, report apogee
};
FlightState state = ST_STANDBY; // go into ST_BOOT For SPI NAND Conig in future

// ----------------------------------------------------------------------------
//  First-order low-pass (exponential) filter
// ----------------------------------------------------------------------------
struct LowPass {
  float y = 0.0f;
  float a = 1.0f;
  bool  init = false;
  void  setCutoff(float fc, float dt) {        // a = dt / (RC + dt)
    float rc = 1.0f / (2.0f * PI * fc);
    a = dt / (rc + dt);
  }
  float update(float x) {
    if (!init) { y = x; init = true; }
    y += a * (x - y);
    return y;
  }
  void reset() { init = false; y = 0.0f; }
};

LowPass lpAx, lpAy, lpAz;     // accelerometer (m/s^2)
LowPass lpGx, lpGy, lpGz;     // gyro (rad/s)

// ----------------------------------------------------------------------------
//  Estimator / orientation state
// ----------------------------------------------------------------------------
float  upHat[3]    = {0, 0, 1};   // unit gravity vector at rest = rocket "up"
float  gyroBias[3] = {0, 0, 0};   // rad/s, removed in flight
float  groundPressure = 1013.25f; // hPa baseline captured on the pad

float  altAGL      = 0.0f;        // Kalman altitude above ground (m)
float  velEst      = 0.0f;        // Kalman vertical velocity (m/s, up +)
float  maxAltAGL   = 0.0f;        // peak altitude this flight (m)
float  vbat        = 0.0f;

// detection counters / timestamps
uint8_t  launchCount = 0, burnoutCount = 0, apogeeCount = 0;
uint32_t tBoost = 0, tApogee = 0, tLandedStart = 0;
bool     apogeeBackupArmed = false;

// pyro channel non-blocking fire timers (0 = idle)
uint32_t pyroOffAt[3]  = {0, 0, 0};
bool     pyroFired[3]  = {false, false, false};

uint32_t lastLoopUs = 0;

// Estimator outputs shared with the state machine.
float  gAccMag  = 0.0f;   // total specific-force magnitude (m/s^2)
float  gNetVert = 0.0f;   // vertical acceleration, gravity removed (m/s^2)

// ----------------------------------------------------------------------------
//  Buzzer - non-blocking tune sequencer
// ----------------------------------------------------------------------------
struct Note { uint16_t freq; uint16_t dur; };   // freq 0 = silent rest

const Note* tunePtr = nullptr;
uint16_t    tuneLen = 0, tuneIdx = 0;
uint32_t    noteStartMs = 0;
bool        tunePlaying = false;

// READY / STANDBY chime: ascending C-E-G-C, "armed and happy"
const Note TUNE_READY[] = {
  {523, 120}, {659, 120}, {784, 120}, {1047, 260}
};
// Liftoff acknowledge (brief, won't block the control loop)
const Note TUNE_LAUNCH[] = { {1047, 80}, {1319, 80} };
// Apogee
const Note TUNE_APOGEE[] = { {1319, 120}, {988, 120} };
// Error / not armed: low repeated buzz
const Note TUNE_ERROR[]  = { {220, 200}, {0, 120}, {220, 200}, {0, 120}, {220, 400} };

void startCurrentNote(uint32_t now) {
  noteStartMs = now;
  uint16_t f = tunePtr[tuneIdx].freq;
  if (f > 0) tone(PIN_BUZZER, f);
  else       noTone(PIN_BUZZER);
}

void playTune(const Note* t, uint16_t len) {
  tunePtr = t; tuneLen = len; tuneIdx = 0;
  tunePlaying = (len > 0);
  if (tunePlaying) startCurrentNote(millis());
}

void updateBuzzer() {
  if (!tunePlaying) return;
  uint32_t now = millis();
  if (now - noteStartMs >= tunePtr[tuneIdx].dur) {
    tuneIdx++;
    if (tuneIdx >= tuneLen) { tunePlaying = false; noTone(PIN_BUZZER); return; }
    startCurrentNote(now);
  }
}

// Altitude "beep-out" for after landing: builds a tune of per-digit beeps,
// e.g. 342 m -> 3 beeps . 4 beeps . 2 beeps. Zero digit -> one long low beep.
Note beepOutBuf[40];
void buildAltitudeBeepOut(uint32_t meters) {
  uint16_t n = 0;
  char digits[6];
  int d = snprintf(digits, sizeof(digits), "%lu", (unsigned long)meters);
  for (int i = 0; i < d && n < 36; i++) {
    int val = digits[i] - '0';
    if (val == 0) {
      beepOutBuf[n++] = {330, 500};            // long low beep = zero
    } else {
      for (int b = 0; b < val && n < 38; b++) {
        beepOutBuf[n++] = {880, 120};
        beepOutBuf[n++] = {0,   120};
      }
    }
    beepOutBuf[n++] = {0, 600};                // gap between digits
  }
  playTune(beepOutBuf, n);
}

// ----------------------------------------------------------------------------
//  Helpers
// ----------------------------------------------------------------------------
void setPyro(uint8_t ch, bool on) {
  digitalWrite(PYRO_PINS[ch], on ? HIGH : LOW);
}

// Fire a pyro channel as a timed pulse (non-blocking).
void firePyro(uint8_t ch) {
  if (pyroFired[ch]) return;          // one-shot per flight
  pyroFired[ch] = true;
  setPyro(ch, true);
  pyroOffAt[ch] = millis() + cfg.pyroPulseMs;
  Serial.print(F("[PYRO] channel ")); Serial.print(ch + 1); Serial.println(F(" FIRED"));
}

void servicePyro() {                  // turn channels back off after the pulse
  uint32_t now = millis();
  for (uint8_t i = 0; i < 3; i++) {
    if (pyroOffAt[i] && now >= pyroOffAt[i]) {
      setPyro(i, false);
      pyroOffAt[i] = 0;
    }
  }
}

float readBattery() {
  uint16_t raw = analogRead(PIN_VBAT);
  return (raw * ADC_VREF / ADC_MAX) * cfg.vbatDivider;
}

float pressureToAltitude(float p_hPa, float p0_hPa) {
  // International barometric formula (troposphere), result in metres AGL.
  return 44330.0f * (1.0f - powf(p_hPa / p0_hPa, 0.1902949f));
}

void feedWatchdog() { rp2040.wdt_reset(); }

// ----------------------------------------------------------------------------
//  setup()
// ----------------------------------------------------------------------------
void setup() {
  pinMode(PIN_PYRO_1, OUTPUT); digitalWrite(PIN_PYRO_1, LOW);
  pinMode(PIN_PYRO_2, OUTPUT); digitalWrite(PIN_PYRO_2, LOW);
  pinMode(PIN_PYRO_3, OUTPUT); digitalWrite(PIN_PYRO_3, LOW);
  pinMode(PIN_USER_LED, OUTPUT); digitalWrite(PIN_USER_LED, LOW);
  pinMode(PIN_BUZZER, OUTPUT);

  analogReadResolution(12);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 1500)) { /* brief wait for USB */ }
  Serial.println(F("\n=== Meadowlark Mini Flight Computer ==="));

  // --- Load saved flight settings + flight summaries from flash
  EEPROM.begin(512);
  loadConfig();
  loadFlightLog();

  // --- I2C bus (default Wire maps to D4/D5 on the XIAO RP2040)
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.begin();
  Wire.setClock(400000);

  bool ok = true;

  // --- IMU. The LSM6DSO32 lives at 0x6A or 0x6B depending on the SA0/SDO pin.
  //     This board straps it to 0x6B (the SparkFun-style default), so try the
  //     default 0x6A first, then 0x6B, rather than assuming one address.
  bool imuOk = imu.begin_I2C(0x6B, &Wire);   // 0x6A

  if (!imuOk) {
    Serial.println(F("[ERR] LSM6DSO32 not found at 0x6B!"));
    ok = false;
  } else {
    imu.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);     // high-g for boost
    imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_208_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_208_HZ);
  }

  // --- Barometer
  if (!baro.begin()) {
    Serial.println(F("[ERR] MS5611 not found!"));
    ok = false;
  } else {
    baro.setOversampling(BARO_OSR);
  }

  // --- Configure IMU low-pass filters (the Kalman filter handles alt/vel)
  lpAx.setCutoff(LP_FC_ACCEL, DT); lpAy.setCutoff(LP_FC_ACCEL, DT); lpAz.setCutoff(LP_FC_ACCEL, DT);
  lpGx.setCutoff(LP_FC_GYRO, DT);  lpGy.setCutoff(LP_FC_GYRO, DT);  lpGz.setCutoff(LP_FC_GYRO, DT);

  // --- Start the hardware watchdog (after slow init, before the flight loop)
  rp2040.wdt_begin(WDT_TIMEOUT_MS);

  if (!ok) {
    Serial.println(F("[ERR] Sensor init failed - NOT arming."));
    Serial.println(F("[ERR] Send any key to enter CONFIG mode for diagnostics."));
    playTune(TUNE_ERROR, sizeof(TUNE_ERROR) / sizeof(Note));
    // Flash/buzz the fault, but stay responsive: a keystroke drops into CONFIG
    // (e.g. to run 'i2cscan' / 'status') instead of locking the user out.
    while (!Serial.available()) {
      digitalWrite(PIN_USER_LED, (millis() / 200) & 1);
      updateBuzzer();
      if (!tunePlaying) playTune(TUNE_ERROR, sizeof(TUNE_ERROR) / sizeof(Note));
      feedWatchdog();
      delay(10);
    }
    while (Serial.available()) Serial.read();   // flush
    noTone(PIN_BUZZER);
    enterState(ST_CONFIG);
    return;                                     // fall through to loop() in CONFIG
  }

  vbat = readBattery();
  Serial.print(F("[BATT] ")); Serial.print(vbat, 2); Serial.println(F(" V"));

  // --- CONFIG vs FLIGHT decision: offer a short window to enter CONFIG mode.
  //     If a USB host is connected and the user sends any character, drop into
  //     the interactive setup. On battery (no host), nothing arrives and the
  //     unit proceeds straight to on-pad calibration and arming.
  Serial.println(F("\n>>> Send any key within 5 s to enter CONFIG mode"));
  Serial.println(F("    (or send a key any time while in STANDBY on the pad)."));
  bool wantConfig = false;
  uint32_t tWin = millis();
  while (millis() - tWin < 5000) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();   // flush
      wantConfig = true;
      break;
    }
    digitalWrite(PIN_USER_LED, (millis() / 100) & 1);  // fast blink = waiting
    feedWatchdog();
    delay(10);
  }
  digitalWrite(PIN_USER_LED, LOW);

  if (wantConfig) enterState(ST_CONFIG);
  else            state = ST_CALIBRATE;
}

// ----------------------------------------------------------------------------
//  Telemetry (Serial)
// ----------------------------------------------------------------------------
void telemetry() {
  // Stay quiet after landing so a CSV download is not interleaved with noise.
  if (state == ST_LANDED) return;
  static uint32_t last = 0;
  if (millis() - last < 100) return;   // 10 Hz
  last = millis();
  Serial.print(F("S:"));   Serial.print(state);
  Serial.print(F(" alt:")); Serial.print(altAGL, 1);
  Serial.print(F(" v:"));   Serial.print(velEst, 1);
  Serial.print(F(" max:")); Serial.print(maxAltAGL, 1);
  Serial.print(F(" |a|:")); Serial.print(gAccMag, 1);
  Serial.print(F(" vb:"));  Serial.print(vbat, 2);
  Serial.println();
}

// ----------------------------------------------------------------------------
//  loop()  -  fixed-rate scheduler
// ----------------------------------------------------------------------------
void loop() {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastLoopUs) < LOOP_DT_US) {
    updateBuzzer();        // keep the tune player smooth between ticks
    return;
  }
  lastLoopUs = nowUs;

  feedWatchdog();          // pet the dog every control tick

  // --- CONFIG mode: serve the USB console; no flight logic runs here.
  //     The console + ground-test implementation lives in ConfigMode.ino.
  if (state == ST_CONFIG) {
    serviceConfigMode();   // parse serial commands, run sensor stream
    servicePyro();         // ends any ground-test pyro pulse
    updateBuzzer();
    digitalWrite(PIN_USER_LED, (millis() / 500) & 1);  // slow blink = config
    return;
  }

  // --- Allow entering CONFIG from the pad at any time: if a USB host sends a
  //     byte while we are armed in STANDBY, drop into the console. This is the
  //     reliable way in if the 5 s boot window was missed.
  if (state == ST_STANDBY && Serial.available()) {
    while (Serial.available()) Serial.read();   // flush
    enterState(ST_CONFIG);
    return;
  }

  // --- Run calibration / arming sequence here so it shares the WDT-fed loop.
  if (state == ST_CALIBRATE) {
    if (calibrate()) {
      altAGL = 0; velEst = 0; maxAltAGL = 0;
      estimatorReset();          // zero the Kalman filter on the pad
      enterState(ST_STANDBY);
    } else {
      playTune(TUNE_ERROR, sizeof(TUNE_ERROR) / sizeof(Note));
      delay(800);          // brief pause, then retry (WDT fed above)
    }
    return;
  }

  updateEstimator();
  runStateMachine();
  dataLogTick();           // capture the flight profile into RAM
  servicePyro();
  updateBuzzer();
  telemetry();

  // After touchdown, serve the USB console so the flight can be downloaded as
  // CSV (the RAM profile is intact until power-off; summaries are in flash).
  if (state == ST_LANDED) serviceConfigMode();

  // Battery sampled slowly; warn (don't disarm) on low voltage in STANDBY.
  static uint32_t vbatNext = 0;
  if (millis() >= vbatNext) {
    vbatNext = millis() + 2000;
    vbat = readBattery();
    if (state == ST_STANDBY && vbat < cfg.vbatLow) {
      Serial.print(F("[WARN] Low battery: ")); Serial.print(vbat, 2); Serial.println(F(" V"));
    }
  }
}