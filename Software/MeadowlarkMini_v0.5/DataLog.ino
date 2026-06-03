/* ----------------------------------------------------------------------------
 *  DataLog
 *
 *  Two independent records are kept:
 *
 *   1. FLIGHT SUMMARIES (flash, survive power-off): the last three successful
 *      flights' apogee, peak g-force and duration (launch -> landing).
 *
 *   2. FLIGHT PROFILE (RAM, lost on power-off): a high-rate time series of the
 *      whole flight - altitude, velocity, acceleration and state - that can be
 *      streamed to a PC as CSV after recovery. A short pre-launch ring buffer
 *      is prepended so the liftoff transient is captured.
 *
 *  Lifecycle:  dataLogReset() on STANDBY entry, dataLogBegin() at launch,
 *  dataLogTick() every control tick, dataLogEnd() at landing.
 * --------------------------------------------------------------------------*/

// ----------------------------------------------------------------------------
//  Persistent flight summaries (flash)
// ----------------------------------------------------------------------------
struct FlightSummary {
  float    maxAltAGL;     // m
  float    maxGforce;     // g
  uint32_t durationMs;    // launch detection -> landing
};

struct FlightLog {
  uint32_t      magic;
  uint16_t      version;
  uint16_t      count;        // total flights ever recorded (running number)
  FlightSummary recent[3];    // recent[0] = most recent
};

#define LOG_MAGIC    0x4D4C4731UL   // 'MLG1'
#define LOG_VERSION  1
#define EE_LOG_ADDR  128            // EEPROM offset (Config lives at 0)

FlightLog flightLog;

// ----------------------------------------------------------------------------
//  In-RAM flight profile
// ----------------------------------------------------------------------------
struct LogSample {
  uint32_t t_ms;     // absolute millis() at capture
  float    alt;      // m AGL (filtered)
  float    vel;      // m/s (fused, up +)
  float    accMag;   // m/s^2 total specific force
  float    axial;    // m/s^2 net vertical accel (gravity removed)
  uint8_t  state;    // FlightState at capture
};

const uint16_t LOG_CAPACITY = 4096;          // ~164 s at 25 Hz  (~96 KB RAM)
const uint8_t  LOG_DECIMATE = 4;             // record every Nth tick (100/4=25Hz)
const uint8_t  PRELOG       = 32;            // pre-launch lead-in samples (~1.3s)

static LogSample logBuf[LOG_CAPACITY];
static uint16_t  logCount = 0;
static bool      logActive = false;
static bool      logFull   = false;

static LogSample preBuf[PRELOG];             // pre-launch ring buffer
static uint8_t   preHead = 0;
static uint8_t   preFill = 0;

static float     maxGforce = 0.0f;           // peak g this flight

// ----------------------------------------------------------------------------
//  Flash helpers
// ----------------------------------------------------------------------------
static void saveFlightLog() {
  flightLog.magic = LOG_MAGIC;
  flightLog.version = LOG_VERSION;
  EEPROM.put(EE_LOG_ADDR, flightLog);
  EEPROM.commit();
}

void loadFlightLog() {
  EEPROM.get(EE_LOG_ADDR, flightLog);
  if (flightLog.magic != LOG_MAGIC || flightLog.version != LOG_VERSION) {
    memset(&flightLog, 0, sizeof(flightLog));
    saveFlightLog();
  }
}

void eraseFlightLog() {
  memset(flightLog.recent, 0, sizeof(flightLog.recent));
  flightLog.count = 0;
  saveFlightLog();
}

// ----------------------------------------------------------------------------
//  Sample capture
//  (Built inline rather than via a helper that returns LogSample: a function
//  with a sketch-defined type in its signature would make the Arduino
//  auto-prototype generator emit a prototype ahead of the struct definition.)
// ----------------------------------------------------------------------------

// Called on STANDBY entry: clear everything, ready to capture the pre-launch
// ring buffer.
void dataLogReset() {
  logCount = 0;
  logActive = false;
  logFull = false;
  preHead = 0;
  preFill = 0;
  maxGforce = 0.0f;
}

// Called at launch: flush the pre-launch ring (oldest first) into the main
// buffer, then start live recording.
void dataLogBegin() {
  logCount = 0;
  logFull = false;
  uint8_t idx = (uint8_t)((preHead + PRELOG - preFill) % PRELOG);   // oldest
  for (uint8_t i = 0; i < preFill; i++) {
    LogSample s = preBuf[idx];
    float gNow = s.accMag / G0;
    if (gNow > maxGforce) maxGforce = gNow;
    if (logCount < LOG_CAPACITY) logBuf[logCount++] = s;
    idx = (uint8_t)((idx + 1) % PRELOG);
  }
  logActive = true;
}

// Called every control tick. Fills the pre-launch ring while in STANDBY, and
// appends to the main buffer (tracking peak g) once recording.
void dataLogTick() {
  static uint8_t div = 0;
  if (++div < LOG_DECIMATE) return;
  div = 0;

  LogSample s;
  s.t_ms   = millis();
  s.alt    = altAGL;
  s.vel    = velEst;
  s.accMag = gAccMag;
  s.axial  = gNetVert;
  s.state  = (uint8_t)state;

  if (logActive) {
    float gNow = s.accMag / G0;
    if (gNow > maxGforce) maxGforce = gNow;
    if (logCount < LOG_CAPACITY) logBuf[logCount++] = s;
    else logFull = true;                 // buffer full: stop, keep what we have
  } else if (state == ST_STANDBY) {
    preBuf[preHead] = s;
    preHead = (uint8_t)((preHead + 1) % PRELOG);
    if (preFill < PRELOG) preFill++;
  }
}

// Called at landing: stop recording and store the flight summary to flash.
void dataLogEnd() {
  logActive = false;

  FlightSummary fs;
  fs.maxAltAGL  = maxAltAGL;
  fs.maxGforce  = maxGforce;
  fs.durationMs = (tBoost > 0) ? (millis() - tBoost) : 0;

  flightLog.recent[2] = flightLog.recent[1];   // shift ring down
  flightLog.recent[1] = flightLog.recent[0];
  flightLog.recent[0] = fs;
  if (flightLog.count < 0xFFFF) flightLog.count++;
  saveFlightLog();

  Serial.print(F("[LOG] Flight #")); Serial.print(flightLog.count);
  Serial.print(F(" saved: apogee ")); Serial.print(fs.maxAltAGL, 1);
  Serial.print(F(" m, max ")); Serial.print(fs.maxGforce, 1);
  Serial.print(F(" g, dur ")); Serial.print(fs.durationMs / 1000.0f, 1);
  Serial.print(F(" s, ")); Serial.print(logCount);
  Serial.println(F(" samples in RAM ('download' to retrieve)."));
}

// ----------------------------------------------------------------------------
//  Console output (called from the CONFIG / post-landing console)
// ----------------------------------------------------------------------------
void printFlightSummaries() {
  Serial.println(F("---- LAST 3 FLIGHTS (newest first) ----"));
  Serial.print(F("total flights recorded: ")); Serial.println(flightLog.count);
  for (int i = 0; i < 3; i++) {
    FlightSummary &f = flightLog.recent[i];
    Serial.print(F("  [")); Serial.print(i); Serial.print(F("] "));
    if (f.maxAltAGL == 0 && f.maxGforce == 0 && f.durationMs == 0) {
      Serial.println(F("(empty)"));
      continue;
    }
    Serial.print(F("apogee ")); Serial.print(f.maxAltAGL, 1);
    Serial.print(F(" m   maxG ")); Serial.print(f.maxGforce, 1);
    Serial.print(F(" g   duration ")); Serial.print(f.durationMs / 1000.0f, 1);
    Serial.println(F(" s"));
  }
  Serial.println(F("---------------------------------------"));
}

void dumpFlightCsv() {
  if (logCount == 0) {
    Serial.println(F("[CSV] no flight profile in RAM (none flown since power-up)"));
    return;
  }
  uint32_t t0 = logBuf[0].t_ms;
  Serial.println(F("=== BEGIN CSV ==="));
  Serial.print(F("# meadowlark flight #")); Serial.print(flightLog.count);
  Serial.print(F("  samples=")); Serial.print(logCount);
  Serial.print(F("  rate_hz=")); Serial.print((float)LOOP_HZ / LOG_DECIMATE, 0);
  if (logFull) Serial.print(F("  [TRUNCATED: buffer full]"));
  Serial.println();
  // state legend: 3=STANDBY 4=BOOST 5=COAST 6=DROGUE 7=MAIN 8=LANDED
  Serial.println(F("t_s,state,alt_m,vel_ms,accel_ms2,axial_ms2,g"));
  for (uint16_t i = 0; i < logCount; i++) {
    LogSample &s = logBuf[i];
    Serial.print((s.t_ms - t0) / 1000.0f, 3); Serial.print(',');
    Serial.print(s.state);            Serial.print(',');
    Serial.print(s.alt, 2);           Serial.print(',');
    Serial.print(s.vel, 2);           Serial.print(',');
    Serial.print(s.accMag, 2);        Serial.print(',');
    Serial.print(s.axial, 2);         Serial.print(',');
    Serial.println(s.accMag / G0, 2);
    if ((i & 0x0F) == 0) feedWatchdog();   // keep the WDT fed during long dumps
  }
  Serial.println(F("=== END CSV ==="));
}