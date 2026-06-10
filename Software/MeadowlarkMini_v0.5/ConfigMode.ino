/* ----------------------------------------------------------------------------
 *  ConfigMode
 *
 *  Contents:
 *    - Flight-settings persistence to flash (EEPROM emulation)
 *    - The interactive USB console entered as ST_CONFIG before flight
 *    - Ground-test routines for the buzzer, LED and pyro outputs
 *
 *  Entered when a host is connected at boot (see the prompt in setup()). The
 *  main loop drives this file through serviceConfigMode() while in ST_CONFIG.
 * --------------------------------------------------------------------------*/

// ----------------------------------------------------------------------------
//  Settings persistence (flash via EEPROM emulation)
// ----------------------------------------------------------------------------
void loadDefaults() {
  cfg.magic            = CFG_MAGIC;
  cfg.version          = CFG_VERSION;
  cfg.mainDeployAltAGL = 150.0f;
  cfg.launchAccelG     = 2.5f;
  cfg.burnoutAccelG    = 1.2f;
  cfg.apogeeAltMargin  = 2.0f;
  cfg.apogeeBackupMs   = 1000;
  cfg.pyroPulseMs      = 1000;
  cfg.enableDrogue     = true;
  cfg.enableMain       = true;
  cfg.enableBackup     = true;
  cfg.vbatDivider      = 2.0f;
  cfg.vbatLow          = 3.50f;
  cfg.baroGateVel      = 100.0f;   // ignore baro above ~Mach 0.3 (lag/inaccuracy)
}

void saveConfig() {
  cfg.magic = CFG_MAGIC; cfg.version = CFG_VERSION;
  EEPROM.put(0, cfg);
  EEPROM.commit();
}

void loadConfig() {
  EEPROM.get(0, cfg);
  if (cfg.magic != CFG_MAGIC || cfg.version != CFG_VERSION) {
    loadDefaults();
    saveConfig();
  }
}

// ----------------------------------------------------------------------------
//  USB console state (private to this tab)
// ----------------------------------------------------------------------------
static char     cmdBuf[48];
static uint8_t  cmdLen        = 0;
static uint32_t lastCharMs    = 0;
static bool     streamSensors = false;

// Assemble a command line without blocking the loop. Tolerant of any serial-
// monitor setting: a command is dispatched on CR, LF or CRLF, OR - if the
// monitor sends no line ending at all - once input has been idle briefly.
static bool readSerialLine() {
  while (Serial.available()) {
    char c = Serial.read();
    lastCharMs = millis();
    if (c == '\n' || c == '\r') {           // end of line (any convention)
      if (cmdLen == 0) continue;            // swallow empty lines / CRLF's LF
      cmdBuf[cmdLen] = 0; cmdLen = 0;
      return true;
    }
    if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = c;
  }
  // Fallback for "No line ending": dispatch a buffered line once input settles.
  if (cmdLen > 0 && (millis() - lastCharMs) > 200) {
    cmdBuf[cmdLen] = 0; cmdLen = 0;
    return true;
  }
  return false;
}

void printConfigMenu() {
  Serial.println(F("\n===== MEADOWLARK CONFIG MODE ====="));
  Serial.println(F("help                  this menu"));
  Serial.println(F("get                   show all settings"));
  Serial.println(F("set <key> <value>     change a setting (keys below)"));
  Serial.println(F("    mainalt launchg burnoutg margin backupms pulsems"));
  Serial.println(F("    en_drogue en_main en_backup vbatdiv vbatlow barogate"));
  Serial.println(F("save                  write settings to flash"));
  Serial.println(F("defaults              restore default settings"));
  Serial.println(F("cal                   calibrate sensors (hold still & vertical)"));
  Serial.println(F("status                sensor + battery self-test"));
  Serial.println(F("i2cscan               list devices on the I2C bus"));
  Serial.println(F("stream                toggle live sensor stream"));
  Serial.println(F("test buzzer|led       exercise an output"));
  Serial.println(F("fire <n> CONFIRM      pulse pyro n (1-3) -- DISCONNECT E-MATCHES"));
  Serial.println(F("flights               show last 3 flights (apogee/maxG/duration)"));
  Serial.println(F("download              dump the RAM flight profile as CSV"));
  Serial.println(F("eraselog              clear stored flight summaries"));
  Serial.println(F("arm                   save & begin flight (calibrate -> standby)"));
  Serial.println(F("==================================\n"));
}

void printSettings() {
  Serial.println(F("---- SETTINGS ----"));
  Serial.print(F("mainalt   (m AGL) : ")); Serial.println(cfg.mainDeployAltAGL, 1);
  Serial.print(F("launchg   (g)     : ")); Serial.println(cfg.launchAccelG, 2);
  Serial.print(F("burnoutg  (g)     : ")); Serial.println(cfg.burnoutAccelG, 2);
  Serial.print(F("margin    (m)     : ")); Serial.println(cfg.apogeeAltMargin, 1);
  Serial.print(F("backupms  (ms)    : ")); Serial.println(cfg.apogeeBackupMs);
  Serial.print(F("pulsems   (ms)    : ")); Serial.println(cfg.pyroPulseMs);
  Serial.print(F("en_drogue         : ")); Serial.println(cfg.enableDrogue);
  Serial.print(F("en_main           : ")); Serial.println(cfg.enableMain);
  Serial.print(F("en_backup         : ")); Serial.println(cfg.enableBackup);
  Serial.print(F("vbatdiv           : ")); Serial.println(cfg.vbatDivider, 3);
  Serial.print(F("vbatlow   (V)     : ")); Serial.println(cfg.vbatLow, 2);
  Serial.print(F("barogate  (m/s)   : ")); Serial.println(cfg.baroGateVel, 0);
  Serial.println(F("------------------"));
}

static void applySetting(const char* key, const char* val) {
  float f = atof(val);
  long  l = atol(val);
  if      (!strcmp(key, "mainalt"))   cfg.mainDeployAltAGL = f;
  else if (!strcmp(key, "launchg"))   cfg.launchAccelG     = f;
  else if (!strcmp(key, "burnoutg"))  cfg.burnoutAccelG    = f;
  else if (!strcmp(key, "margin"))    cfg.apogeeAltMargin  = f;
  else if (!strcmp(key, "backupms"))  cfg.apogeeBackupMs   = (uint32_t)l;
  else if (!strcmp(key, "pulsems"))   cfg.pyroPulseMs      = (uint32_t)l;
  else if (!strcmp(key, "en_drogue")) cfg.enableDrogue     = (l != 0);
  else if (!strcmp(key, "en_main"))   cfg.enableMain       = (l != 0);
  else if (!strcmp(key, "en_backup")) cfg.enableBackup     = (l != 0);
  else if (!strcmp(key, "vbatdiv"))   cfg.vbatDivider      = f;
  else if (!strcmp(key, "vbatlow"))   cfg.vbatLow          = f;
  else if (!strcmp(key, "barogate"))  cfg.baroGateVel      = f;
  else { Serial.print(F("[CFG] unknown key: ")); Serial.println(key); return; }
  Serial.print(F("[CFG] ")); Serial.print(key);
  Serial.print(F(" = "));    Serial.println(val);
  Serial.println(F("[CFG] (remember to 'save')"));
}

static void printStatus() {
  sensors_event_t a, g, t;
  imu.getEvent(&a, &g, &t);
  float am = sqrtf(a.acceleration.x * a.acceleration.x +
                   a.acceleration.y * a.acceleration.y +
                   a.acceleration.z * a.acceleration.z);
  bool  baroOk = (baro.read() == MS5611_READ_OK);
  float vb = readBattery();

  Serial.println(F("---- SELF TEST ----"));
  Serial.print(F("accel |a| (m/s^2) : ")); Serial.print(am, 2);
  Serial.println(fabsf(am - G0) < 3.0f ? F("   OK") : F("   CHECK"));
  Serial.print(F("gyro xyz (rad/s)  : "));
  Serial.print(g.gyro.x, 3); Serial.print(' ');
  Serial.print(g.gyro.y, 3); Serial.print(' ');
  Serial.println(g.gyro.z, 3);
  Serial.print(F("imu temp (C)      : ")); Serial.println(t.temperature, 1);
  if (baroOk) {
    Serial.print(F("baro (hPa)        : ")); Serial.print(baro.getPressure(), 2);
    Serial.print(F("   alt(m)=")); Serial.println(pressureToAltitude(baro.getPressure(), groundPressure), 1);
  } else {
    Serial.println(F("baro              : FAIL"));
  }
  Serial.print(F("battery (V)       : ")); Serial.print(vb, 2);
  Serial.println(vb >= cfg.vbatLow ? F("   OK") : F("   LOW"));
  Serial.print(F("up vector (cal)   : ["));
  Serial.print(upHat[0], 3); Serial.print(','); Serial.print(upHat[1], 3);
  Serial.print(','); Serial.print(upHat[2], 3); Serial.println(F("]"));
  Serial.println(F("-------------------"));
}

static void streamSensorTelemetry() {
  static uint32_t last = 0;
  if (millis() - last < 100) return;          // 10 Hz
  last = millis();
  sensors_event_t a, g, t;
  imu.getEvent(&a, &g, &t);
  float am  = sqrtf(a.acceleration.x * a.acceleration.x +
                    a.acceleration.y * a.acceleration.y +
                    a.acceleration.z * a.acceleration.z);
  float alt = (baro.read() == MS5611_READ_OK)
                ? pressureToAltitude(baro.getPressure(), groundPressure) : NAN;
  Serial.print(F("ax:")); Serial.print(a.acceleration.x, 1);
  Serial.print(F(" ay:")); Serial.print(a.acceleration.y, 1);
  Serial.print(F(" az:")); Serial.print(a.acceleration.z, 1);
  Serial.print(F(" |a|:")); Serial.print(am, 1);
  Serial.print(F(" alt:")); Serial.print(alt, 1);
  Serial.print(F(" vb:")); Serial.println(readBattery(), 2);
}

// ----------------------------------------------------------------------------
//  I2C bus scan (diagnostics) - LSM6DSO32 is 0x6A/0x6B, MS5611 is 0x76/0x77
// ----------------------------------------------------------------------------
static void i2cScan() {
  Serial.println(F("[I2C] scanning D4/D5 bus..."));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  device at 0x"));
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) Serial.println(F("  none found - check wiring / pull-ups / power"));
  else { Serial.print(F("  ")); Serial.print(found); Serial.println(F(" device(s)")); }
}

// ----------------------------------------------------------------------------
//  Ground tests
// ----------------------------------------------------------------------------
static void groundTestLed() {
  Serial.println(F("[TEST] LED blink"));
  for (int i = 0; i < 10; i++) {
    digitalWrite(PIN_USER_LED, i & 1);
    feedWatchdog();
    delay(100);
  }
  digitalWrite(PIN_USER_LED, LOW);
}

// Direct, non-one-shot pyro pulse for bench testing (servicePyro turns it off).
static void groundTestFire(uint8_t ch) {
  Serial.print(F("[TEST] !! Pulsing pyro ")); Serial.print(ch + 1);
  Serial.print(F(" for ")); Serial.print(cfg.pyroPulseMs);
  Serial.println(F(" ms -- ENSURE E-MATCHES ARE DISCONNECTED"));
  setPyro(ch, true);
  pyroOffAt[ch] = millis() + cfg.pyroPulseMs;
}

// ----------------------------------------------------------------------------
//  Command dispatch
// ----------------------------------------------------------------------------
static void handleConfigCommand(char* line) {
  char* cmd = strtok(line, " \t");
  if (!cmd) return;

  if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) { printConfigMenu(); return; }
  if (!strcmp(cmd, "get"))      { printSettings(); return; }
  if (!strcmp(cmd, "save"))     { saveConfig(); Serial.println(F("[CFG] saved to flash")); return; }
  if (!strcmp(cmd, "defaults")) { loadDefaults(); Serial.println(F("[CFG] defaults loaded (not yet saved)")); printSettings(); return; }
  if (!strcmp(cmd, "cal"))      { calibrate(); return; }
  if (!strcmp(cmd, "status"))   { printStatus(); return; }
  if (!strcmp(cmd, "i2cscan"))  { i2cScan(); return; }
  if (!strcmp(cmd, "flights"))  { printFlightSummaries(); return; }
  if (!strcmp(cmd, "download") || !strcmp(cmd, "csv")) { dumpFlightCsv(); return; }
  if (!strcmp(cmd, "eraselog")) { eraseFlightLog(); Serial.println(F("[LOG] flight summaries erased")); return; }
  if (!strcmp(cmd, "stream")) {
    streamSensors = !streamSensors;
    Serial.print(F("[CFG] stream ")); Serial.println(streamSensors ? F("ON") : F("OFF"));
    return;
  }
  if (!strcmp(cmd, "arm") || !strcmp(cmd, "fly")) {
    saveConfig();
    streamSensors = false;
    for (uint8_t i = 0; i < 3; i++) { pyroFired[i] = false; pyroOffAt[i] = 0; setPyro(i, false); }
    Serial.println(F("[CFG] armed - saving & calibrating on the pad..."));
    state = ST_CALIBRATE;
    return;
  }
  if (!strcmp(cmd, "test")) {
    char* sub = strtok(NULL, " \t");
    if      (sub && !strcmp(sub, "buzzer")) { Serial.println(F("[TEST] buzzer")); playTune(TUNE_READY, sizeof(TUNE_READY) / sizeof(Note)); }
    else if (sub && !strcmp(sub, "led"))    { groundTestLed(); }
    else Serial.println(F("[TEST] usage: test buzzer | test led"));
    return;
  }
  if (!strcmp(cmd, "fire")) {
    char* sn   = strtok(NULL, " \t");
    char* conf = strtok(NULL, " \t");
    int ch = sn ? atoi(sn) : 0;
    if (ch < 1 || ch > 3) { Serial.println(F("[TEST] usage: fire <1-3> CONFIRM")); return; }
    if (!conf || strcmp(conf, "CONFIRM")) {
      Serial.println(F("[TEST] DANGER: this energises a pyro output."));
      Serial.println(F("[TEST] Disconnect e-matches, then: fire <n> CONFIRM"));
      return;
    }
    groundTestFire((uint8_t)(ch - 1));
    return;
  }
  if (!strcmp(cmd, "set")) {
    char* key = strtok(NULL, " \t");
    char* val = strtok(NULL, " \t");
    if (!key || !val) { Serial.println(F("[CFG] usage: set <key> <value>")); return; }
    applySetting(key, val);
    return;
  }
  Serial.print(F("[CFG] unknown command: ")); Serial.println(cmd);
  Serial.println(F("[CFG] type 'help'"));
}

// ----------------------------------------------------------------------------
//  Entry point called from the main loop while in ST_CONFIG.
// ----------------------------------------------------------------------------
void serviceConfigMode() {
  if (readSerialLine()) handleConfigCommand(cmdBuf);
  if (streamSensors)    streamSensorTelemetry();
}