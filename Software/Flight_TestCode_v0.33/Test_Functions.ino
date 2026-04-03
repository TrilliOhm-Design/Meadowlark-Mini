// -------------------------------------------------------------------
//  Ground test functions — before handleSerialCommands so calls resolve
// -------------------------------------------------------------------

void testSensorReadout() {
  Serial.println(F("Live sensor readout -- send Q to stop"));
  Serial.println(F("time_ms,alt_m,accel_G,rawX,rawY,rawZ"));

  while (true) {
    esp_task_wdt_reset();

    float dt = (micros() - lastLoopUs) / 1e6f;
    if (dt <= 0 || dt > 1.0f) dt = 0.01f;
    lastLoopUs = micros();

    baro.read();
    //float altMSL = 44330.0f * (1.0f - powf(baro.getPressure() / baroRefPressure, 0.1902949f));
    //altFiltered  = kalmanUpdate(kAlt, altMSL - altBase, dt);
    float altMSL = baro.getAltitude();
    altFiltered  = kalmanUpdate(kAlt, altMSL - altBase, dt);

    float rawX = imu.readFloatAccelX() * 2;
    float rawY = imu.readFloatAccelY() * 2;
    float rawZ = imu.readFloatAccelZ() * 2;
    float calX, calY, calZ;
    imuApplyCalibration(rawX, rawY, rawZ, calX, calY, calZ);
    float magnitude = sqrtf(calX*calX + calY*calY + calZ*calZ);
    float netAccel  = magnitude - 1.0f;
    accZFilt = kalmanUpdateAccel(kAccZ, netAccel);

    Serial.print(millis());       Serial.print(',');
    Serial.print(altFiltered, 2); Serial.print(',');
    Serial.print(accZFilt, 3);    Serial.print(',');
    Serial.print(rawX, 3);        Serial.print(',');
    Serial.print(rawY, 3);        Serial.print(',');
    Serial.println(rawZ, 3);

    if (Serial.available()) {
      char c = toupper(Serial.read());
      if (c == 'Q') {
        Serial.println(F("Sensor readout stopped."));
        break;
      }
    }
    delay(100);
  }
}

void testBuzzerLed() {
  Serial.println(F("Testing buzzer and LED..."));
  for (int i = 0; i < 3; i++) {
    esp_task_wdt_reset();
    digitalWrite(PIN_LED, HIGH);
    tone(PIN_BUZZER, 2200, 150);
    delay(200);
    digitalWrite(PIN_LED, LOW);
    delay(200);
  }
  tone(PIN_BUZZER, 1800, 150);
  delay(200);
  tone(PIN_BUZZER, 2400, 300);
  delay(400);
  Serial.println(F("Buzzer and LED test complete."));
}

void testPyro(uint8_t channel) {
  uint8_t pin;
  switch (channel) {
    case 1: pin = PIN_PYRO_1; break;
    case 2: pin = PIN_PYRO_2; break;
    case 3: pin = PIN_PYRO_3; break;
    default:
      Serial.println(F("Invalid channel. Use T31, T32, or T33."));
      return;
  }
  Serial.print(F("Firing pyro channel ")); Serial.println(channel);
  digitalWrite(pin, HIGH);
  delay(PYRO_FIRE_MS);
  digitalWrite(pin, LOW);
  Serial.println(F("Done."));
}

void enterState(FlightState s);   // forward declaration needed by testSimFlight

void testSimFlight() {
  Serial.println(F("=== Simulated flight starting ==="));
  Serial.println(F("Send Q at any time to abort"));

  #define SIM_DELAY(ms)                               \
    do {                                              \
      unsigned long _t = millis();                    \
      while (millis() - _t < (ms)) {                  \
        esp_task_wdt_reset();                         \
        if (Serial.available() &&                     \
            toupper(Serial.read()) == 'Q') {          \
          Serial.println(F("Simulation aborted."));   \
          state = PRE_LAUNCH;                         \
          stateEntry = millis();                      \
          return;                                     \
        }                                             \
        delay(10);                                    \
      }                                               \
    } while (0)

  Serial.println(F("[SIM] Liftoff detected"));
  enterState(POWERED_ASCENT);
  SIM_DELAY(2000);

  Serial.println(F("[SIM] Burnout detected"));
  enterState(COAST);
  SIM_DELAY(3000);

  Serial.println(F("[SIM] Apogee detected"));
  enterState(DROGUE_DEPLOY);
  Serial.println(F("[SIM] Drogue would fire here (pyro skipped)"));
  enterState(MAIN_DEPLOY);
  SIM_DELAY(4000);

  Serial.println(F("[SIM] Main deploy altitude reached"));
  Serial.println(F("[SIM] Main would fire here (pyro skipped)"));
  enterState(LANDED);
  SIM_DELAY(3500);

  Serial.println(F("[SIM] Landing confirmed"));
  Serial.println(F("[SIM] NVS save skipped in simulation"));
  tone(PIN_BUZZER, 2200, 150);
  delay(400);
  tone(PIN_BUZZER, 1800, 150);
  delay(600);

  Serial.println(F("=== Simulated flight complete ==="));

  state            = PRE_LAUNCH;
  stateEntry       = millis();
  landedActionDone = false;
  logWriteIdx      = 0;
  logCount         = 0;
  logSkip          = 0;
  logFlags         = 0;
  altApogee        = 0.0f;
  flightMaxG       = 0.0f;

  #undef SIM_DELAY

  Serial.println(F("Returned to PRE_LAUNCH. Ready for flight."));
}