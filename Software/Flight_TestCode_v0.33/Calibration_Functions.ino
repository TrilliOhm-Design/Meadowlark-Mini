// -------------------------------------------------------------------
//  IMU calibration
// -------------------------------------------------------------------

AccelSample imuAverageSample(uint16_t samples) {
  AccelSample avg = {0, 0, 0};
  for (uint16_t i = 0; i < samples; i++) {
    esp_task_wdt_reset();
    avg.x += imu.readFloatAccelX() * 2;
    avg.y += imu.readFloatAccelY() * 2;
    avg.z += imu.readFloatAccelZ() * 2;
    delay(10);
  }
  avg.x /= samples;
  avg.y /= samples;
  avg.z /= samples;
  return avg;
}

void imuCalSaveToNvs() {
  prefs.begin("imu_cal", false);
  prefs.putFloat("offX", imuOffsetX);
  prefs.putFloat("offY", imuOffsetY);
  prefs.putFloat("offZ", imuOffsetZ);
  prefs.putFloat("sclX", imuScaleX);
  prefs.putFloat("sclY", imuScaleY);
  prefs.putFloat("sclZ", imuScaleZ);
  prefs.putBool("done",  true);
  prefs.end();
  imuCalibrated = true;
  Serial.println(F("IMU calibration saved to NVS."));
}

void imuCalLoadFromNvs() {
  prefs.begin("imu_cal", true);
  bool done = prefs.getBool("done", false);
  if (done) {
    imuOffsetX    = prefs.getFloat("offX", 0.0f);
    imuOffsetY    = prefs.getFloat("offY", 0.0f);
    imuOffsetZ    = prefs.getFloat("offZ", 0.0f);
    imuScaleX     = prefs.getFloat("sclX", 1.0f);
    imuScaleY     = prefs.getFloat("sclY", 1.0f);
    imuScaleZ     = prefs.getFloat("sclZ", 1.0f);
    imuCalibrated = true;
    Serial.println(F("IMU calibration loaded from NVS."));
  } else {
    Serial.println(F("No IMU calibration found — send C then select IMU calibration."));
  }
  prefs.end();
}

void imuCalPrint() {
  Serial.println(F("-- IMU calibration values --------------"));
  Serial.print(F("  Offset X: ")); Serial.println(imuOffsetX, 4);
  Serial.print(F("  Offset Y: ")); Serial.println(imuOffsetY, 4);
  Serial.print(F("  Offset Z: ")); Serial.println(imuOffsetZ, 4);
  Serial.print(F("  Scale  X: ")); Serial.println(imuScaleX,  4);
  Serial.print(F("  Scale  Y: ")); Serial.println(imuScaleY,  4);
  Serial.print(F("  Scale  Z: ")); Serial.println(imuScaleZ,  4);
  Serial.print(F("  Calibrated: ")); Serial.println(imuCalibrated ? F("yes") : F("no"));
  Serial.println(F("----------------------------------------"));
}

void imuCalClear() {
  prefs.begin("imu_cal", false);
  prefs.clear();
  prefs.end();
  imuOffsetX    = 0.0f; imuOffsetY = 0.0f; imuOffsetZ = 0.0f;
  imuScaleX     = 1.0f; imuScaleY  = 1.0f; imuScaleZ  = 1.0f;
  imuCalibrated = false;
  calPos1Done   = false;
  calPos2Done   = false;
  Serial.println(F("IMU calibration cleared."));
}

void imuCalStep1() {
  Serial.println(F("-- IMU calibration step 1 --------------"));
  Serial.println(F("  Place board FLAT on a level surface,"));
  Serial.println(F("  nose horizontal. Hold still."));
  Serial.println(F("  Sampling in 3 seconds..."));
  for (int i = 3; i > 0; i--) {
    esp_task_wdt_reset();
    Serial.println(i);
    delay(1000);
  }
  Serial.println(F("  Sampling..."));
  calPos1     = imuAverageSample(200);
  calPos1Done = true;
  Serial.println(F("  Position 1 captured:"));
  Serial.print(F("    X=")); Serial.print(calPos1.x, 4);
  Serial.print(F(" Y="));    Serial.print(calPos1.y, 4);
  Serial.print(F(" Z="));    Serial.println(calPos1.z, 4);
  Serial.println(F("  Now flip board UPSIDE DOWN and send CI2."));
  Serial.println(F("----------------------------------------"));
}

void imuCalStep2() {
  if (!calPos1Done) {
    Serial.println(F("Run CI1 first."));
    return;
  }
  Serial.println(F("-- IMU calibration step 2 --------------"));
  Serial.println(F("  Board should be UPSIDE DOWN, nose"));
  Serial.println(F("  horizontal. Hold still."));
  Serial.println(F("  Sampling in 3 seconds..."));
  for (int i = 3; i > 0; i--) {
    esp_task_wdt_reset();
    Serial.println(i);
    delay(1000);
  }
  Serial.println(F("  Sampling..."));
  calPos2     = imuAverageSample(200);
  calPos2Done = true;
  Serial.println(F("  Position 2 captured:"));
  Serial.print(F("    X=")); Serial.print(calPos2.x, 4);
  Serial.print(F(" Y="));    Serial.print(calPos2.y, 4);
  Serial.print(F(" Z="));    Serial.println(calPos2.z, 4);

  imuOffsetX = (calPos1.x + calPos2.x) / 2.0f;
  imuOffsetY = (calPos1.y + calPos2.y) / 2.0f;
  imuOffsetZ = (calPos1.z + calPos2.z) / 2.0f;
  imuScaleX  = (calPos1.x - calPos2.x) / 2.0f;
  imuScaleY  = (calPos1.y - calPos2.y) / 2.0f;
  imuScaleZ  = (calPos1.z - calPos2.z) / 2.0f;

  if (fabsf(imuScaleX) < 0.01f) imuScaleX = 1.0f;
  if (fabsf(imuScaleY) < 0.01f) imuScaleY = 1.0f;
  if (fabsf(imuScaleZ) < 0.01f) imuScaleZ = 1.0f;

  imuCalSaveToNvs();
  imuCalPrint();
  Serial.println(F("IMU calibration complete."));
}

void imuApplyCalibration(float rawX, float rawY, float rawZ,
                         float &calX, float &calY, float &calZ) {
  calX = (rawX - imuOffsetX) / imuScaleX;
  calY = (rawY - imuOffsetY) / imuScaleY;
  calZ = (rawZ - imuOffsetZ) / imuScaleZ;
}

// -------------------------------------------------------------------
//  Barometer calibration
// -------------------------------------------------------------------

void baroCalSaveToNvs() {
  prefs.begin("baro_cal", false);
  prefs.putFloat("knownAlt",    baroKnownAltM);
  prefs.putFloat("refPressure", baroRefPressure);
  prefs.putBool("done",         true);
  prefs.end();
  baroCalibrated = true;
  Serial.println(F("Barometer calibration saved to NVS."));
}

void baroCalLoadFromNvs() {
  prefs.begin("baro_cal", true);
  bool done = prefs.getBool("done", false);
  if (done) {
    baroKnownAltM   = prefs.getFloat("knownAlt",    0.0f);
    baroRefPressure = prefs.getFloat("refPressure",  101325.0f);
    baroCalibrated  = true;
    Serial.println(F("Barometer calibration loaded from NVS."));
    Serial.print(F("  Known alt  : ")); Serial.print(baroKnownAltM, 1);   Serial.println(F(" m MSL"));
    Serial.print(F("  Ref pressure: ")); Serial.print(baroRefPressure, 2); Serial.println(F(" Pa"));
  } else {
    Serial.println(F("No barometer calibration found — send C then select baro calibration."));
  }
  prefs.end();
}

void baroCalPrint() {
  Serial.println(F("-- Barometer calibration values --------"));
  Serial.print(F("  Known alt  : ")); Serial.print(baroKnownAltM, 1);    Serial.println(F(" m MSL"));
  Serial.print(F("  Ref pressure: ")); Serial.print(baroRefPressure, 2);  Serial.println(F(" Pa"));
  Serial.print(F("  Calibrated : ")); Serial.println(baroCalibrated ? F("yes") : F("no"));
  Serial.println(F("----------------------------------------"));
}

void baroCalClear() {
  prefs.begin("baro_cal", false);
  prefs.clear();
  prefs.end();
  baroKnownAltM   = 0.0f;
  baroRefPressure = 101325.0f;
  baroCalibrated  = false;
  Serial.println(F("Barometer calibration cleared."));
}

void baroCalRun() {
  Serial.println(F("-- Barometer calibration ---------------"));
  Serial.println(F("  Enter your current altitude above sea"));
  Serial.println(F("  level in metres, then press Enter."));
  Serial.println(F("  Example: 432 for 432m MSL"));
  Serial.print(F("> "));

  char inputBuf[16] = {0};
  uint8_t inputLen  = 0;
  unsigned long inputStart = millis();

  while (true) {
    esp_task_wdt_reset();
    if (millis() - inputStart > 60000) {
      Serial.println(F("\nTimed out."));
      return;
    }
    if (!Serial.available()) continue;
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputLen > 0) break;
    } else if (inputLen < 15) {
      inputBuf[inputLen++] = c;
      Serial.print(c);
    }
  }
  Serial.println();

  float enteredAlt = atof(inputBuf);
  if (enteredAlt <= 0 || enteredAlt > 9000) {
    Serial.println(F("Invalid altitude. Must be between 0 and 9000m."));
    return;
  }

  Serial.print(F("  Entered altitude: ")); Serial.print(enteredAlt, 1); Serial.println(F(" m MSL"));
  Serial.println(F("  Sampling barometer..."));

  double pressSum = 0;
  for (int i = 0; i < 200; i++) {
    esp_task_wdt_reset();
    baro.read();
    pressSum += baro.getPressure();
    delay(10);
  }
  float measuredPressure = (float)(pressSum / 200.0);

  float ratio = 1.0f - (enteredAlt / 44330.0f);
  float P0    = measuredPressure / powf(ratio, 5.2559f);

  baroKnownAltM   = enteredAlt;
  baroRefPressure = P0;

  Serial.print(F("  Measured pressure : ")); Serial.print(measuredPressure, 2); Serial.println(F(" Pa"));
  Serial.print(F("  Derived P0        : ")); Serial.print(baroRefPressure,  2); Serial.println(F(" Pa"));

  float verifyAlt = 44330.0f * (1.0f - powf(measuredPressure / baroRefPressure, 0.1902949f));
  Serial.print(F("  Verification alt  : ")); Serial.print(verifyAlt, 1); Serial.println(F(" m MSL"));

  baroCalSaveToNvs();
  Serial.println(F("Barometer calibration complete."));
  Serial.println(F("----------------------------------------"));
}