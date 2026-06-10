/* ----------------------------------------------------------------------------
 *  Calibration
 *
 *  Learns the gyro bias, the gravity ("up") vector and the ground-level
 *  pressure while the vehicle is stationary and vertical on the pad. The
 *  gravity vector is what lets the board be mounted in ANY orientation:
 *  flight acceleration is later projected onto it to get the true axial value.
 * --------------------------------------------------------------------------*/

bool calibrate() {
  Serial.println(F("[CAL] Hold still on the pad..."));
  double sa[3] = {0, 0, 0};
  double sg[3] = {0, 0, 0};
  double sp    = 0;
  float  aMin[3] = { 1e9,  1e9,  1e9};
  float  aMax[3] = {-1e9, -1e9, -1e9};
  uint16_t pCount = 0;

  for (uint16_t i = 0; i < CAL_SAMPLES; i++) {
    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    float av[3] = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
    float gv[3] = {g.gyro.x,         g.gyro.y,         g.gyro.z};
    for (int k = 0; k < 3; k++) {
      sa[k] += av[k]; sg[k] += gv[k];
      aMin[k] = min(aMin[k], av[k]);
      aMax[k] = max(aMax[k], av[k]);
    }
    if (baro.read() == MS5611_READ_OK) { sp += baro.getPressure(); pCount++; }

    // Blink LED while calibrating and keep the watchdog happy.
    digitalWrite(PIN_USER_LED, (i >> 4) & 1);
    feedWatchdog();
    delay(5);
  }

  // Reject calibration if the board was moving.
  for (int k = 0; k < 3; k++) {
    if ((aMax[k] - aMin[k]) > CAL_MOTION_TOL) {
      Serial.println(F("[CAL] FAILED - motion detected. Keep the rocket still."));
      return false;
    }
  }

  float gx = sa[0] / CAL_SAMPLES;
  float gy = sa[1] / CAL_SAMPLES;
  float gz = sa[2] / CAL_SAMPLES;
  float mag = sqrtf(gx * gx + gy * gy + gz * gz);
  if (mag < 0.5f * G0 || mag > 1.5f * G0) {
    Serial.println(F("[CAL] FAILED - gravity magnitude out of range."));
    return false;
  }

  // upHat = rocket's vertical axis expressed in the board frame. Because the
  // rocket stands vertical on the pad, the at-rest accelerometer vector points
  // along "up". Projecting flight acceleration onto upHat makes every later
  // calculation independent of how the board is mounted.
  upHat[0] = gx / mag;  upHat[1] = gy / mag;  upHat[2] = gz / mag;

  gyroBias[0] = sg[0] / CAL_SAMPLES;
  gyroBias[1] = sg[1] / CAL_SAMPLES;
  gyroBias[2] = sg[2] / CAL_SAMPLES;

  groundPressure = (pCount > 0) ? (sp / pCount) : 1013.25f;

  Serial.print(F("[CAL] up = ["));
  Serial.print(upHat[0], 3); Serial.print(F(", "));
  Serial.print(upHat[1], 3); Serial.print(F(", "));
  Serial.print(upHat[2], 3); Serial.print(F("]  |g|="));
  Serial.print(mag, 2);
  Serial.print(F(" m/s^2  P0=")); Serial.print(groundPressure, 2);
  Serial.println(F(" hPa"));
  return true;
}