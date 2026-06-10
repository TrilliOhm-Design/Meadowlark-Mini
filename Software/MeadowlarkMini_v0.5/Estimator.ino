/* ----------------------------------------------------------------------------
 *  Estimator
 *
 *  1-D vertical Kalman filter. State:
 *       x = [ altitude (m AGL), velocity (m/s, up+), accelBias (m/s^2) ]
 *
 *  - PREDICT runs every tick from the IMU's vertical acceleration. This is the
 *    fast, low-latency path that drives event timing - it never waits on the
 *    barometer.
 *  - UPDATE (barometer) is applied ONLY when the rocket is slow enough for the
 *    baro to be trustworthy: past burnout AND |velocity| < cfg.baroGateVel.
 *    During boost and high-speed ascent the barometer is ignored entirely (the
 *    airframe is slow to pressurise and the MS5611 lags the IMU, and pressure
 *    is Mach-corrupted), so the estimate is pure-inertial on the way up. The
 *    baro re-engages near apogee and through descent, where it is accurate, to
 *    pin down altitude and confirm the velocity sign for apogee/main events.
 *
 *  Tuning lives in the KF_* constants below; refine them from a 'download' CSV.
 * --------------------------------------------------------------------------*/

// --- Kalman tuning -----------------------------------------------------------
const float KF_SIGMA_ACCEL = 0.5f;    // m/s^2  : accel/process noise (model trust)
const float KF_SIGMA_BIAS  = 0.02f;   // m/s^2  : accel-bias random walk
const float KF_R_BARO      = 2.0f;    // m^2    : baro altitude variance (~1.4 m)
const float KF_P0_ALT      = 1.0f;    // initial covariance
const float KF_P0_VEL      = 1.0f;
const float KF_P0_BIAS     = 1.0f;

// --- Filter state ------------------------------------------------------------
static float kfX[3];          // [alt, vel, accBias]
static float kfP[3][3];       // covariance

void estimatorReset() {
  kfX[0] = 0.0f; kfX[1] = 0.0f; kfX[2] = 0.0f;
  memset(kfP, 0, sizeof(kfP));
  kfP[0][0] = KF_P0_ALT;
  kfP[1][1] = KF_P0_VEL;
  kfP[2][2] = KF_P0_BIAS;
}

// PREDICT: propagate state and covariance with vertical accel `u` over `dt`.
//   alt += vel*dt + 0.5*(u-bias)*dt^2 ;  vel += (u-bias)*dt ;  bias unchanged
//   F = [[1, dt, -0.5dt^2], [0, 1, -dt], [0, 0, 1]]
static void kfPredict(float u, float dt) {
  float dt2 = dt * dt;
  float a   = u - kfX[2];
  kfX[0] += kfX[1] * dt + 0.5f * a * dt2;
  kfX[1] += a * dt;
  // kfX[2] (bias) is modelled as constant

  const float f01 = dt, f02 = -0.5f * dt2, f12 = -dt;

  // FP = F * P
  float FP[3][3];
  for (int j = 0; j < 3; j++) FP[0][j] = kfP[0][j] + f01 * kfP[1][j] + f02 * kfP[2][j];
  for (int j = 0; j < 3; j++) FP[1][j] = kfP[1][j] + f12 * kfP[2][j];
  for (int j = 0; j < 3; j++) FP[2][j] = kfP[2][j];

  // P = FP * F^T   ( (FP*F^T)[i][k] = sum_j FP[i][j]*F[k][j] )
  float Pn[3][3];
  for (int i = 0; i < 3; i++) {
    Pn[i][0] = FP[i][0] + FP[i][1] * f01 + FP[i][2] * f02;
    Pn[i][1] = FP[i][1] + FP[i][2] * f12;
    Pn[i][2] = FP[i][2];
  }

  // + Q : accel process noise mapped through G=[0.5dt^2, dt, 0], plus bias walk
  float sa2 = KF_SIGMA_ACCEL * KF_SIGMA_ACCEL;
  float g0 = 0.5f * dt2, g1 = dt;
  Pn[0][0] += sa2 * g0 * g0;
  Pn[0][1] += sa2 * g0 * g1;
  Pn[1][0] += sa2 * g1 * g0;
  Pn[1][1] += sa2 * g1 * g1;
  Pn[2][2] += KF_SIGMA_BIAS * KF_SIGMA_BIAS * dt;

  memcpy(kfP, Pn, sizeof(kfP));
}

// UPDATE: scalar barometric-altitude measurement z (H = [1,0,0]).
static void kfUpdate(float z, float R) {
  float S  = kfP[0][0] + R;
  float K0 = kfP[0][0] / S;
  float K1 = kfP[1][0] / S;
  float K2 = kfP[2][0] / S;

  float y = z - kfX[0];
  kfX[0] += K0 * y;
  kfX[1] += K1 * y;
  kfX[2] += K2 * y;

  // P = (I - K H) P  ->  row i: P[i][j] -= K_i * P[0][j]
  float r0[3] = { kfP[0][0], kfP[0][1], kfP[0][2] };
  for (int j = 0; j < 3; j++) {
    kfP[0][j] -= K0 * r0[j];
    kfP[1][j] -= K1 * r0[j];
    kfP[2][j] -= K2 * r0[j];
  }
}

// ----------------------------------------------------------------------------
//  Per-tick estimation: read IMU, run the KF, conditionally fold in the baro.
// ----------------------------------------------------------------------------
void updateEstimator() {
  // --- Real elapsed time (the loop rate varies when the baro is/ isn't read).
  static uint32_t lastUs = 0;
  uint32_t nowUs = micros();
  float dt = (lastUs == 0) ? DT : (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dt <= 0.0f || dt > 0.2f) dt = DT;        // clamp scheduling glitches

  // --- IMU (lightly low-passed to tame vibration without adding much lag)
  sensors_event_t a, g, t;
  imu.getEvent(&a, &g, &t);
  float ax = lpAx.update(a.acceleration.x);
  float ay = lpAy.update(a.acceleration.y);
  float az = lpAz.update(a.acceleration.z);
  lpGx.update(g.gyro.x - gyroBias[0]);
  lpGy.update(g.gyro.y - gyroBias[1]);
  lpGz.update(g.gyro.z - gyroBias[2]);

  // Orientation-independent quantities (work in the rocket's "up" axis):
  float accMag  = sqrtf(ax * ax + ay * ay + az * az);   // total specific force
  float axial   = ax * upHat[0] + ay * upHat[1] + az * upHat[2];
  float netVert = axial - G0;                           // vertical accel (up +)

  // --- KF predict from the IMU (always; this is the fast path)
  kfPredict(netVert, dt);

  // --- KF update from the barometer ONLY when it is trustworthy:
  //     on the pad, or past burnout and travelling slowly. During boost and
  //     fast ascent/descent the baro is skipped entirely (not even read), which
  //     also keeps the loop fast exactly when reaction time matters most.
  bool baroTrusted =
      (state == ST_STANDBY) ||
      (state >= ST_COAST && fabsf(kfX[1]) < cfg.baroGateVel);

  if (baroTrusted && baro.read() == MS5611_READ_OK) {
    float zAlt = pressureToAltitude(baro.getPressure(), groundPressure);
    kfUpdate(zAlt, KF_R_BARO);
  }

  // --- Publish estimator outputs for the state machine / logger
  altAGL = kfX[0];
  velEst = kfX[1];
  if (altAGL > maxAltAGL) maxAltAGL = altAGL;
  gAccMag  = accMag;
  gNetVert = netVert;
}