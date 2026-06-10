/* ----------------------------------------------------------------------------
 *  FlightStateMachine
 *
 *  Owns the flight state transitions and the recovery-event triggering, plus
 *  the per-state entry actions (chimes, LED, and starting/ending the data log).
 *
 *  Flow:  STANDBY -> BOOST -> COAST -> DROGUE -> MAIN -> LANDED
 * --------------------------------------------------------------------------*/

// ----------------------------------------------------------------------------
//  State entry actions (chimes / LED / data-log hooks)
// ----------------------------------------------------------------------------
void enterState(FlightState s) {
  state = s;
  switch (s) {
    case ST_CONFIG:
      Serial.println(F("[STATE] CONFIG"));
      digitalWrite(PIN_USER_LED, HIGH);
      printConfigMenu();
      printSettings();
      break;
    case ST_STANDBY:
      Serial.println(F("[STATE] STANDBY - armed and ready for flight"));
      digitalWrite(PIN_USER_LED, HIGH);
      dataLogReset();                                            // arm the logger
      playTune(TUNE_READY, sizeof(TUNE_READY) / sizeof(Note));   // ready chime
      break;
    case ST_BOOST:
      Serial.println(F("[STATE] BOOST"));
      dataLogBegin();                                            // start recording
      playTune(TUNE_LAUNCH, sizeof(TUNE_LAUNCH) / sizeof(Note));
      break;
    case ST_COAST:   Serial.println(F("[STATE] COAST")); break;
    case ST_DROGUE:
      Serial.println(F("[STATE] APOGEE -> DROGUE"));
      playTune(TUNE_APOGEE, sizeof(TUNE_APOGEE) / sizeof(Note));
      break;
    case ST_MAIN:    Serial.println(F("[STATE] MAIN")); break;
    case ST_LANDED:
      Serial.println(F("[STATE] LANDED"));
      dataLogEnd();                          // freeze RAM log, save flash summary
      break;
    default: break;
  }
}

// ----------------------------------------------------------------------------
//  Per-tick flight logic
// ----------------------------------------------------------------------------
void runStateMachine() {
  uint32_t now = millis();
  switch (state) {

    case ST_STANDBY:
      // Detect liftoff by total specific force (orientation independent).
      if (gAccMag > cfg.launchAccelG * G0) {
        if (++launchCount >= LAUNCH_CONFIRM) {
          tBoost = now;
          enterState(ST_BOOST);
        }
      } else launchCount = 0;
      break;

    case ST_BOOST:
      // Burnout when thrust falls away (|a| drops toward free-fall/drag).
      if (now - tBoost > MIN_BOOST_MS && gAccMag < cfg.burnoutAccelG * G0) {
        if (++burnoutCount >= BURNOUT_CONFIRM) enterState(ST_COAST);
      } else burnoutCount = 0;
      break;

    case ST_COAST:
      // Apogee: barometric velocity has gone negative AND we have dropped a
      // small margin below the peak. Baro is primary here; it is immune to the
      // accelerometer's free-fall ambiguity.
      if (velEst < APOGEE_VEL_MS && altAGL < (maxAltAGL - cfg.apogeeAltMargin)) { // add or aggressive check on each sensor to prevent single failure mode 
        if (++apogeeCount >= APOGEE_CONFIRM) {
          tApogee = now;
          apogeeBackupArmed = true;
          if (cfg.enableDrogue) firePyro(0);   // Pyro 1 = drogue
          enterState(ST_DROGUE);
        }
      } else apogeeCount = 0;
      break;

    case ST_DROGUE:
      // Redundant apogee event on Pyro 3 a moment later.
      if (apogeeBackupArmed && now - tApogee >= cfg.apogeeBackupMs) {
        if (cfg.enableBackup) firePyro(2);     // Pyro 3 = apogee backup
        apogeeBackupArmed = false;
      }
      // Main chute at the configured altitude on the way down.
      if (altAGL <= cfg.mainDeployAltAGL) {
        if (cfg.enableMain) firePyro(1);       // Pyro 2 = main
        enterState(ST_MAIN);
      }
      break;

    case ST_MAIN:
      // Touchdown when vertical velocity stays near zero for a while.
      if (fabsf(velEst) < LANDED_VEL_MS) {
        if (tLandedStart == 0) tLandedStart = now;
        else if (now - tLandedStart >= LANDED_HOLD_MS) enterState(ST_LANDED);
      } else tLandedStart = 0;
      break;

    case ST_LANDED:
      // Periodically beep out the recorded apogee (metres AGL).
      if (!tunePlaying) {
        static uint32_t lastBeepOut = 0;
        if (now - lastBeepOut > 4000) {
          lastBeepOut = now;
          buildAltitudeBeepOut((uint32_t)(maxAltAGL + 0.5f));
        }
      }
      break;

    default: break;
  }
}