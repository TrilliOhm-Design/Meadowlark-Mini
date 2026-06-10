#ifndef ADAFRUIT_LSM6DSO32_H
#define ADAFRUIT_LSM6DSO32_H

#include <Arduino.h>
#include <Wire.h>

// Minimal compatibility shim for the real Adafruit_LSM6DSO32 library.
// Provides only the types and methods used by the Meadowlark sketch so the
// project can build inside the devcontainer without fetching upstream libs.

// Ranges / rates used by the sketch (values are irrelevant for the stub)
#define LSM6DSO32_ACCEL_RANGE_32_G  32
#define LSM6DS_GYRO_RANGE_2000_DPS  2000
#define LSM6DS_RATE_208_HZ          208

// sensors_event_t lightweight replacement used by the sketch.
typedef struct {
  struct { float x, y, z; } acceleration;
  struct { float x, y, z; } gyro;
  float temperature;
} sensors_event_t;

class Adafruit_LSM6DSO32 {
public:
  Adafruit_LSM6DSO32() {}
  bool begin_I2C(uint8_t /*addr*/, TwoWire* /*wire*/) { return true; }
  void setAccelRange(int /*r*/) {}
  void setGyroRange(int /*r*/) {}
  void setAccelDataRate(int /*r*/) {}
  void setGyroDataRate(int /*r*/) {}

  // Fill events with zero / nominal values so the sketch can read sensible
  // numbers during builds/tests that don't access real hardware.
  void getEvent(sensors_event_t* a, sensors_event_t* g, sensors_event_t* t) {
    if (a) { a->acceleration.x = 0.0f; a->acceleration.y = 0.0f; a->acceleration.z = 9.80665f; }
    if (g) { g->gyro.x = 0.0f; g->gyro.y = 0.0f; g->gyro.z = 0.0f; }
    if (t) { t->temperature = 25.0f; }
  }
};

#endif // ADAFRUIT_LSM6DSO32_H
