#ifndef MS5611_H
#define MS5611_H

#include <Arduino.h>
#include <Wire.h>

// Minimal MS5611 shim used only to let the sketch build inside the
// devcontainer. Implements the small subset of the API used by Meadowlark.

typedef enum { OSR_LOW, OSR_STANDARD, OSR_HIGH } osr_t;

#define MS5611_READ_OK 0

class MS5611 {
public:
  MS5611(uint8_t /*addr*/ = 0x77, TwoWire* /*wire*/ = &Wire) {}
  bool begin() { return true; }
  void setOversampling(osr_t /*osr*/) {}
  int read() { return MS5611_READ_OK; }
  float getPressure() { return 1013.25f; }
};

#endif // MS5611_H
