#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>

// Observations only: never links to motor current control or enables the driver.
// Scaling is nominal (3.3V ADC reference), not a calibrated phase-current limit.
class BoardCurrentObserver {
 public:
  struct Frame {
    uint32_t us, adc1Frames, adc2Frames;
    uint16_t a, b, c, vbus, temperature;
  };
  bool ready = false;
  float offset[3] = {};
  bool beginDisabled(BLDCDriver6PWM& driver);
  Frame read() const;
  static float busVoltage(uint16_t raw) { return raw * (3.3f / 4096) * (187.0f / 18); }
  float current(uint16_t raw, uint8_t phase) const {
    return (raw - offset[phase]) * (3.3f / 4096) / (0.003f * (-64.0f / 7));
  }
  static bool gatesOff();
  static void configureGatePullDowns();
 private:
  void* params = nullptr;
};
