#pragma once
#include <cstdint>
#include <cmath>

// Signed displacement / elapsed wall time. An uneven pulse interval is never
// held as an instantaneous reciprocal speed for the duration of the next pulse.
class WindowVelocity {
 public:
  uint32_t periodUs = 10000; // Standard profile; current-velocity trial can select 2ms.
  float value = 0;

  void reset(float angle, uint32_t now) {
    previousAngle = angle;
    previousUs = now;
    value = 0;
    initialized = true;
  }

  float update(float angle, uint32_t now) {
    if (!std::isfinite(angle)) {
      initialized = false;
      return value = NAN;
    }
    if (!initialized || now - previousUs > 500000) {
      reset(angle, now);
    } else if (now - previousUs >= periodUs) {
      value = (angle - previousAngle) * 1e6f / (now - previousUs);
      previousAngle = angle;
      previousUs = now;
    }
    return value;
  }

 private:
  float previousAngle = 0;
  uint32_t previousUs = 0;
  bool initialized = false;
};
