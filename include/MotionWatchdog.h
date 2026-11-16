#pragma once
#include <cstdint>
#include <cmath>

// Require progress after startup, not just a displacement from the start point.
// This is a lack-of-motion check; it does not verify direction or steady speed.
class MotionWatchdog {
 public:
  void reset(float angle, uint32_t nowUs) {
    referenceAngle = angle;
    startUs = lastProgressUs = nowUs;
  }

  bool stalled(float angle, uint32_t nowUs) {
    if (!std::isfinite(angle)) return true;
    if (std::abs(angle - referenceAngle) >= 0.03f) {
      referenceAngle = angle;
      lastProgressUs = nowUs;
    }
    return nowUs - startUs >= 1200000 && nowUs - lastProgressUs >= 600000;
  }

 private:
  float referenceAngle = 0;
  uint32_t startUs = 0, lastProgressUs = 0;
};
