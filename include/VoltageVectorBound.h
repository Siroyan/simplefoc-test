#pragma once
#include <cmath>
#include <cstdint>

class VoltageVectorBound {
 public:
  bool invalid = false;
  float q = 0, d = 0, angle = 0;
  void apply(float requestQ, float requestD, float requestAngle, float limit) {
    if (invalid || !std::isfinite(requestQ) || !std::isfinite(requestD) ||
        !std::isfinite(requestAngle) || !std::isfinite(limit) || limit <= 0) {
      invalid = true; q = d = angle = 0; return;
    }
    const float length = std::hypot(requestQ, requestD);
    if (!std::isfinite(length)) { invalid = true; q = d = angle = 0; return; }
    const float scale = length > limit ? limit / length : 1;
    q = requestQ * scale; d = requestD * scale; angle = requestAngle;
  }
};

inline float nominalIqRamp(float goal, uint32_t elapsedUs) {
  if (!std::isfinite(goal) || goal < 0 || goal > .25f) return 0;
  return goal * (elapsedUs >= 300000 ? 1.0f : elapsedUs / 300000.0f);
}
