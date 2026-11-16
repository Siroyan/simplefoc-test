#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

// Experimental 84th-order current correction, anchored only after two matching
// Index observations. This is not an absolute current or torque calibration.
class PositionFeedforward {
 public:
  const float cosA, sinA, boundA;
  uint8_t seenIndices = 0;
  bool invalid = false, armed = false;
  uint32_t activeCalls = 0;
  float correction = 0, maxRequested = 0;
  PositionFeedforward(float cosine, float sine, float bound)
      : cosA(cosine), sinA(sine), boundA(bound) { reset(); }
  void reset() {
    seenIndices = 0; armed = false; activeCalls = 0;
    correction = maxRequested = anchor = 0; armedAtUs = 0;
    invalid = !std::isfinite(cosA) || !std::isfinite(sinA) || !std::isfinite(boundA) || boundA <= 0 || boundA > .03f;
  }
  void observeIndex(float angle, uint32_t elapsedUs) {
    ++seenIndices;
    if (invalid || !std::isfinite(angle)) { invalid = true; correction = 0; return; }
    if (seenIndices == 1) anchor = angle;
    else if (std::abs(std::remainder(angle-anchor, 6.28318530718f)) > 2*6.28318530718f/4000 + 1e-5f) {
      invalid = true; correction = 0; return;
    }
    if (seenIndices == 2) { armed = true; armedAtUs = elapsedUs; }
  }
  float update(float angle, int direction, uint32_t elapsedUs) {
    if (invalid || !std::isfinite(angle) || (direction != 1 && direction != -1)) {
      invalid = true; return correction = 0;
    }
    const uint32_t gateStart = std::max<uint32_t>(800000, armedAtUs);
    if (!armed || elapsedUs <= gateStart) return correction = 0;
    const float blend = std::min(1.0f, (elapsedUs-gateStart)/200000.0f);
    const float phase = 84.0f * direction * (angle-anchor);
    const float wave = cosA*std::cos(phase) + sinA*std::sin(phase);
    if (!std::isfinite(wave)) { invalid = true; return correction = 0; }
    correction = blend*std::max(-boundA, std::min(boundA, wave));
    maxRequested = std::max(maxRequested, std::abs(correction)); ++activeCalls;
    return correction;
  }
  float mix(float pid, float limit) {
    if (invalid || !std::isfinite(pid) || !std::isfinite(limit) || limit <= 0 || !std::isfinite(pid+correction)) {
      invalid = true; correction = 0; return 0;
    }
    return std::max(-limit, std::min(limit, pid+correction));
  }
 private:
  float anchor = 0;
  uint32_t armedAtUs = 0;
};
