#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

// Reduce voltage-command magnitude immediately, but increase it gradually in
// either direction. A sign reversal passes through zero instead of a step.
class MagnitudeSlew {
 public:
  explicit MagnitudeSlew(float rateVPerS = 3.0f, float negativeRateVPerS = 0)
      : riseRateVPerS(rateVPerS), negativeRiseRateVPerS(negativeRateVPerS == 0 ? rateVPerS : negativeRateVPerS) {}
  float riseRateVPerS;
  float negativeRiseRateVPerS;
  float voltageLimit=.5f;
  float value = 0;
  bool invalid = false;
  void reset(uint32_t now) { previousUs = now; value = 0; invalid = false; }

  float update(float request, uint32_t now, bool forwardOnly = false) {
    const uint32_t elapsed = now - previousUs;
    previousUs = now;
    if (invalid || !std::isfinite(request) || !std::isfinite(riseRateVPerS) ||
        !std::isfinite(negativeRiseRateVPerS) || !std::isfinite(voltageLimit) || voltageLimit<=0 || voltageLimit>1.f || riseRateVPerS <= 0 || negativeRiseRateVPerS <= 0) {
      invalid = true;
      return value = 0;
    }
    if (elapsed > 500000) return value = 0;
    // Check finiteness before applying this policy so NaN cannot be masked.
    if (forwardOnly && request < 0) request = 0;
    request = std::max(-voltageLimit, std::min(voltageLimit, request));
    if (value * request < 0) return value = 0;
    if (std::abs(request) <= std::abs(value)) return value = request;
    const float rate = request < 0 ? negativeRiseRateVPerS : riseRateVPerS;
    const float magnitude = std::min(std::abs(request), std::abs(value) + rate * elapsed * 1e-6f);
    return value = std::copysign(magnitude, request);
  }

 private:
  uint32_t previousUs = 0;
};
