#pragma once
#include <cmath>
#include <cstdint>

// A nominal speed reference, not an observed velocity or a current limit.
inline float velocityReference(float goal, uint32_t elapsedUs, uint32_t rampUs) {
  if (!std::isfinite(goal)) return NAN;
  if (!rampUs || elapsedUs >= rampUs) return goal;
  return goal * (static_cast<float>(elapsedUs) / rampUs);
}
