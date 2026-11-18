#pragma once
#include <cmath>
#include <cstdint>

// Unloaded motor only. The short sequence fits the complete 1ms RAM trace.
struct BenchCurrentPulse {
  static constexpr uint32_t durationUs=300000;
  static float target(uint32_t us,float amplitude) {
    if (!std::isfinite(amplitude) || amplitude<=0 || amplitude>.100001f) return NAN;
    if ((us>=40000 && us<60000) || (us>=180000 && us<200000)) return amplitude;
    if ((us>=110000 && us<130000) || (us>=250000 && us<270000)) return -amplitude;
    return 0;
  }
};
