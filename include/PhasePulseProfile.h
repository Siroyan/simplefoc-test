#pragma once
#include <cstdint>

// Six balanced stator-voltage vectors U+/U-/V+/V-/W+/W-, each 150ms ON
// then 100ms zero-vector. Ramp magnitude at 3V/s, cap at 0.10V.
struct PhasePulseProfile {
  static constexpr uint32_t durationUs = 1500000;
  struct Vector { float a, b, c; };
  static Vector at(uint32_t elapsedUs) {
    if (elapsedUs >= durationUs || elapsedUs % 250000 >= 150000) return {0, 0, 0};
    const uint32_t pulse = elapsedUs / 250000;
    const float ramp = (elapsedUs % 250000) * 3e-6f;
    const float amplitude = (ramp < .10f ? ramp : .10f) * (pulse % 2 ? -1 : 1);
    Vector result{-amplitude / 2, -amplitude / 2, -amplitude / 2};
    if (pulse / 2 == 0) result.a = amplitude;
    else if (pulse / 2 == 1) result.b = amplitude;
    else result.c = amplitude;
    return result;
  }
};
