#include "PhasePulseProfile.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
void require(bool condition, const char* message) {
  if (!condition) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
int main() {
  for (uint32_t us = 0; us <= 1600000; us += 37) {
    const auto v = PhasePulseProfile::at(us);
    require(std::abs(v.a + v.b + v.c) < 1e-6f, "balanced vector");
    require(std::abs(v.a) <= .100001f && std::abs(v.b) <= .100001f && std::abs(v.c) <= .100001f, "0.10V vector bound");
    require(std::abs(v.a - v.b) <= .150001f && std::abs(v.b - v.c) <= .150001f && std::abs(v.c - v.a) <= .150001f, "0.15V line voltage bound");
    if (us >= PhasePulseProfile::durationUs || us % 250000 >= 150000)
      require(v.a == 0 && v.b == 0 && v.c == 0, "zero vector in every rest and after deadline");
  }
  for (uint32_t phase = 0; phase < 3; ++phase) {
    const auto plus = PhasePulseProfile::at(phase * 500000 + 100000);
    const auto minus = PhasePulseProfile::at(phase * 500000 + 350000);
    require(std::abs(plus.a + minus.a) < 1e-6f && std::abs(plus.b + minus.b) < 1e-6f && std::abs(plus.c + minus.c) < 1e-6f, "opposite polarities");
    require((phase == 0 ? plus.a : phase == 1 ? plus.b : plus.c) > .09999f, "correct excited phase");
  }
  const auto first = PhasePulseProfile::at(10000);
  require(std::abs(first.a - .03f) < 1e-6f, "3V/s initial ramp");
  const auto end = PhasePulseProfile::at(UINT32_MAX);
  require(end.a == 0 && end.b == 0 && end.c == 0, "large elapsed time disables vector");
  std::cout << "PASS: six phases/signs, balance, voltage bounds, ramps, rests, finite deadline\n";
}
