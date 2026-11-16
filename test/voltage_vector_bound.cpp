#include "VoltageVectorBound.h"
#include <cfloat>
#include <cstdlib>
#include <iostream>
void require(bool condition, const char* message) {
  if (!condition) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
int main() {
  VoltageVectorBound bound;
  for (int a = -20; a <= 20; ++a) for (int b = -20; b <= 20; ++b) {
    const float q = a / 20.0f, d = b / 20.0f;
    bound.apply(q, d, 2.0f, .5f);
    require(!bound.invalid && std::hypot(bound.q, bound.d) <= .500001f, "joint vector bound");
    require(std::abs(bound.q * d - bound.d * q) < 1e-6f, "direction preserved");
    if (std::hypot(q, d) <= .5f)
      require(bound.q == q && bound.d == d, "valid request unchanged");
  }
  bound.apply(0, 0, 1, .5f);
  require(bound.q == 0 && bound.d == 0 && bound.angle == 1, "zero vector avoids division by zero");
  bound.apply(.2f, .1f, NAN, .5f);
  require(bound.invalid && bound.q == 0 && bound.d == 0 && bound.angle == 0, "invalid angle is zeroed too");
  bound.apply(.2f, 0, 0, .5f);
  require(bound.invalid && bound.q == 0 && bound.d == 0, "fault latches");
  for (float invalid : {NAN, INFINITY, -INFINITY}) {
    VoltageVectorBound test;
    test.apply(invalid, .1f, 0, .5f);
    require(test.invalid && test.q == 0 && test.d == 0, "invalid q rejected");
    VoltageVectorBound other;
    other.apply(.1f, invalid, 0, .5f);
    require(other.invalid && other.q == 0 && other.d == 0, "invalid d rejected");
  }
  for (float invalid : {0.0f, -.5f, NAN, INFINITY}) {
    VoltageVectorBound test;
    test.apply(.1f, .1f, 0, invalid);
    require(test.invalid && test.q == 0 && test.d == 0, "invalid limit rejected");
  }
  VoltageVectorBound huge;
  huge.apply(FLT_MAX, FLT_MAX, 0, .5f);
  require(huge.invalid && huge.q == 0 && huge.d == 0, "overflow cannot energize");
  for (float goal : {.15f, .25f}) {
    float previous = 0;
    for (uint32_t us = 0; us < 400000; us += 97) {
      const float target = nominalIqRamp(goal, us);
      require(target >= previous && target <= goal + 1e-6f, "bounded monotonic Iq ramp");
      previous = target;
    }
  }
  require(nominalIqRamp(.15f, 0) == 0 && nominalIqRamp(.15f, 300000) == .15f, "300ms ramp endpoints");
  require(nominalIqRamp(NAN, 1) == 0 && nominalIqRamp(.5f, 1) == 0, "invalid current profile stays zero");
  std::cout << "PASS: vector norm/direction, zero, invalid inputs/angle/limit, fault latch, overflow, Iq ramp\n";
}
