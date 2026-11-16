#include "MagnitudeSlew.h"
#include <cstdlib>
#include <iostream>

void require(bool value, const char* message) {
  if (!value) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
bool near(float a, float b) { return std::abs(a - b) < 1e-5f; }

int main() {
  MagnitudeSlew limiter;
  limiter.reset(0);
  require(near(limiter.update(.5f, 10000), .03f), "gradual initial positive torque");
  require(near(limiter.update(.01f, 11000), .01f), "release without slew delay");
  require(near(limiter.update(-.5f, 12000), 0), "reverse first releases to zero");
  require(near(limiter.update(-.5f, 22000), -.03f), "negative torque builds gradually");
  require(near(limiter.update(-.02f, 23000), -.02f), "negative magnitude also releases immediately");
  require(near(limiter.update(.5f, 24000), 0), "positive reversal also passes zero");
  require(near(limiter.update(.5f, 34000), .03f), "positive rebuild is gradual");
  require(near(limiter.update(0, 34000), 0), "zero-time release");
  require(near(limiter.update(2, 34000), 0), "zero-time increase prohibited");

  limiter.reset(UINT32_MAX - 5000);
  require(near(limiter.update(.5f, 4999), .03f), "clock wrap");
  require(near(limiter.update(.5f, 600000), 0), "long gap releases output");
  limiter.reset(0);
  uint32_t previous = 0;
  for (uint32_t now = 100; now < 1000000; now += 1700) {
    const float before = limiter.value;
    const float after = limiter.update(2, now);
    require(std::abs(after) <= .5f, "hard 0.5V bound");
    require(after - before <= 3.0f * (now - previous) * 1e-6f + 1e-6f, "positive magnitude slew bound");
    previous = now;
  }
  require(near(limiter.value, .5f), "can reach voltage ceiling");
  require(limiter.update(NAN, 1000001) == 0 && limiter.invalid, "invalid request releases and latches fault");
  require(limiter.update(.5f, 1001001) == 0, "fault prevents re-energizing");
  MagnitudeSlew faster{6.0f};
  faster.reset(0);
  require(near(faster.update(.5f, 10000), .06f), "configured 6V/s build-up");
  require(near(faster.update(.01f, 11000), .01f), "configured rate preserves immediate release");
  require(near(faster.update(-.5f, 12000), 0), "configured rate preserves zero crossing");
  MagnitudeSlew forward;
  forward.reset(0);
  require(near(forward.update(.5f, 10000, true), .03f), "forward policy preserves rise");
  require(forward.update(-.5f, 11000, true) == 0, "negative request releases positive output");
  require(forward.update(-.5f, 30000, true) == 0, "negative request cannot build reverse output");
  previous = 30000;
  for (uint32_t now = 30100; now < 500000; now += 1700) {
    const float before = forward.value;
    const float after = forward.update(2, now, true);
    require(after >= 0 && after <= .5f, "forward policy keeps output in 0..0.5V");
    require(after - before <= 3.0f * (now - previous) * 1e-6f + 1e-6f, "forward rise remains bounded");
    previous = now;
  }
  require(near(forward.value, .5f), "forward policy can reach ceiling");
  require(forward.update(-INFINITY, 500001, true) == 0 && forward.invalid, "infinite negative request is a fault, not normal zero");
  forward.reset(0);
  require(forward.update(NAN, 100, true) == 0 && forward.invalid, "forward policy does not mask NaN");
  require(forward.update(.5f, 200, true) == 0, "forward fault stays latched");
  require(limiter.value == 0 && limiter.invalid, "signed policy fault remains independent");
  MagnitudeSlew configurable{30}; configurable.reset(0);
  configurable.riseRateVPerS = configurable.negativeRiseRateVPerS = 100;
  require(near(configurable.update(.5f,1000),.1f),"configured 100V/s rise");
  configurable.negativeRiseRateVPerS = NAN;
  require(configurable.update(.2f,2000)==0 && configurable.invalid,"invalid configured rate faults even on positive request");
  for (float rate : {0.0f,-3.0f,NAN,INFINITY}) {
    MagnitudeSlew invalidRate{rate}; invalidRate.reset(0);
    require(invalidRate.update(.5f,1000)==0 && invalidRate.invalid,"invalid rise rate cannot energize");
  }
  MagnitudeSlew invalidNegative{3,NAN}; invalidNegative.reset(0);
  require(invalidNegative.update(.5f,1000)==0 && invalidNegative.invalid,"invalid negative-rate constructor is not masked");
  std::cout << "PASS: rise, release, both reversal directions, zero time, wrap, long gap, bound, invalid input\n";
  std::cout << "PASS: forward-only negative rejection, original rise/ceiling, non-finite faults\n";
}
