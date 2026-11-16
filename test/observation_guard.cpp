#include "ObservationGuard.h"
#include <cstdlib>
#include <iostream>
void require(bool value, const char* message) {
  if (!value) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
int main() {
  ObservationGuard guard;
  auto sample = [&](uint32_t now, float bus) { return guard.check(now, 80, 0, 2530, 2535, 2541, bus); };
  guard.reset(11.18f);
  for (uint32_t now = 0; now < 3000000; now += 81)
    require(sample(now, 11.19f) == ObservationGuard::none, "nominal long run");
  require(sample(3000000, 9) == ObservationGuard::none, "brief dip starts debounce");
  require(sample(3019999, 9) == ObservationGuard::none, "dip under 20ms is tolerated");
  require(sample(3020000, 11.18f) == ObservationGuard::none, "recovery resets dip");
  require(sample(3030000, 9) == ObservationGuard::none, "new dip gets its own debounce");
  require(sample(3050000, 9) == ObservationGuard::bus_deviation, "sustained dip stops");
  require(sample(3060000, 11.18f) == ObservationGuard::bus_deviation, "fault stays latched");
  guard.reset(11.18f);
  require(sample(UINT32_MAX - 5000, 13) == ObservationGuard::none, "overvoltage starts before wrap");
  require(sample(14999, 13) == ObservationGuard::bus_deviation, "sustained high across clock wrap");
  guard.reset(11.18f);
  require(guard.check(0, 2000, 1, 10, 4085, 2000, 11.18f) == ObservationGuard::none, "valid integrity boundaries");
  require(guard.check(1, 2001, 0, 2500, 2500, 2500, 11.18f) == ObservationGuard::stale, "stale ADC stops");
  guard.reset(11.18f);
  require(guard.check(0, 80, 2, 2500, 2500, 2500, 11.18f) == ObservationGuard::incoherent, "mismatched frames stop");
  guard.reset(11.18f);
  require(guard.check(0, 80, 0, 2500, 4095, 2500, 11.18f) == ObservationGuard::rail, "ADC saturation stops");
  guard.reset(11.18f);
  require(sample(0, NAN) == ObservationGuard::nonfinite, "non-finite bus stops");
  guard.reset(8);
  require(sample(0, 11.18f) == ObservationGuard::baseline, "invalid baseline cannot energize");
  std::cout << "PASS: nominal, transient recovery, sustained low/high, wrap, latch, freshness, coherence, rails, non-finite, baseline\n";
}
