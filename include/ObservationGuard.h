#pragma once
#include <cmath>
#include <cstdint>

// ADC integrity and sustained VBUS deviation only. Not a phase-current limit
// and not a measurement of the power supply's CV/CC operating flag.
class ObservationGuard {
 public:
  enum Fault { none, baseline, stale, incoherent, rail, nonfinite, bus_deviation };
  Fault fault = none;
  float busLow = 0, busHigh = 0;
  void reset(float bus) {
    fault = std::isfinite(bus) && bus >= 10.3f && bus <= 11.9f ? none : baseline;
    busLow = bus * .93f; busHigh = bus * 1.07f;
    outside = false;
  }
  Fault check(uint32_t now, uint32_t age, int32_t frameDifference,
              uint16_t a, uint16_t b, uint16_t c, float bus) {
    if (fault != none) return fault;
    if (age > 2000) return fault = stale;
    if (frameDifference < -1 || frameDifference > 1) return fault = incoherent;
    if (a < 10 || a > 4085 || b < 10 || b > 4085 || c < 10 || c > 4085) return fault = rail;
    if (!std::isfinite(bus)) return fault = nonfinite;
    if (bus < busLow || bus > busHigh) {
      if (!outside) { outside = true; since = now; }
      if (now - since >= 20000) return fault = bus_deviation;
    } else outside = false;
    return none;
  }
  static const char* reason(Fault value) {
    switch (value) {
      case baseline: return "ADC baseline VBUS outside expected range";
      case stale: return "ADC observation stale";
      case incoherent: return "ADC frame mismatch";
      case rail: return "current ADC near rail";
      case nonfinite: return "non-finite ADC VBUS";
      case bus_deviation: return "VBUS deviated over 7 percent for 20ms";
      default: return "no observation fault";
    }
  }
 private:
  bool outside = false;
  uint32_t since = 0;
};
