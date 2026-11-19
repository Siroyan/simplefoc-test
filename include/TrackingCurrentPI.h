#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

// PI with feedback of the voltage actually admitted by the actuator's slew
// and vector limits. Call track() once after applying both limits.
class TrackingCurrentPI {
 public:
  float P = .20f, I = 50.0f;
  float trackingTimeS=0; // Zero preserves the original instantaneous tracking.
  float voltageLimit=.5f; // Explicit actuator/integrator ceiling; default behavior retained.
  bool invalid = false;
  uint32_t elapsedUs() const {return lastElapsedUs;}
  void reset(uint32_t now) {
    previousUs = now; lastElapsedUs=0;integral = error = requested = 0; invalid = false;
  }
  float request(float nextError, uint32_t now) {
    const uint32_t elapsed = now - previousUs;
    lastElapsedUs=elapsed;
    previousUs = now;
    if (invalid || !std::isfinite(nextError) || !std::isfinite(P) ||
        !std::isfinite(I) || !std::isfinite(trackingTimeS) || trackingTimeS<0 || !validLimit() || P < 0 || I < 0 || elapsed > 500000) {
      invalid = true; return requested = 0;
    }
    error = nextError;
    integral = std::max(-voltageLimit, std::min(voltageLimit, integral + I * error * elapsed * 1e-6f));
    requested = P * error + integral;
    if (!std::isfinite(requested)) { invalid = true; requested = 0; }
    return requested;
  }
  void track(float applied) {
    if (invalid || !std::isfinite(applied) || !std::isfinite(trackingTimeS) || trackingTimeS<0 || !validLimit() || std::abs(applied) > voltageLimit+.000001f) {
      invalid = true; integral = 0; return;
    }
    if (std::abs(applied - requested) > 1e-7f) {
      if (trackingTimeS==0) integral = std::max(-voltageLimit, std::min(voltageLimit, applied - P * error));
      else {
        const float gain=std::min(1.f,lastElapsedUs*1e-6f/trackingTimeS);
        integral=std::max(-voltageLimit,std::min(voltageLimit,integral+gain*(applied-requested)));
      }
    }
  }
 private:
  bool validLimit()const{return std::isfinite(voltageLimit) && voltageLimit>0 && voltageLimit<=1.f;}
  uint32_t previousUs = 0,lastElapsedUs=0;
  float integral = 0, error = 0, requested = 0;
};
