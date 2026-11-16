#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

// Small identification signal only; it never raises the existing total limit.
class CurrentProbe {
 public:
  const float frequencyHz, amplitudeA, phaseRad;
  const uint32_t gateUs, blendUs;
  bool invalid=false;
  uint32_t calls=0, clipped=0;
  float value=0, peak=0;
  CurrentProbe(float frequency,float amplitude,uint32_t gate=800000,uint32_t blend=200000,float phase=0)
      :frequencyHz(frequency),amplitudeA(amplitude),phaseRad(phase),gateUs(gate),blendUs(blend) { reset(); }
  void reset() {
    calls=clipped=0;value=peak=0;
    invalid=!std::isfinite(frequencyHz)||frequencyHz<=0||frequencyHz>20||
            !std::isfinite(amplitudeA)||amplitudeA<0||amplitudeA>.02f||
            !std::isfinite(phaseRad)||std::abs(phaseRad)>3.1415928f||
            gateUs<200000||blendUs<100000||uint64_t(gateUs)+blendUs>2000000;
  }
  float mix(float pid,uint32_t elapsedUs,float limit) {
    if (invalid||!std::isfinite(pid)||!std::isfinite(limit)||limit<=0||limit>.45f) {
      invalid=true;value=0;return 0;
    }
    const float blend=elapsedUs<=gateUs ? 0 : std::min(1.0f,(elapsedUs-gateUs)/static_cast<float>(blendUs));
    const float phase=6.28318530718f*frequencyHz*(elapsedUs*1e-6f)+phaseRad;
    value=amplitudeA*blend*std::sin(phase);
    const float requested=pid+value;
    if (!std::isfinite(requested)) { invalid=true;value=0;return 0; }
    const float total=std::max(-limit,std::min(limit,requested));
    ++calls;if (std::abs(total-requested)>1e-7f) ++clipped;
    peak=std::max(peak,std::abs(value));
    return total;
  }
};
