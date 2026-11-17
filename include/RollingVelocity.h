#pragma once
#include <cmath>
#include <cstdint>

// Experimental least-squares slope over 4ms, or a separate 2/8ms comparison.
// Decimation scales with the horizon so both retain it in the same buffer.
class RollingVelocity {
 public:
  float value = 0;
  bool longWindow = false;
  bool shortWindow = false;
  uint32_t horizonUs() const { return shortWindow ? 2000u : longWindow ? 8000u : 4000u; }
  uint32_t minimumSampleUs() const { return shortWindow ? 75u : longWindow ? 300u : 150u; }
  uint32_t warmupUs() const { return shortWindow ? 1500u : longWindow ? 6000u : 3000u; }
  void reset(float angle,uint32_t now) {
    next=1;size=1;initialized=true;samples[0]={now,angle};value=0;
  }
  float update(float angle,uint32_t now) {
    if (!std::isfinite(angle)) { initialized=false;return value=NAN; }
    if (!initialized || now-samples[(next+capacity-1)%capacity].us>500000) {
      reset(angle,now);return value;
    }
    if (now-samples[(next+capacity-1)%capacity].us<minimumSampleUs()) return value;
    samples[next]={now,angle};next=(next+1)%capacity;
    if (size<capacity) ++size;
    while (size>2 && now-samples[(next+capacity-size)%capacity].us>horizonUs()) --size;
    if (now-samples[(next+capacity-size)%capacity].us<warmupUs()) return value=0;
    float sumT=0,sumY=0,sumTT=0,sumTY=0;
    for (uint8_t n=0;n<size;++n) {
      const auto& sample=samples[(next+capacity-size+n)%capacity];
      const float t=-static_cast<float>(now-sample.us)*1e-6f;
      const float y=sample.angle-angle;
      sumT+=t;sumY+=y;sumTT+=t*t;sumTY+=t*y;
    }
    const float denominator=size*sumTT-sumT*sumT;
    value=denominator>1e-12f ? (size*sumTY-sumT*sumY)/denominator : 0;
    if (!std::isfinite(value)) initialized=false;
    return value;
  }
 private:
  struct Sample { uint32_t us;float angle; };
  static constexpr uint8_t capacity=32;
  Sample samples[capacity]{};
  uint8_t next=0,size=0;
  bool initialized=false;
};
