#pragma once
#include <cmath>
#include <cstdint>

// Experimental four-edge period estimate. edge() is integer-only for the ISR.
// The caller must exclude interrupts during reset() and snapshot(); evaluate()
// runs outside that critical section. Independent speed protection is separate.
class EdgeVelocity {
 public:
  static constexpr float phaseGain=200.f;
  static constexpr uint32_t sampleGapLimitUs=5000;
  enum Fault : uint8_t {none=0,invalidWindowOrClock=1,controlSampleGap=2,nonfinitePhase=3};
  Fault fault=none;
  uint32_t diagnosticNow=0,diagnosticEdge=0,diagnosticAge=0,diagnosticGap=0;
  struct Snapshot {
    uint32_t times[5]{};
    uint32_t count=0,anchorCount=0,anchorUs=0;
    uint8_t size=0;
    int8_t direction=0;
    bool invalid=false;
  };
  void reset(uint32_t count) {
    previousCount=count;size=0;direction=0;invalid=false;
    originCount=count;anchorCount=count;anchorUs=0;
    estimateAngle=angleRemainder=value=0;estimateUs=0;estimateValid=false;
    fault=none;diagnosticNow=diagnosticEdge=diagnosticAge=diagnosticGap=0;
  }
  void edge(uint32_t count,uint32_t now) {
    if (invalid) return;
    const uint32_t delta=count-previousCount;
    previousCount=count;
    if (delta!=1u && delta!=UINT32_MAX) {invalid=true;return;}
    if (size && now-times[size-1]>0x7fffffffu) {invalid=true;return;}
    const int8_t nextDirection=delta==1u ? 1 : -1;
    const bool restart=direction!=nextDirection || (size && now-times[size-1]>500000u);
    if (restart) size=0;
    direction=nextDirection;
    if (size && now-times[size-1]==0) {invalid=true;return;}
    if (size==5) {
      for (uint8_t i=0;i<4;++i) times[i]=times[i+1];
      size=4;
    }
    times[size]=now;++size;
    if (restart || (count&3u)==0) {anchorCount=count;anchorUs=now;}
  }
  Snapshot snapshot() const {
    Snapshot result;result.size=size;result.direction=direction;result.invalid=invalid;
    result.count=previousCount;result.anchorCount=anchorCount;result.anchorUs=anchorUs;
    for (uint8_t i=0;i<result.size && i<5;++i) result.times[i]=times[i];
    return result;
  }
  static float evaluate(const Snapshot& sample,uint32_t now) {
    if (sample.invalid || sample.size>5) return NAN;
    if (sample.size<2) return 0;
    if (sample.direction!=1 && sample.direction!=-1) return NAN;
    uint32_t maximumGap=0;
    for (uint8_t i=1;i<sample.size;++i) {
      const uint32_t gap=sample.times[i]-sample.times[i-1];
      if (!gap || gap>500000u) return NAN;
      if (gap>maximumGap) maximumGap=gap;
    }
    const uint32_t span=sample.times[sample.size-1]-sample.times[0];
    const uint32_t age=now-sample.times[sample.size-1];
    if (age>0x7fffffffu) return NAN; // A snapshot from the future is not stale data.
    if (age>=2*maximumGap) return 0;
    const float decay=age<=maximumGap ? 1.f : 2.f-static_cast<float>(age)/maximumGap;
    constexpr float radiansPerCount=6.283185307179586f/4000;
    return sample.direction*radiansPerCount*(sample.size-1)*1e6f/span*decay;
  }
  // Period feedforward gives a fast response. Position feedback prevents its
  // integral from drifting on irregular edges. Anchor every four counts to
  // suppress repeatable quarter-edge placement error; never extrapolate past
  // one count beyond the latest observed edge. This remains an ABI estimate.
  float update(const Snapshot& sample,uint32_t now) {
    diagnosticNow=now;diagnosticEdge=sample.size && sample.size<=5 ? sample.times[sample.size-1]:0;
    diagnosticAge=now-diagnosticEdge;diagnosticGap=now-estimateUs;
    const float period=evaluate(sample,now);
    if (!std::isfinite(period)) {fault=invalidWindowOrClock;return value=NAN;}
    constexpr float tick=6.283185307179586f/4000;
    const float countAngle=static_cast<int32_t>(sample.count-originCount)*tick;
    const float anchorAngle=static_cast<int32_t>(sample.anchorCount-originCount)*tick;
    const uint32_t anchorAge=now-sample.anchorUs;
    float observed=countAngle;
    if (sample.size>=2 && period!=0) {
      const float predicted=anchorAngle+period*(anchorAge*1e-6f);
      observed=sample.direction>0 ? std::fmax(countAngle,std::fmin(countAngle+tick,predicted))
                                  : std::fmin(countAngle,std::fmax(countAngle-tick,predicted));
    }
    const uint32_t elapsed=now-estimateUs;
    estimateUs=now;
    if (!estimateValid) {
      estimateValid=true;estimateAngle=observed;angleRemainder=0;return value=0;
    }
    if (elapsed>sampleGapLimitUs) {fault=controlSampleGap;return value=NAN;}
    const float change=value*(elapsed*1e-6f)-angleRemainder;
    const float nextAngle=estimateAngle+change;
    angleRemainder=(nextAngle-estimateAngle)-change;estimateAngle=nextAngle;
    const float error=observed-estimateAngle;
    if (!std::isfinite(error)) {fault=nonfinitePhase;return value=NAN;}
    if (period==0 && std::abs(error)<1e-6f) {estimateAngle=observed;angleRemainder=0;return value=0;}
    return value=period+phaseGain*error;
  }
 private:
  volatile uint32_t previousCount=0,times[5]{};
  volatile uint32_t anchorCount=0,anchorUs=0;
  volatile uint8_t size=0;
  volatile int8_t direction=0;
  volatile bool invalid=false;
  uint32_t originCount=0,estimateUs=0;
  float estimateAngle=0,angleRemainder=0,value=0;
  bool estimateValid=false;
};
