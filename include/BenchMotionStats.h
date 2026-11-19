#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
struct BenchMotionStats {
  struct Segment {
    uint32_t samples=0,limited=0,slew=0,nearCurrent=0;
    float errorSquare=0,iqErrorSquare=0,maxError=0,peakVelocity=0,maxVector=0,minBus=100,maxBus=0;
  } segments[2]; // Only outbound/return moving phases; holds use whole RAM trace.
  bool invalid=false;
  void observe(unsigned phase,float error,float velocity,float iqError,float target,float uq,float ud,float bus,bool limited,bool slew,float currentLimit=.45f){
    if(phase>=5 || !std::isfinite(error)||!std::isfinite(velocity)||!std::isfinite(iqError)||!std::isfinite(target)||!std::isfinite(uq)||!std::isfinite(ud)||!std::isfinite(bus)||!std::isfinite(currentLimit)||currentLimit<=0){invalid=true;return;}
    if(phase!=1 && phase!=3)return;
    auto& s=segments[phase==1 ? 0:1];++s.samples;s.limited+=limited;s.slew+=slew;s.nearCurrent+=std::abs(target)>=currentLimit*.993f;
    s.errorSquare+=error*error;s.iqErrorSquare+=iqError*iqError;s.maxError=std::max(s.maxError,std::abs(error));
    s.peakVelocity=std::max(s.peakVelocity,std::abs(velocity));s.maxVector=std::max(s.maxVector,std::hypot(uq,ud));
    s.minBus=std::min(s.minBus,bus);s.maxBus=std::max(s.maxBus,bus);
  }
};
