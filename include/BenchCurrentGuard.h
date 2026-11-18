#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

// Nominal ADC scaling only; not a calibrated physical current guarantee.
class BenchCurrentGuard {
 public:
  enum Fault {none,nonfinite,phase_overcurrent,dq_overcurrent};
  Fault fault=none;
  float peakPhase=0,peakIq=0,peakId=0;
  uint32_t dqSince=0,lastFrame=0;unsigned phaseFrames=0;
  bool dqOutside=false,frameValid=false;
  void reset(){*this={};}
  bool check(uint32_t now,uint32_t frame,float a,float b,float c,float iq,float id,bool driving) {
    if(fault!=none)return false;
    if(!std::isfinite(a)||!std::isfinite(b)||!std::isfinite(c)||
       (driving&&(!std::isfinite(iq)||!std::isfinite(id)))){fault=nonfinite;return false;}
    const float phase=std::max(std::abs(a),std::max(std::abs(b),std::abs(c)));
    peakPhase=std::max(peakPhase,phase);
    const float phaseLimit=driving ? .60f:2.2f; // Free-motor .7V alignment measured ~1.73A in saved survey.
    if(!frameValid || frame!=lastFrame){
      frameValid=true;lastFrame=frame;
      phaseFrames=phase>phaseLimit ? phaseFrames+1:0;
      if(phaseFrames>=3){fault=phase_overcurrent;return false;}
    }
    if(driving){
      peakIq=std::max(peakIq,std::abs(iq));peakId=std::max(peakId,std::abs(id));
      if(std::abs(iq)>.50f || std::abs(id)>.50f){
        if(!dqOutside){dqOutside=true;dqSince=now;}
        if(now-dqSince>=2000){fault=dq_overcurrent;return false;}
      }else dqOutside=false;
    }
    return true;
  }
  const char* reason() const {
    switch(fault){
      case nonfinite:return "nonfinite bench current";
      case phase_overcurrent:return "bench nominal phase current exceeded limit for 3 fresh frames";
      case dq_overcurrent:return "bench nominal dq current exceeded .50A for 2ms";
      default:return "no bench current fault";
    }
  }
};
