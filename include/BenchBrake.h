#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
class BenchBrake {
 public:
  static constexpr float currentLimit=.22f;
  static constexpr float brakeCurrent=-.20f;
  static constexpr uint32_t rampUs=10000;
  unsigned phase=0;
  uint32_t transitionUs=0;
  uint32_t negativeTargetUs=0,negativeCurrentUs=0;
  float startVelocity=0,startGuard=0,currentTarget=0;
  bool active=false,released=false,invalid=false,complete=false;
  enum Fault {none,nonfinite,gap,position,speed_not_reached};
  Fault fault=none;
  void reset(uint32_t now,bool activeBrake){*this={};lastUs=now;active=activeBrake;}
  float update(uint32_t now,uint32_t elapsed,float motorAngle,float velocity,float guard){
    if(invalid||complete)return 0;
    if(!std::isfinite(motorAngle)||!std::isfinite(velocity)||!std::isfinite(guard))return fail(nonfinite);
    if(now-lastUs>5000)return fail(gap);
    lastUs=now;
    if(std::abs(motorAngle)>.41887902f)return fail(position); // 3deg at motor/8.
    if(phase<2){
      if(elapsed>=400000)return fail(speed_not_reached);
      phase=elapsed<40000 ? 0:1;
      if(elapsed>=40000+rampUs && velocity>=2 && guard>=2){
        phase=2;transitionUs=elapsed;startVelocity=velocity;startGuard=guard;
      }else return currentTarget=currentLimit*std::max(0.f,std::min(1.f,(static_cast<float>(elapsed)-40000)/rampUs));
    }
    if(velocity<=.1f)released=true;
    if(elapsed-transitionUs>=150000){phase=3;complete=true;return currentTarget=0;}
    return currentTarget=active&&!released ? brakeCurrent:0;
  }
  bool coast()const{return !active && phase>=2;}
  void observeCurrent(uint32_t elapsed,float applied,float iq){
    if(!active || phase<2 || invalid)return;
    if(applied<-.01f){
      if(!negativeTargetUs)negativeTargetUs=elapsed;
      if(iq<-.02f && !negativeCurrentUs)negativeCurrentUs=elapsed;
    }
  }
  const char* reason()const{
    switch(fault){
      case nonfinite:return "invalid bench brake input";
      case gap:return "bench brake gap exceeded 5ms";
      case position:return "bench brake displacement exceeded virtual 3deg";
      case speed_not_reached:return "bench brake required speed not reached within 400ms";
      default:return "no bench brake fault";
    }
  }
 private:
  uint32_t lastUs=0;
  float fail(Fault why){invalid=true;fault=why;return currentTarget=0;}
};
