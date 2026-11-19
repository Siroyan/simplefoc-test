#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

// Free motor only: 70 joint-deg * 8, accelerating until nominal takeoff,
// then a separate 250ms smooth deceleration. Electrical bounds are unchanged.
class BenchHighSpeedMotion {
 public:
  static constexpr float degree=.01745329252f,currentLimit=.40f,velocityP=.12f,velocityI=1.2f;
  static constexpr unsigned rpm=BENCH_HIGH_SPEED_RPM,shape=2;
  static_assert(rpm==120||rpm==240||rpm==360||rpm==600||rpm==1200||rpm==2400||rpm==3733,"planned high-speed stages only");
  static constexpr uint32_t moveUs=rpm==3733 ? 50000:static_cast<uint32_t>((186666667ull+rpm/2)/rpm);
  static constexpr uint32_t initialUs=100000,brakeUs=250000,finalUs=400000;
  static constexpr uint32_t returnUs=initialUs+moveUs,durationUs=returnUs+brakeUs+finalUs;
  static constexpr uint32_t controlPeriodUs=250,controlGapUs=1000,errorDelayUs=20000;
  static constexpr float amplitude=70*degree,peakMotor=2*amplitude*8*1e6f/moveUs;
  static constexpr float finalJoint=amplitude+peakMotor/8*brakeUs*.5e-6f;
  static constexpr float positionLimit=finalJoint+5*degree,speedLimit=1.2f*peakMotor+2;
  static constexpr float velocityCommandFactor=1.10f,velocityCommandLimit=velocityCommandFactor*peakMotor;
  static_assert(velocityCommandLimit<speedLimit,"command recovery margin must remain inside independent protection");
  static_assert(durationUs<3000000,"whole acceleration/braking/hold must fit 3 seconds");
  float targetJoint=0,velocityTarget=0,currentTarget=0,integral=0;
  uint32_t controlUpdates=0,maxControlGap=0;
  bool invalid=false,holdingBand=false;
  int direction=1;unsigned phase=0;
  enum Fault {none,nonfinite,gap,position,speed,tracking};Fault fault=none;
  static void reference(uint32_t t,int sign,float& q,float& v,unsigned& p){
    if(t<initialUs){q=v=0;p=0;return;}
    if(t<returnUs){
      const float u=(t-initialUs)/static_cast<float>(moveUs),u2=u*u;
      q=sign*amplitude*u2*u*(2-u);v=sign*amplitude*(6*u2-4*u2*u)*1e6f/moveUs;p=1;return;
    }
    if(t<returnUs+brakeUs){
      const float u=(t-returnUs)/static_cast<float>(brakeUs),u2=u*u,u3=u2*u;
      q=sign*(amplitude+peakMotor/8*brakeUs*1e-6f*(u-u3+.5f*u3*u));
      v=sign*peakMotor/8*(1-3*u2+2*u3);p=3;return;
    }
    q=sign*finalJoint;v=0;p=4;
  }
  static uint32_t tracePeriod(uint32_t t){float q,v;unsigned p;reference(t,1,q,v,p);return p==1 ? moveUs/160:p==3 ? brakeUs/80:10000;}
  void reset(uint32_t now,int sign){*this={};lastUs=now;direction=sign;if(sign!=1&&sign!=-1)fail(nonfinite);}
  float update(uint32_t now,uint32_t elapsed,float q,float velocity,float independent){
    if(invalid)return 0;
    if(!std::isfinite(q)||!std::isfinite(velocity)||!std::isfinite(independent))return fail(nonfinite);
    if(direction*q < -5*degree || direction*q>positionLimit)return fail(position);
    if(std::abs(independent)>speedLimit)return fail(speed);
    float feed;reference(elapsed,direction,targetJoint,feed,phase);const float error=targetJoint-q;
    if(std::abs(error)>3*degree)return fail(tracking);
    if(std::abs(error)>degree){if(!outside){outside=true;outsideUs=now;}if(now-outsideUs>=errorDelayUs)return fail(tracking);}else outside=false;
    const uint32_t dt=now-lastUs;maxControlGap=std::max(maxControlGap,dt);if(dt>controlGapUs)return fail(gap);if(dt<controlPeriodUs)return currentTarget;
    lastUs=now;++controlUpdates;maxControlGap=std::max(maxControlGap,dt);
    if(std::abs(error)>.02f*degree && integral*error<0)integral=0;
    if(feed!=0)holdingBand=false;
    else if(holdingBand && std::abs(error)>.075f*degree)holdingBand=false;
    else if(!holdingBand && std::abs(error)<=.05f*degree)holdingBand=true;
    if(holdingBand){integral=0;velocityTarget=0;return currentTarget=clip(-velocityP*8*velocity,currentLimit);}
    velocityTarget=clip(30*error+feed,velocityCommandLimit/8);
    const float e=8*(velocityTarget-velocity),next=clip(integral+velocityI*e*dt*1e-6f,currentLimit),request=velocityP*e+next;
    if(std::abs(request)<=currentLimit || request*e<0)integral=next;
    return currentTarget=clip(velocityP*e+integral,currentLimit);
  }
  const char* reason()const{switch(fault){case nonfinite:return "nonfinite fast motion input";case gap:return "fast motion control gap exceeded 1ms";case position:return "fast motion position envelope exceeded";case speed:return "fast motion independent speed exceeded";case tracking:return "fast motion tracking exceeded 3deg or 1deg for 20ms";default:return "no fast motion fault";}}
 private:
  uint32_t lastUs=0,outsideUs=0;bool outside=false;
  static float clip(float v,float l){return std::max(-l,std::min(l,v));}
  float fail(Fault f){invalid=true;fault=f;return currentTarget=0;}
};
