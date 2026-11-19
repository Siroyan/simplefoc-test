#pragma once
#ifdef BENCH_HIGH_SPEED_RPM
#include <BenchHighSpeedMotion.h>
using BenchMotion = BenchHighSpeedMotion;
#else
#include <algorithm>
#include <cmath>
#include <cstdint>
#ifndef BENCH_MOTION_RPM
#define BENCH_MOTION_RPM 10
#endif
#ifndef BENCH_MOTION_SHAPE
#define BENCH_MOTION_SHAPE 0
#endif
#ifndef BENCH_MOTION_VELOCITY_P
#define BENCH_MOTION_VELOCITY_P .12f
#endif
// Motor only. Coordinates, gains and hard error guard are in motor/8 radians.
class BenchMotion {
 public:
  static constexpr float degree=.01745329252f,currentLimit=.45f,velocityP=BENCH_MOTION_VELOCITY_P,velocityI=1.2f;
  static constexpr unsigned rpm=BENCH_MOTION_RPM,shape=BENCH_MOTION_SHAPE;
  static_assert(rpm==10||rpm==20||rpm==40||rpm==60,"validated speed stages only");
  static_assert(shape<=1,"quintic or trapezoidal speed only");
  static constexpr float amplitude=3.75f*degree,peakMotor=rpm*.10471975512f;
  static constexpr uint32_t moveUs=shape==0 ? 9375000/rpm : 5000000/rpm+62500;
  static constexpr uint32_t returnUs=100000+moveUs+400000,durationUs=returnUs+moveUs+500000;
  static constexpr float speedLimit=peakMotor+2>5 ? peakMotor+2:5;
  static_assert(durationUs<3000000,"all motion must fit 3 seconds");
  float targetJoint=0,velocityTarget=0,currentTarget=0,integral=0;
  bool invalid=false,holdingBand=false;
  int direction=1;unsigned phase=0;
  enum Fault {none,nonfinite,gap,position,speed,tracking};Fault fault=none;
  static void reference(uint32_t t,int sign,float& q,float& v,unsigned& p){
    if(t<100000){q=v=0;p=0;return;}
    bool backward=t>=returnUs;
    uint32_t local=t-(backward ? returnUs:100000);
    if(local>=moveUs){q=backward ? 0:sign*amplitude;v=0;p=backward ? 4:2;return;}
    p=backward ? 3:1;
    float x=0,dx=0;
    if(shape==0){
      const float u=local/static_cast<float>(moveUs),u2=u*u,u3=u2*u;
      x=u3*(10+u*(-15+6*u));dx=30*u2*(1-u)*(1-u)*1e6f/moveUs;
    }else{
      const float time=local*1e-6f,total=moveUs*1e-6f,vmax=peakMotor/8;
      float distance,speed;
      if(time<.1f){speed=vmax*time/.1f;distance=.5f*speed*time;}
      else if(time<total-.025f){speed=vmax;distance=vmax*(time-.05f);}
      else{const float left=total-time;speed=vmax*left/.025f;distance=amplitude-.5f*speed*left;}
      x=distance/amplitude;dx=speed/amplitude;
    }
    q=sign*amplitude*(backward ? 1-x:x);v=sign*amplitude*dx*(backward ? -1:1);
  }
  static uint32_t tracePeriod(uint32_t t){float q,v;unsigned p;reference(t,1,q,v,p);return p==1||p==3 ? moveUs/96:10000;}
  void reset(uint32_t now,int sign){*this={};lastUs=now;direction=sign;if(sign!=1&&sign!=-1)fail(nonfinite);}
  float update(uint32_t now,uint32_t elapsed,float q,float velocity,float independent){
    if(invalid)return 0;
    if(!std::isfinite(q)||!std::isfinite(velocity)||!std::isfinite(independent))return fail(nonfinite);
    if(std::abs(q)>6*degree)return fail(position);
    if(std::abs(independent)>speedLimit)return fail(speed);
    float feed;reference(elapsed,direction,targetJoint,feed,phase);
    const float error=targetJoint-q;
    if(std::abs(error)>degree){if(!outside){outside=true;outsideUs=now;}if(now-outsideUs>=100000)return fail(tracking);}else outside=false;
    const uint32_t dt=now-lastUs;if(dt>5000)return fail(gap);if(dt<1000)return currentTarget;lastUs=now;
    if(std::abs(error)>.02f*degree && integral*error<0)integral=0;
    if(feed!=0)holdingBand=false;
    else if(holdingBand && std::abs(error)>.075f*degree)holdingBand=false;
    else if(!holdingBand && std::abs(error)<=.05f*degree)holdingBand=true;
    if(holdingBand){integral=0;velocityTarget=0;return currentTarget=clip(-velocityP*8*velocity,currentLimit);}
    velocityTarget=clip(30*error+feed,peakMotor/8);
    const float e=8*(velocityTarget-velocity),next=clip(integral+velocityI*e*dt*1e-6f,currentLimit),request=velocityP*e+next;
    if(std::abs(request)<=currentLimit || request*e<0)integral=next;
    return currentTarget=clip(velocityP*e+integral,currentLimit);
  }
  const char* reason()const{switch(fault){case nonfinite:return "nonfinite motion input";case gap:return "motion control gap exceeded 5ms";case position:return "motion position exceeded 6deg";case speed:return "motion independent speed exceeded";case tracking:return "motion tracking error exceeded 1deg for 100ms";default:return "no motion fault";}}
 private:
  uint32_t lastUs=0,outsideUs=0;bool outside=false;
  static float clip(float v,float l){return std::max(-l,std::min(l,v));}
  float fail(Fault f){invalid=true;fault=f;return currentTarget=0;}
};
#endif
