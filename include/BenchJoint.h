#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

// Standalone motor only. Joint coordinates are virtual: signed motor angle / 8.
class BenchJoint {
 public:
  static constexpr float currentLimit=.30f;
  static constexpr float velocityP=.05f;
  static constexpr float positionP=30;
  static constexpr float velocityI=3.0f;
  static constexpr float degree=.01745329252f;
  static constexpr uint32_t durationUs=2700000;
  static constexpr uint32_t rampUs=100000;
  float targetJoint=0,velocityTarget=0,currentTarget=0,integral=0,guardVelocity=0;
  bool invalid=false;
  bool holdingBand=false;
  enum Fault {none,nonfinite,gap,position,speed,tracking};
  Fault fault=none;
  int direction=1;
  static float reference(uint32_t elapsed,int sign){
    float shape=elapsed<100000 ? 0 : elapsed<100000+rampUs ? (elapsed-100000)/static_cast<float>(rampUs) :
      elapsed<1300000 ? 1 : elapsed<1300000+rampUs ? (1300000+rampUs-elapsed)/static_cast<float>(rampUs) : 0;
    return sign*.5f*degree*shape;
  }
  void reset(uint32_t now,int sign){
    *this={};direction=sign;lastUs=guardUs=now;
    if(sign!=1&&sign!=-1)fail(nonfinite);
  }
  float update(uint32_t now,uint32_t elapsed,float joint,float velocity){
    if(invalid)return 0;
    if(!std::isfinite(joint)||!std::isfinite(velocity))return fail(nonfinite);
    if(std::abs(joint)>3*degree)return fail(position);
    if(now-guardUs>=20000){
      guardVelocity=(joint-guardJoint)*1e6f/(now-guardUs);guardJoint=joint;guardUs=now;
      if(std::abs(guardVelocity)>15*degree)return fail(speed);
    }
    targetJoint=reference(elapsed,direction);
    const float error=targetJoint-joint;
    // A velocity-loop bias that pushes farther past the position target must
    // not remain stored while a small-angle reference reverses or settles.
    if(std::abs(error)>.02f*degree && integral*error<0)integral=0;
    if(std::abs(error)>degree){
      if(!outside){outside=true;outsideUs=now;}
      if(now-outsideUs>=100000)return fail(tracking);
    }else outside=false;
    const uint32_t dt=now-lastUs;
    if(dt>5000)return fail(gap);
    if(dt<1000)return currentTarget;
    lastUs=now;
    const float rampVelocity=.5f*degree*1e6f/rampUs;
    const float feed=elapsed>=100000&&elapsed<100000+rampUs ? direction*rampVelocity :
                     elapsed>=1300000&&elapsed<1300000+rampUs ? -direction*rampVelocity:0;
    if(feed!=0)holdingBand=false;
    else if(holdingBand && std::abs(error)>.075f*degree)holdingBand=false;
    else if(!holdingBand && std::abs(error)<=.05f*degree)holdingBand=true;
    if(holdingBand){
      integral=0;velocityTarget=0;
      return currentTarget=clip(-velocityP*8*velocity,currentLimit);
    }
    velocityTarget=clip(positionP*error+feed,5*degree);
    const float motorVelocityError=8*(velocityTarget-velocity);
    const float nextIntegral=clip(integral+velocityI*motorVelocityError*dt*1e-6f,currentLimit);
    const float request=velocityP*motorVelocityError+nextIntegral;
    if(std::abs(request)<=currentLimit || request*motorVelocityError<0)integral=nextIntegral;
    currentTarget=clip(velocityP*motorVelocityError+integral,currentLimit);
    return currentTarget;
  }
  const char* reason()const{
    switch(fault){
      case nonfinite:return "invalid bench joint input";
      case gap:return "bench joint control gap exceeded 5ms";
      case position:return "bench joint position exceeded 3deg";
      case speed:return "bench joint 20ms speed exceeded 15deg_s";
      case tracking:return "bench joint error exceeded 1deg for 100ms";
      default:return "no bench joint fault";
    }
  }
 private:
  uint32_t lastUs=0,guardUs=0,outsideUs=0;
  float guardJoint=0;bool outside=false;
  static float clip(float v,float limit){return std::max(-limit,std::min(limit,v));}
  float fail(Fault why){invalid=true;fault=why;return currentTarget=0;}
};
