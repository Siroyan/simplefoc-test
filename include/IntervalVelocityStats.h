#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

// Whole post-ramp interval statistics, independent of the control estimator.
// Uses the same ABI angle; it is not an independent physical speed sensor.
class IntervalVelocityStats {
 public:
  static constexpr uint32_t periodUs=2000;
  uint32_t startUs=0,waitUs=0,firstUs=0,lastUs=0,coveredUs=0,samples=0,slowUs=0,reverseUs=0;
  float goal=0,firstAngle=0,lastAngle=0,errorSquareUs=0,minimum=0,maximum=0;
  bool active=false,invalid=false;
  void reset(uint32_t now,float target,uint32_t wait) {
    *this={};startUs=now;goal=target;waitUs=wait;
    invalid=!std::isfinite(target)||target==0||wait>=3000000;
  }
  void update(uint32_t now,float angle) {
    if (invalid) return;
    if (!std::isfinite(angle)) {invalid=true;return;}
    if (now-startUs<waitUs) return;
    if (!active) {firstUs=lastUs=now;firstAngle=lastAngle=angle;active=true;return;}
    const uint32_t dt=now-lastUs;
    if (dt<periodUs) return;
    if (dt>20000) {invalid=true;return;}
    const float velocity=(angle-lastAngle)*1e6f/dt;
    if (!std::isfinite(velocity)) {invalid=true;return;}
    if (!samples) minimum=maximum=velocity;
    minimum=std::min(minimum,velocity);maximum=std::max(maximum,velocity);
    const float error=velocity-goal;
    errorSquareUs+=error*error*dt;
    if (!std::isfinite(errorSquareUs)) {invalid=true;return;}
    coveredUs+=dt;++samples;
    if (std::abs(velocity)<.2f) slowUs+=dt;
    if (velocity*(goal>0 ? 1.f : -1.f)<-.2f) reverseUs+=dt;
    lastUs=now;lastAngle=angle;
  }
  float mean() const {return coveredUs ? (lastAngle-firstAngle)*1e6f/coveredUs : NAN;}
  float rmsError() const {return coveredUs ? std::sqrt(errorSquareUs/coveredUs) : NAN;}
};
static_assert(sizeof(IntervalVelocityStats)<=80,"Keep interval statistics small enough for the existing trace SRAM budget");
