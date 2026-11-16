#pragma once
#include <SimpleFOC.h>
#include <VoltageVectorBound.h>
#include <MagnitudeSlew.h>
#include <TrackingCurrentPI.h>

class BoundedBLDCMotor : public BLDCMotor {
 public:
  using BLDCMotor::BLDCMotor;
  bool vectorBoundEnabled = false;
  bool asymmetricCurrentVoltage = false;
  bool trackingCurrentControl = false;
  TrackingCurrentPI qPI, dPI;
  MagnitudeSlew qSlew{30.0f}, dSlew{3.0f};
  VoltageVectorBound vectorBound;
  struct ActuatorStats {
    uint32_t startUs=0, lastUs=0, calls=0, coveredUs=0, slewUs=0, vectorUs=0, ignoreStartupUs=800000;
    float errorIntegral=0, errorSquaredIntegral=0, differenceIntegral=0, maxDifference=0;
    bool armed=false;
    void reset(uint32_t now,uint32_t startup=800000) { *this={};startUs=lastUs=now;ignoreStartupUs=startup;armed=true; }
    void record(uint32_t now,float requested,float beforeSlew,float applied,float error) {
      if (!armed) return;
      const uint32_t dt=now-lastUs;lastUs=now;
      if (now-startUs<ignoreStartupUs || dt>500000 || !std::isfinite(error) || !std::isfinite(requested)) return;
      ++calls;coveredUs+=dt;
      if (std::abs(beforeSlew-applied)>1e-6f) slewUs+=dt;
      if (std::abs(requested-beforeSlew)>1e-6f) vectorUs+=dt;
      const float seconds=dt*1e-6f;
      errorIntegral+=error*seconds;errorSquaredIntegral+=error*error*seconds;
      differenceIntegral+=(requested-applied)*seconds;
      maxDifference=std::max(maxDifference,std::abs(requested-applied));
    }
  } actuatorStats;
  void setPhaseVoltage(float q, float d, float angle) override {
    if (vectorBoundEnabled) {
      const uint32_t now = micros();
      if (trackingCurrentControl) {
        q = qPI.request(current_sp - current.q, now);
        d = dPI.request(-current.d, now);
        if (qPI.invalid || dPI.invalid) vectorBound.invalid = true;
      }
      const float qRequested=q;
      vectorBound.apply(q, d, angle, voltage_limit);
      q = vectorBound.q; d = vectorBound.d; angle = vectorBound.angle;
      const float qBeforeSlew=q;
      if (asymmetricCurrentVoltage && !vectorBound.invalid) {
        q = qSlew.update(q, now); d = dSlew.update(d, now);
      }
      if (trackingCurrentControl) { qPI.track(q); dPI.track(d); }
      if (trackingCurrentControl) actuatorStats.record(now,qRequested,qBeforeSlew,q,current_sp-current.q);
      voltage.q = q; voltage.d = d; // Log the applied vector, not the pre-limit request.
    }
    BLDCMotor::setPhaseVoltage(q, d, angle);
  }
};
