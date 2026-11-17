#pragma once
#include <algorithm>
#include <cmath>

// A separately selected speed range. Legacy profiles retain their fixed 5rad/s
// guard. At reference=3 this envelope still rejects speeds above 5rad/s.
struct ReferenceSpeedGuard {
  static constexpr float maximumGoal=6.28318530718f;
  static constexpr float deviation=2, hardCeiling=8.5f;
  struct Result { bool valid=false, outside=true;float lower=0,upper=0,along=0; };
  static Result evaluate(float measuredMotorVelocity,float reference,float finalGoal) {
    Result result;
    if (!std::isfinite(measuredMotorVelocity)||!std::isfinite(reference)||!std::isfinite(finalGoal)||
        finalGoal==0 || std::abs(finalGoal)>maximumGoal+1e-6f ||
        std::abs(reference)>std::abs(finalGoal)+1e-6f || reference*finalGoal<0) return result;
    result.along=measuredMotorVelocity*(finalGoal>0 ? 1.f : -1.f);
    result.lower=std::abs(reference)-deviation;
    result.upper=std::min(hardCeiling,std::abs(reference)+deviation);
    result.valid=true;
    result.outside=result.along<result.lower || result.along>result.upper || std::abs(measuredMotorVelocity)>hardCeiling;
    return result;
  }
};
