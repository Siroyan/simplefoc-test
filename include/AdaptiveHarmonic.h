#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

// Experimental per-run harmonic correction for a guarded bench comparison.
// This is not an accepted calibration or a physical phase-current limit.
class AdaptiveHarmonic {
 public:
  const float learningRate, boundA, responseLagRad, referenceMagnitude;
  const unsigned order;
  bool invalid=false;
  float cosineA=0,sineA=0,correction=0,peakA=0;
  uint32_t calls=0,learned=0,clipped=0;
  explicit AdaptiveHarmonic(float rate=.2f,float bound=.03f,float lag=.7853981634f,float reference=1,unsigned harmonicOrder=84)
      :learningRate(rate),boundA(bound),responseLagRad(lag),referenceMagnitude(reference),order(harmonicOrder),cosLag(std::cos(lag)),sinLag(std::sin(lag)) {reset(0,0);}
  void reset(float angle,uint32_t now) {
    anchor=angle;startUs=previousUs=now;
    cosineA=sineA=correction=peakA=0;calls=learned=clipped=0;
    invalid=!std::isfinite(angle)||!std::isfinite(learningRate)||learningRate<0||learningRate>.3f||
      !std::isfinite(boundA)||boundA<=0||boundA>.03f||
      !std::isfinite(responseLagRad)||responseLagRad<0||responseLagRad>1.5707963268f||
      !std::isfinite(referenceMagnitude)||referenceMagnitude<=0||
      (order!=14 && order!=84)||referenceMagnitude>(order==14 ? 6.283186f : 3.f);
  }
  float update(float angle,float error,float velocity,float pid,float limit,int commandDirection,uint32_t now) {
    const uint32_t delta=now-previousUs;previousUs=now;
    if (invalid||delta>20000||!std::isfinite(angle)||!std::isfinite(error)||!std::isfinite(velocity)||
        !std::isfinite(pid)||!std::isfinite(limit)||limit<=0||limit>.45f||
        (commandDirection!=1 && commandDirection!=-1)) {
      invalid=true;return correction=0;
    }
    ++calls;
    const uint32_t elapsed=now-startUs;
    if (elapsed<=800000) return std::max(-limit,std::min(limit,pid));
    const float phase=order*std::remainder(angle-anchor,6.28318530718f);
    // Reuse the phase pair for order 14. Keep the legacy order-84 arithmetic.
    const float cosine=order==14 ? std::cos(phase):0;
    const float sine=order==14 ? std::sin(phase):0;
    const float blend=std::min(1.f,(elapsed-800000)/200000.f);
    const float seconds=delta*1e-6f;
    // Freeze learning near saturation, stalls, reversals and large transients.
    // Learning gate only. The independent whole-run speed guard is unchanged.
    if (commandDirection*velocity>=.3f*referenceMagnitude && commandDirection*velocity<=1.7f*referenceMagnitude &&
        std::abs(error)<=1.5f && std::abs(pid)<limit-boundA) {
      const float laggedPhase=phase-commandDirection*responseLagRad;
      cosineA+=learningRate*blend*error*(order==14 ? cosine*cosLag+commandDirection*sine*sinLag : std::cos(laggedPhase))*seconds;
      sineA+=learningRate*blend*error*(order==14 ? sine*cosLag-commandDirection*cosine*sinLag : std::sin(laggedPhase))*seconds;
      ++learned;
    }
    const float norm=std::hypot(cosineA,sineA);
    if (!std::isfinite(norm)) {invalid=true;return correction=0;}
    if (norm>boundA) {cosineA*=boundA/norm;sineA*=boundA/norm;}
    correction=blend*(cosineA*(order==14 ? cosine:std::cos(phase))+sineA*(order==14 ? sine:std::sin(phase)));
    if (!std::isfinite(correction)) {invalid=true;return correction=0;}
    correction=std::max(-boundA,std::min(boundA,correction));
    peakA=std::max(peakA,std::abs(correction));
    const float requested=pid+correction;
    const float applied=std::max(-limit,std::min(limit,requested));
    if (applied!=requested) ++clipped;
    return applied;
  }
 private:
  const float cosLag,sinLag;
  float anchor=0;
  uint32_t startUs=0,previousUs=0;
};
