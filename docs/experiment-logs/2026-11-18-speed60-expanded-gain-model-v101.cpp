// Coupled diagnostic model, not motor identification. Actual SimpleFOC PID,
// project current PI/slew and quantized rolling velocity are executed here.
#include "AdaptiveHarmonic.h"
#include "TrackingCurrentPI.h"
#include "MagnitudeSlew.h"
#include "RollingVelocity.h"
#include "pid.h"
#include "VelocityReference.h"
#include "WindowVelocity.h"
#include "ReferenceSpeedGuard.h"
#include <cstdlib>
#include <iostream>
uint32_t simulationUs=0;
unsigned long _micros() {return simulationUs;}
void require(bool value,const char* message) {
  if (!value) {std::cerr<<"FAIL: "<<message<<'\n';std::exit(1);}
}
struct Result {double rms=0,lateRms=0;float peakVelocity=0,peakCorrection=0,peakGuardVelocity=0;uint32_t learned=0,peakUs=0,envelopeFaults=0;float maximumEnvelopeExcess=0,steadyIntervalRms=0;uint32_t firstEnvelopeFaultUs=0;};
Result simulate(float accelerationPerA,float phase,int sign,float rate,float cogging=.02f,float resistance=.6f,float currentTau=.001f,float lag=.7853981634f,float velocityP=.15f,float speed=1,uint32_t rampUs=0,float velocityI=7.2f,float velocityTf=.002f,float currentTf=.002f,float adaptiveReference=1,bool shortWindow=false) {
  constexpr float dt=.00008f,tick=6.28318530718f/4000;
  simulationUs=0;
  PIDController velocityPI(velocityP,velocityI,0,0,.45f,.0004f);
  TrackingCurrentPI currentPI;currentPI.P=.6f;currentPI.I=200;currentPI.reset(0);
  MagnitudeSlew slew{100,100};slew.reset(0);
  RollingVelocity estimator;estimator.shortWindow=shortWindow;estimator.reset(0,0);
  WindowVelocity guardEstimator;guardEstimator.periodUs=2000;guardEstimator.reset(0,0);
  float guardFiltered=0;
  AdaptiveHarmonic adaptive{rate,.03f,lag,adaptiveReference};adaptive.reset(0,0);
  float angle=0,velocity=0,current=0,filteredCurrent=0,filteredVelocity=0,target=0,previousAngle=0;
  float previousIntervalAngle=0;double intervalSquared=0;unsigned intervalSamples=0;
  double squared=0,lateSquared=0;unsigned samples=0,lateSamples=0;Result result;
  for (simulationUs=80;simulationUs<=3000000;simulationUs+=80) {
    const float measuredAngle=std::round(angle/tick)*tick;
    if (simulationUs%400==0) {
      guardFiltered+=(guardEstimator.update(measuredAngle,simulationUs)-guardFiltered)*(.0004f/(.002f+.0004f));
      result.peakGuardVelocity=std::max(result.peakGuardVelocity,std::abs(guardFiltered));
      if (speed>5) {const auto decision=ReferenceSpeedGuard::evaluate(guardFiltered,velocityReference(sign*speed,simulationUs,rampUs),sign*speed);if(decision.outside){++result.envelopeFaults;if(!result.firstEnvelopeFaultUs)result.firstEnvelopeFaultUs=simulationUs;}result.maximumEnvelopeExcess=std::max(result.maximumEnvelopeExcess,std::max(decision.along-decision.upper,decision.lower-decision.along));}
      const float observed=estimator.update(measuredAngle,simulationUs);
      filteredVelocity+=(observed-filteredVelocity)*(.0004f/(velocityTf+.0004f));
      const float error=velocityReference(sign*speed,simulationUs,rampUs)-filteredVelocity;
      target=adaptive.update(measuredAngle,error,filteredVelocity,velocityPI(error),.45f,sign,simulationUs);
    }
    // The 12-bit nominal current increment is about .029A. Deliberately include
    // deterministic sub-count noise; no claim of the bench noise spectrum.
    const float noise=.009f*std::sin(simulationUs*.00157f);
    const float quantizedCurrent=std::round((current+noise)/.0294f)*.0294f;
    filteredCurrent+=(quantizedCurrent-filteredCurrent)*(dt/(currentTf+dt));
    const float request=currentPI.request(target-filteredCurrent,simulationUs);
    const float voltage=slew.update(std::max(-.5f,std::min(.5f,request)),simulationUs);
    currentPI.track(voltage);
    current+=(voltage/resistance-current)*(dt/currentTau);
    const float effort=current-cogging*std::cos(84*angle+phase);
    if (std::abs(velocity)<.005f && std::abs(effort)<.14f) velocity=0;
    else {
      const float direction=std::abs(velocity)>=.005f ? (velocity>0 ? 1.f : -1.f) : (effort>0 ? 1.f : -1.f);
      velocity+=accelerationPerA*(effort-.12f*direction-.01f*velocity)*dt;
    }
    angle+=velocity*dt;
    if(simulationUs==2000000)previousIntervalAngle=measuredAngle;
    else if(simulationUs>2000000 && simulationUs%2000==0) {const float v=(measuredAngle-previousIntervalAngle)/.002f;previousIntervalAngle=measuredAngle;intervalSquared+=(v-sign*speed)*(v-sign*speed);++intervalSamples;}
    require(!adaptive.invalid && !currentPI.invalid && !slew.invalid,"model stays finite and initialized");
    require(std::abs(target)<=.450001f && std::abs(voltage)<=.500001f && adaptive.peakA<=.030001f,"existing current/voltage and correction bounds hold");
    if (std::abs(velocity)>result.peakVelocity) {result.peakVelocity=std::abs(velocity);result.peakUs=simulationUs;}
    if (simulationUs%10000==0) {
      const float windowVelocity=(measuredAngle-previousAngle)/.01f;
      previousAngle=measuredAngle;
      if (simulationUs>=800000) {squared+=(windowVelocity-sign*speed)*(windowVelocity-sign*speed);++samples;}
      if (simulationUs>=1500000) {lateSquared+=(windowVelocity-sign*speed)*(windowVelocity-sign*speed);++lateSamples;}
    }
  }
  result.rms=std::sqrt(squared/samples);result.lateRms=std::sqrt(lateSquared/lateSamples);
  result.peakCorrection=adaptive.peakA;result.learned=adaptive.learned;
  result.steadyIntervalRms=intervalSamples ? std::sqrt(intervalSquared/intervalSamples) : 0;
  return result;
}





int main() {
 std::cout<<"P,I,inside_of_96,worst_excess,physical_peak,guard_peak,mean_steady2msRMS\n";
 for(float p:{.02f,.0225f,.025f,.0275f}) for(float integral:{.15f,.2f,.25f,.3f}) {
  unsigned count=0,inside=0;float peak=0,guard=0,excess=0;double rms=0;
  for(float acceleration:{400.f,700.f,2000.f,4000.f,8000.f,16000.f}) for(float resistance:{.5f,.7f}) for(float tau:{.0003f,.002f}) for(float phase:{.3f,1.7f}) for(int sign:{-1,1}) {
   const auto r=simulate(acceleration,phase,sign,0,.04f,resistance,tau,.7853981634f,p,ReferenceSpeedGuard::maximumGoal,2000000,integral,.0005f,.001f,1,true);
   ++count;if(!r.envelopeFaults)++inside;peak=std::max(peak,r.peakVelocity);guard=std::max(guard,r.peakGuardVelocity);excess=std::max(excess,r.maximumEnvelopeExcess);rms+=r.steadyIntervalRms;
  }
  std::cout<<p<<','<<integral<<','<<inside<<','<<excess<<','<<peak<<','<<guard<<','<<rms/count<<'\n';
 }
}
