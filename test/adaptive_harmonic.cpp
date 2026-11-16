// Coupled diagnostic model, not motor identification. Actual SimpleFOC PID,
// project current PI/slew and quantized rolling velocity are executed here.
#include "AdaptiveHarmonic.h"
#include "TrackingCurrentPI.h"
#include "MagnitudeSlew.h"
#include "RollingVelocity.h"
#include "pid.h"
#include <cstdlib>
#include <iostream>
uint32_t simulationUs=0;
unsigned long _micros() {return simulationUs;}
void require(bool value,const char* message) {
  if (!value) {std::cerr<<"FAIL: "<<message<<'\n';std::exit(1);}
}
struct Result {double rms=0,lateRms=0;float peakVelocity=0,peakCorrection=0;uint32_t learned=0;};
Result simulate(float accelerationPerA,float phase,int sign,float rate,float cogging=.02f,float resistance=.6f,float currentTau=.001f,float lag=.7853981634f) {
  constexpr float dt=.00008f,tick=6.28318530718f/4000;
  simulationUs=0;
  PIDController velocityPI(.15f,7.2f,0,0,.45f,.0004f);
  TrackingCurrentPI currentPI;currentPI.P=.6f;currentPI.I=200;currentPI.reset(0);
  MagnitudeSlew slew{100,100};slew.reset(0);
  RollingVelocity estimator;estimator.reset(0,0);
  AdaptiveHarmonic adaptive{rate,.03f,lag};adaptive.reset(0,0);
  float angle=0,velocity=0,current=0,filteredCurrent=0,filteredVelocity=0,target=0,previousAngle=0;
  double squared=0,lateSquared=0;unsigned samples=0,lateSamples=0;Result result;
  for (simulationUs=80;simulationUs<=3000000;simulationUs+=80) {
    const float measuredAngle=std::round(angle/tick)*tick;
    if (simulationUs%400==0) {
      const float observed=estimator.update(measuredAngle,simulationUs);
      filteredVelocity+=(observed-filteredVelocity)*(.0004f/(.002f+.0004f));
      const float error=sign-filteredVelocity;
      target=adaptive.update(measuredAngle,error,filteredVelocity,velocityPI(error),.45f,sign,simulationUs);
    }
    // The 12-bit nominal current increment is about .029A. Deliberately include
    // deterministic sub-count noise; no claim of the bench noise spectrum.
    const float noise=.009f*std::sin(simulationUs*.00157f);
    const float quantizedCurrent=std::round((current+noise)/.0294f)*.0294f;
    filteredCurrent+=(quantizedCurrent-filteredCurrent)*(dt/(.002f+dt));
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
    require(!adaptive.invalid && !currentPI.invalid && !slew.invalid,"model stays finite and initialized");
    require(std::abs(target)<=.450001f && std::abs(voltage)<=.500001f && adaptive.peakA<=.030001f,"existing current/voltage and correction bounds hold");
    result.peakVelocity=std::max(result.peakVelocity,std::abs(velocity));
    if (simulationUs%10000==0) {
      const float windowVelocity=(measuredAngle-previousAngle)/.01f;
      previousAngle=measuredAngle;
      if (simulationUs>=800000) {squared+=(windowVelocity-sign)*(windowVelocity-sign);++samples;}
      if (simulationUs>=1500000) {lateSquared+=(windowVelocity-sign)*(windowVelocity-sign);++lateSamples;}
    }
  }
  result.rms=std::sqrt(squared/samples);result.lateRms=std::sqrt(lateSquared/lateSamples);
  result.peakCorrection=adaptive.peakA;result.learned=adaptive.learned;
  return result;
}
int main() {
  AdaptiveHarmonic invalid{.31f};require(invalid.invalid,"excessive learning rate rejected");
  AdaptiveHarmonic guard;guard.reset(0,UINT32_MAX-500);
  require(guard.update(0,0,1,0,.45f,1,499)==0,"early gate across wrap boundary");
  require(!guard.invalid,"legal clock wrap interval");
  require(guard.update(NAN,0,1,0,.45f,1,1499)==0 && guard.invalid,"nonfinite angle faults and clears output");
  guard.reset(0,0);require(guard.update(0,0,1,0,.46f,1,1000)==0 && guard.invalid,"larger total current bound refused");
  guard.reset(0,0);require(guard.update(0,0,1,0,.45f,1,21000)==0 && guard.invalid,"unexpected control gap faults");
  guard.reset(0,0);
  for (uint32_t now=1000;now<=3000000;now+=1000) {
    const float total=guard.update(0,1,1,0,.45f,1,now);
    require(!guard.invalid && std::abs(total)<=.450001f && std::hypot(guard.cosineA,guard.sineA)<=.030001f,"coefficient projection and total bound hold through sustained learning");
  }
  require(std::hypot(guard.cosineA,guard.sineA)>.029f,"test reaches the coefficient norm bound");
  const float peakAngle=std::atan2(guard.sineA,guard.cosineA)/84;
  guard.update(peakAngle,1,1,.449f,.45f,1,3001000);
  require(guard.clipped>0,"sum clipping exercised at existing total limit");
  for (float invalidValue : {NAN,INFINITY}) {
    guard.reset(0,0);guard.update(0,invalidValue,1,0,.45f,1,1000);require(guard.invalid && guard.correction==0,"invalid errors fail closed");
  }
  std::cout<<"acceleration_per_A,phase,sign,rate,baseline_RMS,adaptive_RMS,baseline_late_RMS,adaptive_late_RMS,peak_velocity,peak_correction_A\n";
  unsigned improved=0,total=0;double baselineSum=0,adaptiveSum=0;
  for (float acceleration : {400.f,700.f,1200.f}) for (float phase : {.3f,1.7f}) for (int sign : {-1,1}) {
    const Result baseline=simulate(acceleration,phase,sign,0);
    const Result adapted=simulate(acceleration,phase,sign,.2f);
    ++total;if (adapted.lateRms<baseline.lateRms) ++improved;
    baselineSum+=baseline.lateRms;adaptiveSum+=adapted.lateRms;
    require(adapted.peakVelocity<5,"native parameter envelope stays below existing velocity guard");
    std::cout<<acceleration<<','<<phase<<','<<sign<<",.2,"<<baseline.rms<<','<<adapted.rms<<','<<baseline.lateRms<<','<<adapted.lateRms<<','<<adapted.peakVelocity<<','<<adapted.peakCorrection<<'\n';
  }
  std::cerr<<"model_cases_improved="<<improved<<'/'<<total<<" mean_late_ratio="<<adaptiveSum/baselineSum<<"; not bench validation\n";
  require(improved==total && adaptiveSum<baselineSum*.8,"modeled periodic load is attenuated in the nominal envelope");
  unsigned stressImproved=0,stressCount=0;
  for (float resistance : {.5f,.7f}) for (float tau : {.0003f,.002f}) for (int sign : {-1,1}) for (float load : {0.f,.04f}) {
    const Result baseline=simulate(700,1.7f,sign,0,load,resistance,tau);
    const Result adapted=simulate(700,1.7f,sign,.2f,load,resistance,tau);
    ++stressCount;if(adapted.lateRms<baseline.lateRms) ++stressImproved;
    require(adapted.peakVelocity<5,"stress model remains below the existing guard");
    require(adapted.lateRms<=baseline.lateRms*1.2+.005,"stress model does not substantially amplify ripple, including no-load null");
  }
  std::cerr<<"stress_cases_improved="<<stressImproved<<'/'<<stressCount<<"; physical response remains unverified\n";
  unsigned flatImproved=0,flatCases=0;double flatBase=0,flatTotal=0;
  for (float acceleration : {400.f,700.f,1200.f,2000.f}) for (float phase : {.3f,1.7f}) for (int sign : {-1,1}) {
    const Result baseline=simulate(acceleration,phase,sign,0);
    const Result flat=simulate(acceleration,phase,sign,.2f,.02f,.6f,.001f,0);
    ++flatCases;if(flat.lateRms<baseline.lateRms) ++flatImproved;
    flatBase+=baseline.lateRms;flatTotal+=flat.lateRms;
    require(flat.peakVelocity<5 && flat.peakCorrection<=.030001f,"zero-lag model retains existing guard and correction limits");
    require(flat.lateRms<=baseline.lateRms*1.2+.005,"zero-lag model has no substantial amplification in selected envelope");
  }
  std::cerr<<"zero_lag_cases_improved="<<flatImproved<<'/'<<flatCases<<" mean_late_ratio="<<flatTotal/flatBase<<"; not a universal phase identification\n";
  std::cerr<<"PASS: limits, projection, sum clipping, invalid inputs, control gap, clock wrap and coupled model checks\n";
}
