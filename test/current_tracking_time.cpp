#include "TrackingCurrentPI.h"
#include "MagnitudeSlew.h"
#include "VoltageVectorBound.h"
#include <cstdlib>
#include <iostream>

void require(bool ok,const char* why){if(!ok){std::cerr<<"FAIL: "<<why<<'\n';std::exit(1);}}
struct Result{double meanError=0,rms=0;float peakCurrent=0;};
Result run(float tracking,float resistance,float tau,float hz,float noise,int sign,float qI=200,float currentTf=.001f) {
  constexpr float dt=.00008f;
  TrackingCurrentPI pi;pi.P=.6f;pi.I=qI;pi.trackingTimeS=tracking;pi.reset(0);
  MagnitudeSlew slew{100,100};slew.reset(0);
  float current=0,filtered=0,target=0;Result result;unsigned measured=0;
  for(uint32_t us=80;us<=1500000;us+=80) {
    if(us%400==0)target=sign*(.2f+.1f*std::sin(6.2831853f*hz*us*1e-6f)+((us/400)%2 ? .02f:-.02f));
    const float observed=std::round((current+noise*std::sin(us*.00157f))/.0294f)*.0294f;
    filtered+=(observed-filtered)*(dt/(currentTf+dt));
    VoltageVectorBound bound;bound.apply(pi.request(target-filtered,us),0,0,.5f);
    const float applied=slew.update(bound.q,us);pi.track(applied);
    require(!pi.invalid && !bound.invalid && std::abs(applied)<=.500001f,"actuator and integrator remain bounded");
    current+=((applied-sign*.15f)/resistance-current)*(dt/tau);
    result.peakCurrent=std::max(result.peakCurrent,std::abs(current));
    if(us>=500000) {const double error=target-current;result.meanError+=error;result.rms+=error*error;++measured;}
  }
  result.meanError/=measured;result.rms=std::sqrt(result.rms/measured);return result;
}
int main(){
  unsigned cases=0,betterBias=0,better5=0,better400=0,betterHalfFilter=0;double oldBias=0,newBias=0,oldRms=0,newRms=0,bias5=0,rms5=0,rms400=0,rmsHalfFilter=0;float peak400=0;
  for(float r:{.5f,.7f})for(float tau:{.0003f,.002f})for(float hz:{20.f,48.f,84.f})for(float noise:{.009f,.03f})for(int sign:{-1,1}) {
    const auto before=run(0,r,tau,hz,noise,sign),after=run(.002f,r,tau,hz,noise,sign);
    ++cases;betterBias+=std::abs(after.meanError)<std::abs(before.meanError);
    oldBias+=std::abs(before.meanError);newBias+=std::abs(after.meanError);oldRms+=before.rms;newRms+=after.rms;
    const auto five=run(.005f,r,tau,hz,noise,sign);
    better5+=std::abs(five.meanError)<std::abs(after.meanError) && five.rms<after.rms;
    bias5+=std::abs(five.meanError);rms5+=five.rms;
    const auto stronger=run(.005f,r,tau,hz,noise,sign,400),halfFilter=run(.005f,r,tau,hz,noise,sign,200,.0005f);
    better400+=stronger.rms<five.rms;betterHalfFilter+=halfFilter.rms<five.rms;
    rms400+=stronger.rms;rmsHalfFilter+=halfFilter.rms;peak400=std::max(peak400,stronger.peakCurrent);
    std::cout<<"case R="<<r<<" tau="<<tau<<" Hz="<<hz<<" noise="<<noise<<" sign="<<sign<<" bias_before="<<before.meanError<<" after="<<after.meanError<<" rms_before="<<before.rms<<" after="<<after.rms<<" peak_after="<<after.peakCurrent<<'\n';
  }
  std::cout<<"SUMMARY cases="<<cases<<" lower_abs_bias="<<betterBias<<" mean_abs_bias_before="<<oldBias/cases<<" after="<<newBias/cases<<" mean_RMS_before="<<oldRms/cases<<" after="<<newRms/cases<<'\n';
  std::cout<<"FIVE_MS lower_bias_and_RMS_vs_2ms="<<better5<<" mean_abs_bias="<<bias5/cases<<" mean_RMS="<<rms5/cases<<'\n';
  std::cout<<"CURRENT_I400 lower_RMS_vs_5ms="<<better400<<" mean_RMS="<<rms400/cases<<" peak_current="<<peak400<<" HALF_FILTER lower_RMS="<<betterHalfFilter<<" mean_RMS="<<rmsHalfFilter/cases<<'\n';
  for(float bad:{NAN,INFINITY,-.001f}){TrackingCurrentPI pi;pi.trackingTimeS=bad;pi.reset(0);require(pi.request(.1f,100)==0 && pi.invalid,"invalid tracking constant rejected");}
  for(float tracking:{0.f,.002f,.005f}) {
    TrackingCurrentPI pi;pi.P=.6f;pi.I=200;pi.trackingTimeS=tracking;pi.reset(0xfffffff0u);
    require(std::isfinite(pi.request(.1f,40)),"tracking clock wrap");pi.track(.001f);
    require(!pi.invalid && pi.trackingTimeS==tracking,"reset preserves explicit tracking configuration");
    pi.request(.1f,600041);require(pi.invalid,"tracking gap faults");
    pi.reset(0);pi.request(.1f,80);pi.trackingTimeS=NAN;pi.track(0);
    require(pi.invalid,"nonfinite feedback configuration rejected");
    for(float qI:{200.f,400.f}) {
    pi.trackingTimeS=tracking;pi.I=qI;pi.reset(0);MagnitudeSlew actuator{100,100};actuator.reset(0);
    float current=0,filtered=0;
    for(uint32_t us=80;us<=1200000;us+=80) {
      const float target=us<600000 ? .45f:-.45f;
      filtered+=(current-filtered)*(.00008f/.00108f);
      VoltageVectorBound bound;bound.apply(pi.request(target-filtered,us),0,0,.5f);
      const float applied=actuator.update(bound.q,us);pi.track(applied);
      current+=(applied/.6f-current)*(.00008f/.002f);
      require(!pi.invalid && std::abs(applied)<=.500001f,"step/reversal stays inside the applied vector bound");
      if(us==500000 || us==1200000)require(std::abs(target-current)<.001f,"both tracking modes settle after current steps and reversal");
    }
    }
  }
  require(betterBias>=cases*3/4 && newBias<oldBias*.5,"time-limited tracking reduces the modeled mean bias across most cases");
  require(better5==cases,"5ms improves bias and RMS over 2ms in every synthetic case");
  require(better400>=cases*3/4 && rms400<rms5,"current I400 improves average RMS across most synthetic conditions");
  std::cout<<"PASS: bounded applied vectors, signed quantized synthetic plants, tracking-time validation; not motor identification\n";
}
