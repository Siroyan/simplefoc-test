#include <TrackingCurrentPI.h>
#include <VoltageVectorBound.h>
#include <MagnitudeSlew.h>
#include <iostream>
#include <cstdlib>
void check(bool b,const char* s){if(!b){std::cerr<<s<<"\n";std::exit(1);}}
int main(){
  for(float limit:{.5f,.7f,1.f})for(float tracking:{0.f,.005f}){
    TrackingCurrentPI q,d;q.P=.6f;q.I=400;q.trackingTimeS=tracking;q.voltageLimit=d.voltageLimit=limit;
    MagnitudeSlew qs{100.f},ds{3.f};qs.voltageLimit=ds.voltageLimit=limit;q.reset(0xffff0000u);d.reset(0xffff0000u);qs.reset(0xffff0000u);ds.reset(0xffff0000u);
    float peak=0;
    for(uint32_t t=100;t<=400000;t+=100){
      const uint32_t now=0xffff0000u+t;const float sign=t<200000 ? 1.f:-1.f;
      VoltageVectorBound bound;bound.apply(q.request(sign,now),d.request(.2f*sign,now),0,limit);
      const float appliedQ=qs.update(bound.q,now),appliedD=ds.update(bound.d,now);
      q.track(appliedQ);d.track(appliedD);peak=std::max(peak,std::hypot(appliedQ,appliedD));
      check(!q.invalid&&!d.invalid,"finite independently configured signed PI/actuator pipeline");
      check(std::hypot(appliedQ,appliedD)<=limit+.000001f,"combined vector ceiling after independent slew");
    }
    check(peak>limit-.001f,"configured ceiling is reachable in the model");
    q.reset(0);check(q.voltageLimit==limit,"reset preserves configured limit");
    qs.reset(0);check(qs.voltageLimit==limit,"slew reset preserves configured ceiling");
    q.P=0;q.I=1000;const float requested=q.request(1,1000);check(std::abs(requested-limit)<.000001f,"integrator uses configured ceiling");
    q.track(limit+.01f);check(q.invalid,"over-ceiling actuator feedback rejected");
  }
  for(float limit:{0.f,-.1f,1.1f,NAN}){
    TrackingCurrentPI pi;pi.voltageLimit=limit;pi.reset(0);pi.request(.1f,100);check(pi.invalid,"invalid or unreviewed voltage range rejected");
    MagnitudeSlew slew;slew.voltageLimit=limit;slew.reset(0);slew.update(.1f,100);check(slew.invalid,"slew refuses invalid/unreviewed range");
  }
  TrackingCurrentPI legacy;legacy.reset(0);legacy.track(.6f);check(legacy.invalid,"default .5V feedback guard remains intact");
  std::cout<<"PASS voltage parameter: default .5 and explicit .7/1.0 PI/vector/slew bounds, signed reversal, reset/clock wrap, invalid ranges and actuator overshoot rejection\n";
}
