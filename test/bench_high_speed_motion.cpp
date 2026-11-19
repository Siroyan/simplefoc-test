#include <BenchMotion.h>
#include <BenchMotionStats.h>
#include <iostream>
#include <cstdlib>
void check(bool value,const char* text){if(!value){std::cerr<<text<<"\n";std::exit(1);}}
int main(){
  for(int sign:{-1,1}){
    BenchMotion m;m.reset(0xffff0000u,sign);uint32_t lastTrace=0;unsigned samples=0;
    float peak=0,previousQ=0,previousV=0;double integral=0;
    for(uint32_t t=0;t<BenchMotion::durationUs+100;t+=50){
      float q,v;unsigned phase;BenchMotion::reference(t,sign,q,v,phase);
      const float current=m.update(0xffff0000u+t,t,q,v,v*8);
      check(!m.invalid && std::isfinite(current) && std::abs(current)<=BenchMotion::currentLimit+.000001f,"perfect signed track/caps/wrap");
      check(sign*q>=sign*previousQ-.00001f && sign*q<=BenchMotion::finalJoint+.00001f,"monotonic finite stroke and brake allowance");
      check(std::abs(m.velocityTarget)<=BenchMotion::velocityCommandLimit/8+.00001f,"velocity command cap");
      if(phase==0 || phase==4)check(v==0,"stationary initial/final hold");
      if(t)integral+=(previousV+v)*.5*.00005;
      if(!t || t-lastTrace>=BenchMotion::tracePeriod(t)){++samples;lastTrace=t;}
      peak=std::max(peak,std::abs(v)*8);previousQ=q;previousV=v;
    }
    check(std::abs(integral-sign*BenchMotion::finalJoint)<.00004,"velocity integration matches full stroke including separate brake");
    check(std::abs(peak-BenchMotion::peakMotor)<.002,"true declared peak");
    check(samples<318 && samples>260,"whole trace including brake and holds fits SRAM");
    check(m.maxControlGap==250 && m.controlUpdates>100,"bounded controller update schedule");
    float q,v;unsigned phase;BenchMotion::reference(BenchMotion::returnUs,sign,q,v,phase);
    check(std::abs(q-sign*BenchMotion::amplitude)<.000002 && std::abs(v*8-sign*BenchMotion::peakMotor)<.00002 && phase==3,"takeoff velocity remains nonzero and brake is continuous");
    BenchMotion::reference(BenchMotion::returnUs+BenchMotion::brakeUs,sign,q,v,phase);
    check(q==sign*BenchMotion::finalJoint && v==0 && phase==4,"brake reaches exact final hold");
  }
  for(int sign:{-1,1}){
    float q,v;unsigned phase;BenchMotion::reference(BenchMotion::returnUs,sign,q,v,phase);
    BenchMotion recovery;recovery.reset(0,sign);
    recovery.update(250,BenchMotion::returnUs,q-sign*.5f*BenchMotion::degree,v,v*8);
    check(!recovery.invalid && sign*recovery.velocityTarget>BenchMotion::peakMotor/8,"position lag can recover at reference peak");
    check(std::abs(recovery.velocityTarget)<=BenchMotion::velocityCommandLimit/8+.00001f,"recovery command bounded");
    check(BenchMotion::velocityCommandLimit<BenchMotion::speedLimit && BenchMotion::speedLimit==1.2f*BenchMotion::peakMotor+2,"independent speed protection unchanged");
  }
  BenchMotion b;b.reset(0,1);b.update(1001,0,0,0,0);check(b.fault==BenchMotion::gap,"1ms gap fault");
  b.reset(0,1);b.update(1,0,NAN,0,0);check(b.fault==BenchMotion::nonfinite,"nonfinite fault");
  b.reset(0,1);b.update(1,0,-6*BenchMotion::degree,0,0);check(b.fault==BenchMotion::position,"negative boundary");
  b.reset(0,-1);b.update(1,0,-BenchMotion::positionLimit-.001f,0,0);check(b.fault==BenchMotion::position,"signed upper boundary");
  b.reset(0,1);b.update(1,0,0,0,BenchMotion::speedLimit+.01f);check(b.fault==BenchMotion::speed,"independent speed fault");
  b.reset(0,1);b.update(1,0,4*BenchMotion::degree,0,0);check(b.fault==BenchMotion::tracking,"instant tracking error");
  b.reset(0,1);for(uint32_t t=250;t<=20500;t+=250)b.update(t,0,1.2f*BenchMotion::degree,0,0);
  check(b.fault==BenchMotion::tracking && b.currentTarget==0,"sustained 20ms error");
  check(b.update(20750,0,0,0,0)==0 && b.invalid,"fault remains latched");
  b.reset(21000,1);check(!b.invalid && b.currentTarget==0 && b.integral==0,"reset clears all state");
  b.reset(0,0);check(b.invalid,"bad direction rejected");
  b.reset(0,1);b.update(250,0,0,-100,0);check(!b.invalid && std::abs(b.currentTarget-BenchMotion::currentLimit)<.000001f,"saturated request obeys actual current command cap");
  BenchMotionStats stats;stats.observe(1,0,0,0,BenchMotion::currentLimit*.995f,0,0,11,false,false,BenchMotion::currentLimit);
  check(stats.segments[0].nearCurrent==1,"statistics threshold follows actual command cap");
  std::cout<<"PASS high-speed rpm="<<BenchMotion::rpm<<": signed waveform, nonzero takeoff, separate brake, 3s/SRAM bounds, current/velocity caps, clock wrap and each fault\n";
}
