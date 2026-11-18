#include <BenchMotion.h>
#include <BenchMotionStats.h>
#include <iostream>
#include <cstdlib>
void check(bool x,const char* s){if(!x){std::cerr<<s<<"\n";std::exit(1);}}
int main(){
  for(int sign:{-1,1}){
    BenchMotion m;m.reset(0xffff0000u,sign);unsigned samples=0;uint32_t lastTrace=0;float peak=0;
    for(uint32_t t=0;t<=BenchMotion::durationUs;t+=100){
      float q,v;unsigned p;BenchMotion::reference(t,sign,q,v,p);peak=std::max(peak,std::abs(v)*8);
      const float current=m.update(0xffff0000u+t,t,q,v,v*8);
      check(!m.invalid&&std::isfinite(current)&&std::abs(current)<=BenchMotion::currentLimit+.000001f,"perfect track must remain valid and bounded across clock wrap");
      check(std::abs(m.velocityTarget)<=BenchMotion::peakMotor/8+.000001f,"position correction cannot demand more than declared peak speed");
      if(t==0||t-lastTrace>=BenchMotion::tracePeriod(t)){++samples;lastTrace=t;}
      check(sign*q>=-.000001f && sign*q<=BenchMotion::amplitude+.000001f,"trajectory cannot leave its signed range");
      if(p==0||p==2||p==4)check(v==0,"hold velocity must be zero");
    }
    check(std::abs(peak-BenchMotion::peakMotor)<.002f,"both trajectories must have their declared peak speed");
    check(samples<318,"whole outgoing/return/hold trace must fit SRAM with terminal sample");
  }
  BenchMotion b;b.reset(0,1);b.update(6000,0,0,0,0);check(b.fault==BenchMotion::gap,"gap guard");
  b.reset(0,1);b.update(1,0,NAN,0,0);check(b.fault==BenchMotion::nonfinite,"nonfinite guard");
  b.reset(0,1);b.update(1,0,6.1f*BenchMotion::degree,0,0);check(b.fault==BenchMotion::position,"position guard");
  b.reset(0,1);b.update(1,0,0,0,BenchMotion::speedLimit+.01f);check(b.fault==BenchMotion::speed,"independent speed guard");
  b.reset(0,1);for(uint32_t t=1000;t<=102000;t+=1000)b.update(t,0,2*BenchMotion::degree,0,0);check(b.fault==BenchMotion::tracking,"persistent tracking error guard");
  b.reset(0,1);b.update(1000,0,0,100,0);check(b.currentTarget==-BenchMotion::currentLimit,"signed negative torque clamp");
  b.reset(0,1);b.update(1000,0,0,-100,0);check(b.currentTarget==BenchMotion::currentLimit,"signed positive torque clamp");
  BenchMotionStats stats;stats.observe(1,.01f,2,.03f,BenchMotion::currentLimit,.3f,.4f,11,true,false);
  check(stats.segments[0].samples==1 && stats.segments[0].nearCurrent==1 && stats.segments[0].limited==1 && std::abs(stats.segments[0].maxVector-.5f)<1e-6f,"all-loop limit/current/voltage statistics");
  stats.observe(1,NAN,0,0,0,0,0,11,false,false);check(stats.invalid,"nonfinite statistics latch");
  std::cout<<"PASS motion rpm="<<BenchMotion::rpm<<" shape="<<BenchMotion::shape<<": waveform, whole trace, caps, wraps and independent faults\n";
}
