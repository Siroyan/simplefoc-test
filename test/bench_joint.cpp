#include "BenchJoint.h"
#include "TraceSample.h"
#include <cstdlib>
#include <iostream>
void check(bool v,const char* why){if(!v){std::cerr<<why<<'\n';std::exit(1);}}
int main(){
  TraceSample row{};row.iqFiltered=.123f;row.iqTarget=.15f;row.ud=.02f;row.idFiltered=-.01f;
  row.jointTarget=.0087f;row.jointVelocityTarget=.017f;
  check(row.iqFiltered==.123f&&row.iqTarget==.15f&&row.ud==.02f&&row.idFiltered==-.01f,"Joint commands preserve every current/voltage payload");
  for(int s:{1,-1}){
    check(BenchJoint::reference(0,s)==0 && BenchJoint::reference(2700000,s)==0,"endpoints");
    check(std::abs(BenchJoint::reference(600000,s)-s*.5f*BenchJoint::degree)<1e-7,"peak reference");
    BenchJoint b;b.reset(0xfffffff0,s);
    for(uint32_t t=0;t<=2700000;t+=1000){
      float q=BenchJoint::reference(t,s),v=t>=100000&&t<200000?s*5*BenchJoint::degree:t>=1300000&&t<1400000?-s*5*BenchJoint::degree:0;
      float iq=b.update(0xfffffff0+t,t,q,v);
      check(!b.invalid&&std::isfinite(iq)&&std::abs(iq)<=BenchJoint::currentLimit+.000001f,"tracking/caps/wrap");
      check(std::abs(iq)<1e-5f,"matching ramp speed/position yields no spurious current");
    }
  }
  BenchJoint b;b.reset(0,1);b.update(1000,1000,4*BenchJoint::degree,0);
  check(b.invalid&&b.fault==BenchJoint::position,"position stop");
  b.reset(0,1);b.integral=.20f;
  check(b.update(1000,1000,.2f*BenchJoint::degree,0)<0&&std::abs(b.integral)<.003f,"opposing stored bias clears after overshoot");
  b.reset(0,1);b.integral=-.20f;
  check(b.update(1000,1000,-.2f*BenchJoint::degree,0)>0&&std::abs(b.integral)<.003f,"symmetric reversal release");
  b.reset(0,1);b.integral=.10f;
  check(b.update(1000,1000,0,0)==0&&b.holdingBand&&b.integral==0,"bare-motor hold band releases stored bias");
  b.reset(0,1);b.integral=.2f;
  check(b.update(1000,600000,.54f*BenchJoint::degree,.1f)<0&&b.holdingBand&&b.integral==0,"hold band supplies viscous damping");
  b.update(2000,601000,.56f*BenchJoint::degree,0);check(b.holdingBand,"hold hysteresis avoids boundary chatter");
  b.update(3000,602000,.59f*BenchJoint::degree,0);check(!b.holdingBand&&b.currentTarget<0,"hold exit resumes position correction");
  b.reset(0,1);b.update(6000,6000,0,0);check(b.fault==BenchJoint::gap,"gap stop");
  b.reset(0,1);b.update(0,0,NAN,0);check(b.fault==BenchJoint::nonfinite,"finite stop");
  b.reset(0,1);for(uint32_t t=1000;t<=20000;t+=1000)b.update(t,t,t*20*BenchJoint::degree/1e6f,0);
  check(b.fault==BenchJoint::speed,"independent window speed stop");
  b.reset(0,-1);for(uint32_t t=1000;t<100000;t+=1000){float i=b.update(t,600000,0,0);check(i<=0&&i>=-BenchJoint::currentLimit,"negative cap");}
  std::cout<<"PASS: trajectory, virtual joint scaling, signed bounded PI, independent speed/position, gaps, invalids, clock wrap\n";
}
