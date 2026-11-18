#include "BenchBrake.h"
#include <iostream>
#include <cstdlib>
void check(bool v,const char* message){if(!v){std::cerr<<message<<'\n';std::exit(1);}}
int main(){
  for(bool active:{false,true}){
    BenchBrake b;b.reset(0xfffffff0,active);
    for(uint32_t t=0;t<=100000;t+=1000){
      float i=b.update(0xfffffff0+t,t,.05f,t>=100000?2.1f:0,t>=100000?2.1f:0);
      check(!b.invalid&&std::abs(i)<=BenchBrake::currentLimit+.000001f,"bounded acceleration/clock wrap");
      if(t<40000)check(i==0,"initial zero current");
    }
    check(b.phase==2&&b.transitionUs==100000&&b.coast()==!active,"matched trigger");
    check(b.currentTarget==(active?BenchBrake::brakeCurrent:0),"active/coast is sole changed factor");
    b.observeCurrent(100100,-.2f,.1f);b.observeCurrent(100200,-.2f,-.03f);b.observeCurrent(100300,-.2f,-.1f);
    check(b.negativeTargetUs==(active?100100u:0u)&&b.negativeCurrentUs==(active?100200u:0u),"first applied negative reference/current are latched only in active mode");
    for(uint32_t t=101000;t<=250000;t+=1000)b.update(0xfffffff0+t,t,.06f,t>=110000?-.1f:2,0);
    check(b.complete&&b.released&&b.currentTarget==0,"release latches at first near-zero speed, finish deadline");
  }
  BenchBrake b;b.reset(0,true);b.update(6000,0,0,0,0);check(b.fault==BenchBrake::gap,"gap rejects");
  b.reset(0,true);for(uint32_t t=1000;t<=50000;t+=1000)b.update(t,t,.01f,t>=50000?2.1f:0,t>=50000?2.1f:0);
  check(b.phase==2&&b.transitionUs==50000,"eligibility follows the actual ramp end, without an extra 10ms delay");
  b.reset(0,true);b.update(1,0,NAN,0,0);check(b.fault==BenchBrake::nonfinite,"nonfinite rejects");
  b.reset(0,true);b.update(1,0,.42f,0,0);check(b.fault==BenchBrake::position,"position bound");
  b.reset(0,true);b.update(1,400000,0,0,0);check(b.fault==BenchBrake::speed_not_reached,"no acceleration evidence means reject braking trial");
  std::cout<<"PASS: bounded signed current/trigger, coast/active separation, reversal release, wrap, deadlines and failed prerequisites\n";
}
