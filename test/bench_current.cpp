#include "BenchCurrentPulse.h"
#include "BenchCurrentGuard.h"
#include "TrackingCurrentPI.h"
#include <cstdlib>
#include <iostream>
void require(bool ok,const char* why){if(!ok){std::cerr<<"FAIL: "<<why<<'\n';std::exit(1);}}
int main(){
  TrackingCurrentPI legacy,fresh;
  legacy.P=fresh.P=.6f;legacy.I=fresh.I=400;legacy.reset(1000);
  const float oldVoltage=legacy.request(.040491f,23300);
  fresh.reset(23300);const float newVoltage=fresh.request(.040491f,23302);
  require(oldVoltage>.38f && newVoltage<.025f && fresh.elapsedUs()==2,"UART gap excluded by first-FOC reset without changing PI gains");
  for(float amplitude:{.05f,.1f}){
    double integral=0;unsigned positive=0,negative=0;
    for(uint32_t us=0;us<=300000;us+=100){
      const float target=BenchCurrentPulse::target(us,amplitude);
      require(std::isfinite(target)&&std::abs(target)<=.100001f,"small bounded current reference");
      integral+=target;positive+=target>0;negative+=target<0;
      if(us<40000 || us>=270000)require(target==0,"initial and final zero intervals");
    }
    require(positive==400&&negative==400&&std::abs(integral)<1e-6,"balanced signed pulse durations");
  }
  require(std::isnan(BenchCurrentPulse::target(50000,.15f)),"unapproved larger pulse rejected");
  BenchCurrentGuard g;
  require(g.check(0,1,0,0,0,.49f,0,true) && g.check(2500,2,0,0,0,.49f,0,true),"new .50A dq range accepts .49A without changing phase bound");g.reset();
  require(g.check(0,1,.7f,0,0,0,0,true),"first overcurrent frame");
  require(g.check(10,1,.7f,0,0,0,0,true),"same ADC frame cannot increment overcurrent counter");
  require(g.check(20,2,.7f,0,0,0,0,true),"second fresh frame");
  require(!g.check(30,3,.7f,0,0,0,0,true)&&g.fault==BenchCurrentGuard::phase_overcurrent,"third fresh frame faults");
  require(!g.check(40,4,0,0,0,0,0,true),"fault latches");
  g.reset();require(g.check(0,1,1.8f,0,0,NAN,NAN,false),"alignment phase limit and no pre-alignment dq assumption");
  g.reset();require(g.check(0xfffffff0u,1,0,0,0,.55f,0,true),"dq timer starts");
  require(!g.check(1984,2,0,0,0,.55f,0,true)&&g.fault==BenchCurrentGuard::dq_overcurrent,"dq timing works over clock wrap");
  g.reset();require(!g.check(0,1,NAN,0,0,0,0,false),"invalid phase fails closed");
  g.reset();require(!g.check(0,1,0,0,0,NAN,0,true),"invalid running dq fails closed");
  g.reset();g.check(0,1,.7f,0,0,0,0,true);g.check(1,2,0,0,0,0,0,true);
  require(g.check(2,3,.7f,0,0,0,0,true)&&g.phaseFrames==1,"healthy frame resets consecutive counter");
  std::cout<<"PASS: complete signed pulse timing, caps, alignment/run separation, fresh-frame and timed dq current guards, wrap/reset/nonfinite/latching\n";
}
