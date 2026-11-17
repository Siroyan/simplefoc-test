#include "AdaptiveHarmonic.h"
#include <cstdlib>
#include <iostream>
void require(bool ok,const char* why){if(!ok){std::cerr<<"FAIL: "<<why<<'\n';std::exit(1);}}
// The previous order-14 four-trig calculation for valid samples, independently
// retained here to check that the two-trig rotation preserves commanded output.
struct Reference {
  float cosine=0,sine=0;uint32_t previous=0,start=0;
  float update(float angle,float error,float velocity,float pid,int direction,uint32_t now,float lag){
    const uint32_t dt=now-previous;previous=now;
    const uint32_t elapsed=now-start;
    if(elapsed<=800000)return std::max(-.45f,std::min(.45f,pid));
    const float phase=14*std::remainder(angle-.37f,6.28318530718f);
    const float blend=std::min(1.f,(elapsed-800000)/200000.f),seconds=dt*1e-6f;
    if(direction*velocity>=.3f*6.2831853f && direction*velocity<=1.7f*6.2831853f && std::abs(error)<=1.5f && std::abs(pid)<.45f-.01f){
      const float shifted=phase-direction*lag;
      cosine+=.2f*blend*error*std::cos(shifted)*seconds;
      sine+=.2f*blend*error*std::sin(shifted)*seconds;
    }
    const float norm=std::hypot(cosine,sine);
    if(norm>.01f){cosine*=.01f/norm;sine*=.01f/norm;}
    const float correction=std::max(-.01f,std::min(.01f,blend*(cosine*std::cos(phase)+sine*std::sin(phase))));
    return std::max(-.45f,std::min(.45f,pid+correction));
  }
};
int main(){
  float maxDifference=0;
  for(float lag:{0.f,.7853981634f,1.5707963268f})for(int sign:{-1,1}){
    const uint32_t start=0xfff00000u;
    AdaptiveHarmonic fast{.2f,.01f,lag,6.2831853f,14};fast.reset(.37f,start);
    Reference ref;ref.start=ref.previous=start;
    for(uint32_t elapsed=400;elapsed<3000000;elapsed+=137+elapsed%511){
      const float angle=.37f+sign*6.2831853f*elapsed*1e-6f;
      const float error=.7f*std::cos(14*angle)+.2f*std::sin(.0002f*elapsed);
      const float velocity=sign*6.2831853f-error;
      const float pid=elapsed%70000<10000 ? .449f:.2f;
      const float expected=ref.update(angle,error,velocity,pid,sign,start+elapsed,lag);
      const float actual=fast.update(angle,error,velocity,pid,.45f,sign,start+elapsed);
      const float difference=std::abs(expected-actual);maxDifference=std::max(maxDifference,difference);
      require(!fast.invalid && difference<2e-6f,"rotation matches prior output with signed phases, changing cadence, projection, clipping and clock wrap");
    }
  }
  std::cout<<"PASS: phase rotation vs previous four-trig reference, maximum total-command difference="<<maxDifference<<" nominal A\n";
}
