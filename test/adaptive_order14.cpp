#include "AdaptiveHarmonic.h"
#include <cstdlib>
#include <iostream>
void require(bool ok,const char* why){if(!ok){std::cerr<<"FAIL: "<<why<<'\n';std::exit(1);}}
struct Result{double rms=0;float peak=0;};
// Known first-order closed-loop response, not motor identification.
Result run(float rate,float gain,float tau,float amplitude,float phase,int sign){
  constexpr float target=6.2831853f,dt=.0004f;
  const uint32_t start=0xfff00000u;
  AdaptiveHarmonic model{rate,.01f,.7853981634f,target,14};model.reset(0,start);
  float response=0;double squared=0;unsigned samples=0;
  for(uint32_t us=400;us<=3000000;us+=400){
    const float angle=sign*target*us*1e-6f;
    const float disturbance=amplitude*std::cos(14*angle+phase)+.08f*std::sin(6.2831853f*48*us*1e-6f);
    const float error=disturbance-response;
    const float pid=sign*.2f;
    const float total=model.update(angle,error,sign*target-error,pid,.45f,sign,start+us);
    require(!model.invalid && std::abs(total)<=.450001f && std::abs(model.correction)<=.010001f,"14th-order and total bounds through clock wrap");
    if(us<=800000)require(model.correction==0,"startup gate");
    if(us<1000000)require(std::abs(model.correction)<=.01f*std::max(0.f,(us-800000.f)/200000.f)+1e-6f,"blend bound");
    response+=(gain*model.correction-response)*dt/(tau+dt);
    if(us>=1500000){squared+=error*error;++samples;}
  }
  return {std::sqrt(squared/samples),model.peakA};
}
int main(){
  unsigned cases=0,improved=0;double base=0,adapted=0;
  for(float gain:{6.f,10.f,14.f})for(float tau:{.001f,.004f,.008f})for(float amplitude:{.3f,.6f})for(float phase:{.3f,1.7f})for(int sign:{-1,1}){
    const auto before=run(0,gain,tau,amplitude,phase,sign),after=run(.2f,gain,tau,amplitude,phase,sign);
    ++cases;improved+=after.rms<before.rms;base+=before.rms;adapted+=after.rms;
  }
  std::cout<<"cases="<<cases<<" improved="<<improved<<" mean_RMS_before="<<base/cases<<" after="<<adapted/cases<<"; synthetic response only\n";
  require(improved==cases,"known response envelope suppresses the targeted harmonic");
  for(unsigned order:{0u,28u,100000u}){AdaptiveHarmonic bad{.2f,.01f,0,1,order};require(bad.invalid,"unsupported orders rejected");}
  AdaptiveHarmonic old{.2f,.03f,0,3.1f};require(old.invalid,"legacy reference restriction preserved");
  AdaptiveHarmonic high{.2f,.01f,.7853981634f,6.2831853f,14};require(!high.invalid,"explicit 60rpm reference accepted");
  for(uint32_t us=1000;us<=2000000;us+=1000)high.update(0,1,6,.2f,.45f,1,us);
  const float peakAngle=std::atan2(high.sineA,high.cosineA)/14;
  const auto learned=high.learned;
  high.update(peakAngle,1,6,.449f,.45f,1,2001000);
  require(high.clipped>0 && high.learned==learned,"total clipping and learning freeze near saturation");
  high.reset(0,0);require(high.order==14 && high.referenceMagnitude>6 && high.peakA==0,"reset clears learned state and preserves configuration");
  high.update(0,NAN,6,.2f,.45f,1,1000);require(high.invalid,"nonfinite error latches a fault");
  high.reset(0,0);high.update(0,1,6,.2f,.45f,1,21000);require(high.invalid,"sample gap latches a fault");
  AdaptiveHarmonic excess{.2f,.01f,0,6.29f,14};require(excess.invalid,"higher reference rejected");
  std::cout<<"PASS: order/reference selection, original restrictions, signed synthetic responses, startup/blend, total/correction bounds, clipping/freeze, clock wrap/reset/faults; not bench validation\n";
}
