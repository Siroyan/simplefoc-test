// Diagnostic envelope, not motor identification. Use the actual PI, limiter
// and probe to compare a current-loop change before the low-current bench test.
#include "TrackingCurrentPI.h"
#include "MagnitudeSlew.h"
#include "VoltageVectorBound.h"
#include "CurrentProbe.h"
#include <cstdlib>
#include <iostream>
void require(bool condition,const char* message) {
  if (!condition) {std::cerr<<"FAIL: "<<message<<'\n';std::exit(1);}
}
int main() {
  constexpr float dt=.00008f, piValue=3.14159265358979323846f;
  std::cout<<"nominal_R,plant_tau_s,q_I,gain20,phase20_deg,mean_error_A,max_voltage_V\n";
  for (float resistance : {.5f,.7f}) for (float tau : {.0001f,.001f,.002f}) for (float integral : {100.0f,200.0f}) {
    TrackingCurrentPI controller;controller.P=.6f;controller.I=integral;controller.reset(0);
    MagnitudeSlew slew{100,100};slew.reset(0);
    CurrentProbe probe{20,.02f,400000,100000};
    float current=0,filtered=0,maxVoltage=0;
    double cosine=0,sine=0,sumError=0;unsigned samples=0;
    for (uint32_t us=80;us<=1000000;us+=80) {
      const float t=us*1e-6f;
      const float base=.1f*std::min(1.0f,t/.3f);
      const float target=probe.mix(base,us,.15f);
      const float noise=(us/80%2 ? .03f : -.03f);
      filtered+=(current+noise-filtered)*(dt/(.002f+dt));
      VoltageVectorBound vector;vector.apply(controller.request(target-filtered,us),0,0,.5f);
      const float voltage=slew.update(vector.q,us);controller.track(voltage);
      current+=(voltage/resistance-current)*(dt/tau);
      require(!controller.invalid && !probe.invalid && std::isfinite(current),"finite bounded model");
      require(std::abs(voltage)<=.500001f && std::abs(current)<.25f,"model voltage/current remain bounded");
      maxVoltage=std::max(maxVoltage,std::abs(voltage));
      if (us>=600000 && us<1000000) {
        const float phase=2*piValue*20*t;
        cosine+=filtered*std::cos(phase);sine+=filtered*std::sin(phase);
        sumError+=filtered-target;++samples;
      }
    }
    const double c=2*cosine/samples,s=2*sine/samples;
    const double gain=std::hypot(c,s)/.02;
    const double phase=std::atan2(c,s)*180/piValue;
    require(std::abs(sumError/samples)<.002,"settled signed mean error small in envelope");
    std::cout<<resistance<<','<<tau<<','<<integral<<','<<gain<<','<<phase<<','<<sumError/samples<<','<<maxVoltage<<'\n';
  }
}
