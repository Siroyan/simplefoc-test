// Diagnostic model using actual firmware PI/slew. No claim of motor parameter
// identification: R=.5 ohm, time constant 2ms, synthetic quantization/noise.
#include "TrackingCurrentPI.h"
#include "MagnitudeSlew.h"
#include <cmath>
#include <iostream>
int main() {
  std::cout << "P,q_rise_V_s,mean_current_error_A,rms_current_error_A\n";
  for (float gain : {.6f,.8f}) for (float rate : {30.0f,100.0f}) {
    TrackingCurrentPI pi; pi.P=gain; pi.I=100; pi.reset(0);
    MagnitudeSlew slew{rate,rate}; slew.reset(0);
    float current=0,filtered=0,target=0;
    double sum=0,squared=0;unsigned count=0;
    for (uint32_t us=100;us<=3000000;us+=100) {
      const float quantizedRequest=.2f+(us%4000<2000 ? .12f : -.12f);
      target+=(quantizedRequest-target)/21;
      const float noise=(us/100%2 ? .03f : -.03f);
      filtered+=(current+noise-filtered)/21;
      const float request=pi.request(target-filtered,us);
      const float voltage=slew.update(request,us);
      pi.track(voltage);
      if (pi.invalid || slew.invalid || !std::isfinite(voltage) || std::abs(voltage)>.500001f) return 1;
      current+=.05f*(voltage/.5f-current);
      if (us>=400000) {
        const double error=filtered-target;sum+=error;squared+=error*error;++count;
      }
    }
    std::cout<<gain<<','<<rate<<','<<sum/count<<','<<std::sqrt(squared/count)<<'\n';
  }
}
