#include "CurrentProbe.h"
#include <cstdlib>
#include <iostream>
void require(bool condition,const char* message) {
  if (!condition) {std::cerr<<"FAIL: "<<message<<'\n';std::exit(1);}
}
int main() {
  for (float frequency : {10.0f,13.5f,17.5f,20.0f}) {
    CurrentProbe probe{frequency,.02f};
    const uint32_t start=UINT32_MAX-900000;
    double sum=0;unsigned samples=0;
    for (uint32_t elapsed=0;elapsed<=3000000;elapsed+=100) {
      const uint32_t now=start+elapsed;
      const float pid=.44f*std::sin(elapsed*1e-5f);
      const float output=probe.mix(pid,now-start,.45f);
      require(!probe.invalid && std::isfinite(output) && std::abs(output)<=.450001f,"bounded total across clock wrap");
      require(std::abs(probe.value)<=.020001f,"probe amplitude retained");
      if (elapsed<=800000) require(probe.value==0,"probe starts only after startup gate");
      if (elapsed>=800000 && elapsed<1000000)
        require(std::abs(probe.value)<=.02f*(elapsed-800000)/200000.0f+1e-6f,"activation blend bound");
      if (elapsed>=1000000 && elapsed<3000000) {sum+=probe.value;++samples;}
    }
    require(std::abs(sum/samples)<1e-5,"settled sine probe has no DC component");
    require(probe.peak>.0199f,"full probe amplitude exercised");
    CurrentProbe saturating{frequency,.02f};
    for (uint32_t us=1100000;us<1300000;us+=100) {
      const float matchingPid=std::copysign(.449f,std::sin(6.28318530718f*frequency*us*1e-6f));
      const float output=saturating.mix(matchingPid,us,.45f);
      require(std::abs(output)<=.450001f,"aligned saturation fixture retains the total bound");
    }
    require(saturating.clipped>0,"deterministic clipping exercised at every probe frequency");
  }
  for (float invalid : {NAN,INFINITY,-INFINITY}) {
    CurrentProbe probe{20,.02f};
    require(probe.mix(invalid,1200000,.45f)==0 && probe.invalid,"invalid PID faults");
    require(probe.mix(.2f,1200100,.45f)==0,"fault latches");
    CurrentProbe badFrequency{invalid,.02f};require(badFrequency.invalid,"invalid frequency refused");
    CurrentProbe badAmplitude{20,invalid};require(badAmplitude.invalid,"invalid amplitude refused");
    CurrentProbe badPhase{13.5f,.02f,800000,200000,invalid};require(badPhase.invalid,"invalid phase refused");
  }
  CurrentProbe excessive{20,.0201f};require(excessive.invalid,"amplitude above .02A refused");
  for (float frequency : {10.0f,20.0f}) {
  CurrentProbe stationary{frequency,.02f,400000,100000};
  double stationarySum=0;unsigned stationarySamples=0;
  for (uint32_t us=0;us<1000000;us+=100) {
    const float total=stationary.mix(.10f,us,.15f);
    require(total>=.079999f && total<=.120001f,"static probe remains .08 to .12A within .15A cap");
    if (us<=400000) require(stationary.value==0,"static probe gate retained");
    if (us>=500000) {stationarySum+=stationary.value;++stationarySamples;}
  }
  require(std::abs(stationarySum/stationarySamples)<1e-5,"static probe settled waveform has no DC");
  }
  CurrentProbe early{20,.02f,100000,100000};require(early.invalid,"unapproved early activation refused");
  CurrentProbe probe{20,.02f};
  require(probe.mix(.2f,1200000,.46f)==0 && probe.invalid,"existing .45A total ceiling cannot be raised");
  CurrentProbe straight{13.5f,.02f},inverse{13.5f,.02f,800000,200000,3.14159265f};
  for (uint32_t us=0;us<=3000000;us+=137) {
    const float forward=straight.mix(.15f,us,.45f),backward=inverse.mix(.15f,us,.45f);
    require(std::abs(straight.value+inverse.value)<1e-6f,"inverse-phase probe cancels at the same timestamp");
    require(std::abs(forward+backward-.30f)<1e-6f,"inverse-phase comparison preserves mean current request");
    require(std::abs(inverse.value)<=.020001f && !inverse.invalid,"inverse probe retains existing amplitude bound");
  }
  CurrentProbe excessivePhase{13.5f,.02f,800000,200000,3.15f};require(excessivePhase.invalid,"unbounded phase configuration refused");
  std::cout<<"PASS: startup/blend, bounded probe/total, zero DC, clock wrap, invalid input and latch\n";
}
