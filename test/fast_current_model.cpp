// Hypothesis comparison across unidentified RL plants, NOT motor identification.
#include <TrackingCurrentPI.h>
#include <MagnitudeSlew.h>
#include <cmath>
#include <iostream>
struct Metrics{double square=0,peak=0,over=0;unsigned n=0;};
Metrics simulate(double resistance,double tau,int wave,int candidate,float limit=.7f,double emfEnd=.42){
  TrackingCurrentPI pi;pi.P=.6f;pi.I=candidate==1 ? 200.f:400.f;pi.trackingTimeS=.005f;pi.voltageLimit=limit;pi.reset(0);
  MagnitudeSlew slew{100.f};slew.voltageLimit=limit;slew.reset(0);
  const double dt=.0001,measurementTf=candidate==2 ? .0005:.001;
  double current=0,filtered=0,targetFiltered=0;Metrics result;
  for(unsigned us=100;us<=500000;us+=100){
    const double time=us*1e-6;
    const double target=wave==0 ? .4:.2+.2*std::sin(2*3.141592653589793*(wave==1 ? 60:150)*time);
    targetFiltered+=(target-targetFiltered)*dt/(.001+dt);
    const double command=candidate==3 ? targetFiltered:target;
    const double quantized=std::round(current/.025)*.025;
    filtered+=(quantized-filtered)*dt/(measurementTf+dt);
    float request=pi.request(static_cast<float>(command-filtered),us);
    request=std::max(-limit,std::min(limit,request));
    const float applied=slew.update(request,us);pi.track(applied);
    const double bemf=.1+(emfEnd-.1)*std::min(1.,time/.4);
    const double equilibrium=(applied-bemf)/resistance;
    current=equilibrium+(current-equilibrium)*std::exp(-dt/tau);
    if(pi.invalid || slew.invalid || !std::isfinite(current)){std::cerr<<"invalid model\n";std::exit(1);}
    if(us>=50000){++result.n;result.square+=(command-filtered)*(command-filtered);result.peak=std::max(result.peak,std::abs(current));result.over+=std::abs(current)>.5;}
  }
  result.square=std::sqrt(result.square/result.n);result.over/=result.n;return result;
}
int main(){
  unsigned betterPeak[4]={},betterRms[4]={},both[4]={},count=0;double meanRms[4]={},meanPeak[4]={},meanOver[4]={};
  std::cout<<"R,tau_s,wave,candidate,RMS,peak_raw_proxy,over_05_fraction\n";
  for(double r:{.2,.4,.7})for(double tau:{.00005,.0001,.0002,.0005,.001,.002})for(int wave:{0,1,2}){
    auto base=simulate(r,tau,wave,0);++count;
    for(int c=0;c<4;++c){auto value=c ? simulate(r,tau,wave,c):base;
      meanRms[c]+=value.square;meanPeak[c]+=value.peak;meanOver[c]+=value.over;
      betterPeak[c]+=value.peak<base.peak-1e-6;betterRms[c]+=value.square<base.square-1e-6;
      both[c]+=value.peak<base.peak-1e-6 && value.square<base.square-1e-6;
      std::cout<<r<<','<<tau<<','<<wave<<','<<c<<','<<value.square<<','<<value.peak<<','<<value.over<<'\n';
    }
  }
  for(int c=0;c<4;++c)std::cout<<"SUMMARY candidate="<<c<<" cases="<<count<<" lower_peak="<<betterPeak[c]<<" lower_RMS="<<betterRms[c]<<" both="<<both[c]<<" mean_RMS="<<meanRms[c]/count<<" mean_peak_proxy="<<meanPeak[c]/count<<" mean_over05="<<meanOver[c]/count<<'\n';
  std::cout<<"Candidates: 0=I400/LPF1ms baseline; 1=I200 only; 2=current LPF.5ms only; 3=target LPF1ms only. .025A quantization and .1->.42V imposed back-EMF; all unidentified assumptions.\n";
  double rms07=0,rms10=0,peak10=0,over10=0;
  for(double r:{.2,.4,.7})for(double tau:{.00005,.0001,.0002,.0005,.001,.002})for(int wave:{0,1,2}){
    const auto before=simulate(r,tau,wave,2,.7f,.6),after=simulate(r,tau,wave,2,1.f,.6);
    rms07+=before.square;rms10+=after.square;peak10=std::max(peak10,after.peak);over10+=after.over;
  }
  std::cout<<"ONE_VOLT_STRESS qLPF=.5ms cases=54 emf_end=.6V mean_RMS_07="<<rms07/54<<" mean_RMS_10="<<rms10/54<<" max_raw_proxy_10="<<peak10<<" mean_over05_10="<<over10/54<<" (not a real phase-current or motor guarantee)\n";
}
