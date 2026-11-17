#include "EdgeVelocity.h"
#include "RollingVelocity.h"
#include "ShortRunClock.h"
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

constexpr double tick=6.283185307179586/4000;
struct Event {uint32_t us;int32_t count;};
void require(bool ok,const char* message) {
  if (!ok) {std::cerr<<"FAIL: "<<message<<'\n';std::exit(1);}
}
void replay(const std::vector<Event>& events,uint32_t offset,bool jitter) {
  EdgeVelocity estimator;estimator.reset(uint32_t(events.front().count));
  size_t next=1;float previous=0;double integral=0;uint32_t previousUs=0,step=0;
  const uint32_t origin=events.front().us,end=events.back().us-origin+1100000;
  for (uint32_t us=28;us<end;us+=jitter ? (++step%7==0 ? 1300:43) : 28) {
    while (next<events.size() && events[next].us-origin<=us) {
      estimator.edge(uint32_t(events[next].count),events[next].us-origin+offset);++next;
    }
    integral+=previous*(us-previousUs)*1e-6;previousUs=us;
    previous=estimator.update(estimator.snapshot(),us+offset);
    require(std::isfinite(previous),"saved/uneven replay remains finite");
  }
  const double displacement=(events.back().count-events.front().count)*tick,error=std::abs(integral-displacement);
  std::cout<<"replay count="<<events.size()<<" wrap="<<bool(offset)<<" jitter="<<jitter<<" integral="<<integral<<" displacement="<<displacement<<" error="<<error<<'\n';
  require(error<std::max(.01,std::abs(displacement)*.02),"edge velocity integrates to signed count displacement within 2% or .01rad");
  require(previous==0,"stop expires rather than holding last speed");
}
void dynamics(bool modulation,int sign) {
  EdgeVelocity edges;edges.reset(0);
  constexpr double goal=6.283185307179586,omega=goal*84;
  auto angle=[&](double seconds) {
    if (modulation) return goal*seconds+.5/omega*(1-std::cos(omega*seconds));
    return seconds<2 ? .5*(goal/2)*seconds*seconds : goal*(seconds-1);
  };
  uint32_t n=0;double squared=0,sine=0,cosine=0,lag=0;unsigned samples=0;
  for (uint32_t us=400;us<=3000000;us+=400) {
    const double t=us*1e-6;
    while (angle(t)>=(n+1)*tick) {
      ++n;double low=0,high=t;
      for (int i=0;i<36;++i) {const double middle=(low+high)/2;if(angle(middle)<n*tick) low=middle;else high=middle;}
      edges.edge(uint32_t(sign*int32_t(n)),uint32_t(std::llround(high*1e6)));
    }
    const float observed=sign*edges.update(edges.snapshot(),us);
    require(std::isfinite(observed),"known dynamics remain finite");
    if (modulation && us>100000) {
      const double truth=goal+.5*std::sin(omega*t);
      squared+=std::pow(observed-truth,2);sine+=(observed-goal)*std::sin(omega*t);cosine+=(observed-goal)*std::cos(omega*t);++samples;
    } else if (!modulation && us>200000 && us<1800000) {
      lag+=(goal/2*t-observed)/(goal/2);++samples;
    }
  }
  if (modulation) {
    const double amplitude=2*std::hypot(sine,cosine)/samples,phase=std::atan2(cosine,sine)*180/3.141592653589793;
    std::cout<<"84Hz sign="<<sign<<" RMS_error="<<std::sqrt(squared/samples)<<" amplitude="<<amplitude<<" phase_deg="<<phase<<'\n';
    require(amplitude>.4 && amplitude<.6 && phase>-45 && phase<5,"real 84Hz speed ripple remains observable with limited phase lag");
  } else {
    std::cout<<"ramp sign="<<sign<<" equivalent_lag_s="<<lag/samples<<'\n';
    require(std::abs(lag/samples)<.0015,"60rpm ramp has less than 1.5ms mean equivalent lag");
  }
  edges.update(edges.snapshot(),3000400);
  require(!std::isfinite(edges.update(edges.snapshot(),3010400)) && edges.fault==EdgeVelocity::controlSampleGap,"unexpected 10ms control sample gap is rejected and classified");
}
int main(int argc,char** argv) {
  for (uint32_t origin : {0u,UINT32_MAX-10000}) {
    for (uint32_t us=0;us<=3000000;us+=997)
      require(ShortRunClock::elapsedUs(origin,origin+us*170u)==us,"cycle-clock wrap preserves all <=3s elapsed times");
    require(ShortRunClock::elapsedUs(origin,origin+3000000u*170u)==3000000u,"cycle clock exact 3s endpoint");
  }
  require(argc==2,"provide saved manual edge log");
  std::ifstream file(argv[1]);require(bool(file),"saved fixture exists");
  std::vector<Event> saved;bool reading=false;
  for (std::string line;std::getline(file,line);) {
    if (line.find("EDGES_LAST us,")==0 || line.find("RUN_EDGES_BEGIN:")==0) {reading=true;continue;}
    if (line.find("MANUAL_END:")==0 || line.find("RUN_EDGES_END")==0) break;
    if (!reading) continue;
    std::replace(line.begin(),line.end(),',',' ');Event e{};std::istringstream row(line);
    if (row>>e.us>>e.count) saved.push_back(e);
  }
  require(saved.size()>10,"saved edges are present");
  replay(saved,0,false);replay(saved,UINT32_MAX-50000,true);
  std::vector<Event> uneven{{0,0}};uint32_t time=0;int32_t count=0;
  for (unsigned i=0;i<2000;++i) {time+=i%2 ? 900:100;count+=i<1500 ? 1:-1;uneven.push_back({time,count});}
  replay(uneven,0,false);replay(uneven,UINT32_MAX-50000,true);
  dynamics(false,1);dynamics(false,-1);dynamics(true,1);dynamics(true,-1);
  EdgeVelocity clockFault;clockFault.reset(0);clockFault.edge(1,100);clockFault.edge(2,200);
  require(!std::isfinite(clockFault.update(clockFault.snapshot(),199)) && clockFault.fault==EdgeVelocity::invalidWindowOrClock,"future-snapshot fault is recorded without accepting regressed time");
  for (int sign : {-1,1}) {
    EdgeVelocity edges;edges.reset(0);RollingVelocity rolling;rolling.shortWindow=true;rolling.reset(0,0);
    double edgeSquared=0,rollingSquared=0;unsigned samples=0;int32_t n=0;
    const double offsets[]={0,.17,.23,-.11}; // Repeatable quarter-edge placement errors.
    auto edgeTime=[&](int32_t number) {return uint32_t(std::llround((number+offsets[number%4])*250+((number%3)-1)*2));};
    for (uint32_t us=80;us<1000000;us+=80) {
      while (edgeTime(n+1)<=us) {++n;edges.edge(uint32_t(sign*n),edgeTime(n));}
      const float observed=edges.update(edges.snapshot(),us);
      const float baseline=rolling.update(float(sign*n*tick),us);
      if (us>10000) {edgeSquared+=std::pow(observed-sign*6.28318530718,2);rollingSquared+=std::pow(baseline-sign*6.28318530718,2);++samples;}
    }
    const double edgeRms=std::sqrt(edgeSquared/samples),rollingRms=std::sqrt(rollingSquared/samples);
    std::cout<<"60rpm sign="<<sign<<" edge_RMS="<<edgeRms<<" LS2_RMS="<<rollingRms<<'\n';
    require(edgeRms<.05 && edgeRms<rollingRms*.5,"four-edge timing reduces known constant-speed quantization and quarter-edge errors");
  }
  EdgeVelocity invalid;invalid.reset(0);invalid.edge(2,100);
  require(!std::isfinite(EdgeVelocity::evaluate(invalid.snapshot(),200)),"skipped count propagates invalid state");
  invalid.reset(0);invalid.edge(1,100);invalid.edge(2,100);
  require(!std::isfinite(EdgeVelocity::evaluate(invalid.snapshot(),200)),"duplicate timestamps cannot produce infinite speed");
  invalid.reset(UINT32_MAX-1);invalid.edge(UINT32_MAX,UINT32_MAX-30);invalid.edge(0,20);
  require(EdgeVelocity::evaluate(invalid.snapshot(),30)>8.5,"count/time wrap works and overspeed is not clipped");
  invalid.edge(UINT32_MAX,100);require(EdgeVelocity::evaluate(invalid.snapshot(),110)==0,"reversal clears forward intervals");
  invalid.edge(UINT32_MAX-1,200);require(EdgeVelocity::evaluate(invalid.snapshot(),210)<0,"reverse speed follows new edges");
  require(!std::isfinite(EdgeVelocity::evaluate(invalid.snapshot(),199)),"future edge snapshot rejected");
  invalid.edge(UINT32_MAX-2,600000);require(EdgeVelocity::evaluate(invalid.snapshot(),600001)==0,"long pause restarts period acquisition");
  invalid.edge(UINT32_MAX-3,599999);require(!std::isfinite(EdgeVelocity::evaluate(invalid.snapshot(),600010)),"out-of-order edge timestamps are rejected");
  std::cout<<"PASS: saved edges, uneven intervals, both signs, quarter-edge errors, read jitter, reversal, stop, count/time wrap and invalid state\n";
}
