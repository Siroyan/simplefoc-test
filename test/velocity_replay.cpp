// Native test of the actual firmware estimator, not a Python reimplementation.
#include "WindowVelocity.h"
#include "RollingVelocity.h"
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <type_traits>

struct Event { uint32_t us; int32_t count; };
constexpr double radiansPerCount = 6.283185307179586 / 4000;

void require(bool condition, const char* message) {
  if (!condition) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}

template<class Estimator = WindowVelocity>
void replay(const std::vector<Event>& events, bool jitter, uint32_t clockOffset, uint32_t windowUs = 10000) {
  require(events.size() > 1, "fixture has enough edges");
  Estimator estimator;
  if constexpr (std::is_same_v<Estimator,WindowVelocity>) estimator.periodUs = windowUs;
  if constexpr (std::is_same_v<Estimator,RollingVelocity>) {estimator.longWindow = windowUs == 8000;estimator.shortWindow=windowUs==2000;}
  const uint32_t origin = events.front().us;
  estimator.reset(float(events.front().count * radiansPerCount), clockOffset);
  size_t next = 1;
  int32_t count = events.front().count;
  uint32_t previousTime = 0, step = 0;
  double area = 0;
  float previousSpeed = 0;
  const uint32_t end = events.back().us - origin + 150000;
  for (uint32_t time = 28; time < end; time += jitter ? (++step % 7 == 0 ? 1300 : 43) : 28) {
    while (next < events.size() && events[next].us - origin <= time) count = events[next++].count;
    area += previousSpeed * (time - previousTime) * 1e-6;
    previousSpeed = estimator.update(float(count * radiansPerCount), time + clockOffset);
    require(std::isfinite(previousSpeed), "finite speed throughout replay");
    previousTime = time;
  }
  const double displacement = (events.back().count - events.front().count) * radiansPerCount;
  const double error = std::abs(area - displacement);
  std::cout << "replay events=" << events.size() << " window_us=" << windowUs << " jitter=" << jitter
            << " wrap=" << bool(clockOffset) << " count_angle=" << displacement
            << " velocity_integral=" << area << " error_rad=" << error << '\n';
  require(error < std::max(0.01, std::abs(displacement) * 0.02), "integrated speed agrees with net counts within 2% or 0.01rad");
  require(std::abs(previousSpeed) < 1e-6, "speed returns to zero after stop");
}

int main(int argc, char** argv) {
  require(argc == 2, "provide saved manual log");
  std::ifstream input(argv[1]);
  require(bool(input), "open saved log");
  std::vector<Event> saved;
  bool reading = false;
  for (std::string line; std::getline(input, line);) {
    if (line.find("EDGES_LAST us,") == 0) { reading = true; continue; }
    if (line.find("MANUAL_END:") == 0) break;
    if (!reading) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    Event event{};
    std::istringstream row(line);
    if (row >> event.us >> event.count) saved.push_back(event);
  }
  replay(saved, false, 0);
  replay(saved, true, 0);
  replay(saved, true, UINT32_MAX - 50000);
  replay(saved, false, 0, 2000);
  replay(saved, true, UINT32_MAX - 50000, 2000);
  replay<RollingVelocity>(saved,false,0,2000);
  replay<RollingVelocity>(saved,true,UINT32_MAX-50000,2000);
  replay<RollingVelocity>(saved,false,0,4000);
  replay<RollingVelocity>(saved,true,UINT32_MAX-50000,4000);
  replay<RollingVelocity>(saved,false,0,8000);
  replay<RollingVelocity>(saved,true,UINT32_MAX-50000,8000);

  // Alternating edge intervals, then reversal: substantial nonuniformity with
  // a known signed displacement, without assuming a steady hand-turn speed.
  std::vector<Event> uneven{{0, 0}};
  uint32_t us = 0;
  int32_t count = 0;
  for (int n = 0; n < 2000; ++n) {
    us += n % 2 ? 900 : 100;
    count += n < 1500 ? 1 : -1;
    uneven.push_back({us, count});
  }
  replay(uneven, false, 0);
  replay(uneven, true, UINT32_MAX - 50000);
  replay(uneven, true, UINT32_MAX - 50000, 2000);
  replay<RollingVelocity>(uneven,true,UINT32_MAX-50000,2000);
  replay<RollingVelocity>(uneven,true,UINT32_MAX-50000,4000);
  replay<RollingVelocity>(uneven,true,UINT32_MAX-50000,8000);

  WindowVelocity guard;
  guard.reset(0, 0);
  require(guard.update(0.1f, 10000) > 5, "overspeed is not hidden by clamping");
  require(!std::isfinite(guard.update(NAN, 20000)), "invalid angle propagates to protection");
  require(guard.update(0, 30000) == 0, "recover from invalid input without stale speed");
  guard.update(1, 40000);
  require(guard.update(100, 600000) == 0, "long initialization gap resets estimator");
  RollingVelocity rollingGuard;rollingGuard.reset(0,0);
  require(rollingGuard.update(.1f,10000)>5,"rolling slope propagates overspeed");
  require(!std::isfinite(rollingGuard.update(NAN,20000)),"rolling slope propagates invalid angle");
  require(rollingGuard.update(0,30000)==0,"rolling slope recovers without stale speed");
  RollingVelocity rolling;
  WindowVelocity window;window.periodUs=2000;
  rolling.reset(0,0);window.reset(0,0);
  double rollingError=0,windowError=0,area=0;unsigned measured=0;float previous=0,lastAngle=0;
  for (uint32_t sampleUs=250;sampleUs<=2300000;sampleUs+=250) {
    const double trueAngle=sampleUs<100000 ? 0 : sampleUs<2100000 ? (sampleUs-100000)*1e-6 : 2;
    lastAngle=float(std::floor(trueAngle/radiansPerCount)*radiansPerCount);
    area+=previous*.00025;
    previous=rolling.update(lastAngle,sampleUs);
    const float old=window.update(lastAngle,sampleUs);
    if (sampleUs>=400000 && sampleUs<2000000) {
      rollingError+=(previous-1)*(previous-1);windowError+=(old-1)*(old-1);++measured;
    }
  }
  const double rollingRms=std::sqrt(rollingError/measured),windowRms=std::sqrt(windowError/measured);
  std::cout<<"quantized_constant_speed rolling_RMS="<<rollingRms<<" window2ms_RMS="<<windowRms<<" integral_error="<<area-lastAngle<<'\n';
  require(rollingRms<windowRms*.8,"rolling regression reduces constant-speed quantization noise");
  require(std::abs(area-lastAngle)<.005,"rolling constant-speed integral agrees with count displacement");
  rolling.reset(0,0);window.reset(0,0);
  double rollingLag=0,windowLag=0;unsigned lagSamples=0;
  for (uint32_t sampleUs=250;sampleUs<=500000;sampleUs+=250) {
    const float t=sampleUs*1e-6f,angle=.5f*t*t;
    const float rv=rolling.update(angle,sampleUs),wv=window.update(angle,sampleUs);
    if (sampleUs>=50000) {rollingLag+=t-rv;windowLag+=t-wv;++lagSamples;}
  }
  std::cout<<"constant_acceleration mean_lag_s rolling="<<rollingLag/lagSamples<<" window2ms="<<windowLag/lagSamples<<'\n';
  require(rollingLag/lagSamples<=windowLag/lagSamples+.0005,"noise reduction does not add over .5ms mean estimator delay");
  RollingVelocity shortWindow,longWindow;longWindow.longWindow=true;
  require(shortWindow.horizonUs()==4000 && shortWindow.minimumSampleUs()==150 && shortWindow.warmupUs()==3000,"4ms configuration remains unchanged");
  require(longWindow.horizonUs()==8000 && longWindow.minimumSampleUs()==300 && longWindow.warmupUs()==6000,"8ms retains its horizon with the existing buffer");
  for (const float sign : {-1.f,1.f}) {
    shortWindow.reset(0,0);longWindow.reset(0,0);
    double smallError=0,largeError=0;unsigned samples=0;
    for (uint32_t trialUs=80;trialUs<=600000;trialUs+=80) {
      const float angle=sign*float(std::floor(trialUs*1e-6/radiansPerCount)*radiansPerCount);
      const float small=shortWindow.update(angle,trialUs),large=longWindow.update(angle,trialUs);
      if (trialUs>50000) {smallError+=(small-sign)*(small-sign);largeError+=(large-sign)*(large-sign);++samples;}
    }
    std::cout<<"quantized_speed sign="<<sign<<" RMS4ms="<<std::sqrt(smallError/samples)<<" RMS8ms="<<std::sqrt(largeError/samples)<<'\n';
    require(largeError<smallError*.64,"8ms reduces simulated count quantization RMS by at least 20% in both directions");
  }
  longWindow.reset(0,0);double longLag=0;unsigned longSamples=0;
  for (uint32_t trialUs=80;trialUs<=500000;trialUs+=80) {
    const float t=trialUs*1e-6f,v=longWindow.update(.5f*t*t,trialUs);
    if (trialUs>50000) {longLag+=t-v;++longSamples;}
  }
  require(longLag/longSamples>.0035 && longLag/longSamples<.0045,"8ms extra delay is measured, not treated as free smoothing");
  require(longWindow.update(10,1100000)==0,"8ms long gap clears stale estimate");
  require(!std::isfinite(longWindow.update(NAN,1200000)),"8ms invalid angle propagates");
  longWindow.reset(0,0);
  require(longWindow.update(.1f,10000)>5,"8ms estimator does not clip overspeed");
  std::cout << "PASS: saved edges, uneven timing, reverse, stop, clock wrap, overspeed, invalid input, long gap\n";
}
