#include "IntervalVelocityStats.h"
#include <cassert>
#include <iostream>

int main() {
  for(float sign : {-1.f,1.f}) {
    IntervalVelocityStats stats;
    const uint32_t base=UINT32_MAX-1000000;
    stats.reset(base,sign*3,2000000);
    for(uint32_t us=0;us<=3000000;us+=400) stats.update(base+us,sign*3.2f*us*1e-6f);
    assert(!stats.invalid && stats.samples==500 && stats.coveredUs==1000000);
    assert(std::abs(stats.mean()-sign*3.2f)<1e-5f && std::abs(stats.rmsError()-.2f)<.001f);
    assert(stats.slowUs==0 && stats.reverseUs==0 && stats.firstUs-base==2000000);
    stats.reset(0,sign*3,0);stats.update(0,0);stats.update(2000,-sign*.001f);
    assert(stats.reverseUs==2000);
    stats.update(4000,-sign*.001f);assert(stats.slowUs==2000);
    stats.update(25000,0);assert(stats.invalid);
    stats.reset(0,sign*3,0);stats.update(0,NAN);assert(stats.invalid);
  }
  IntervalVelocityStats bad;bad.reset(0,NAN,0);assert(bad.invalid);
  bad.reset(0,1,3000000);assert(bad.invalid);
  bad.reset(0,1,0);bad.update(0,0);bad.update(2000,1e30f);assert(bad.invalid);
  std::cout<<"PASS: full post-ramp interval, known .2rad/s error, mean/displacement, both signs, reversal/slow duration, wrap, gaps and invalid inputs\n";
}
