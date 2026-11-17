#include "ReferenceSpeedGuard.h"
#include "VelocityReference.h"
#include <cassert>
#include <iostream>

int main() {
  using G=ReferenceSpeedGuard;
  for (float sign : {-1.f,1.f}) {
    assert(G::evaluate(sign*5.039947f,sign*3,sign*G::maximumGoal).outside);
    assert(G::evaluate(sign*3,sign*3,sign*G::maximumGoal).upper==5);
    for(uint32_t rampUs : {800000u,2000000u}) for(uint32_t us=0;us<=3000000;us+=997) {
      const float reference=velocityReference(sign*G::maximumGoal,us,rampUs);
      const auto good=G::evaluate(reference,reference,sign*G::maximumGoal);
      assert(good.valid && !good.outside && good.upper<=G::hardCeiling);
      assert(G::evaluate(sign*(good.upper+.01f),reference,sign*G::maximumGoal).outside);
      assert(G::evaluate(sign*(good.lower-.01f),reference,sign*G::maximumGoal).outside);
    }
    assert(G::evaluate(sign*9,sign*G::maximumGoal,sign*G::maximumGoal).outside);
    assert(G::evaluate(-sign*3,0,sign*G::maximumGoal).outside);
  }
  for(float bad : {NAN,INFINITY}) {
    assert(!G::evaluate(bad,1,6).valid);
    assert(!G::evaluate(1,bad,6).valid);
    assert(!G::evaluate(1,1,bad).valid);
  }
  assert(!G::evaluate(1,1,0).valid && !G::evaluate(1,1,7).valid);
  assert(!G::evaluate(1,4,3).valid && !G::evaluate(1,-1,6).valid);
  std::cout<<"PASS: independent reference envelope, unchanged upper threshold at3, bounded higher range, both directions, under/overspeed, hard cap and invalid inputs\n";
}
