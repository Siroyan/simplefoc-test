#include "AlignmentPlan.h"
#include <cstdlib>
#include <iostream>
void require(bool condition,const char* message) {
  if (!condition) {std::cerr<<"FAIL: "<<message<<'\n';std::exit(1);}
}
int main() {
  constexpr float tau=6.28318530718f;
  AlignmentPlan normal,coverage{true};
  require(normal.cycles==3 && normal.steps()==1500 && normal.deadlineUs==0,"ordinary alignment preserved");
  require(coverage.cycles==4 && coverage.steps()==2000 && coverage.deadlineUs==20000000,"coverage bounded to four turns and twenty seconds");
  for (const auto& plan : {normal,coverage}) {
    const float angle=plan.cycles*tau/7;
    require(plan.passes(angle,angle,7) && plan.passes(-angle,-angle,7),"both encoder directions retain pole-pair check");
    require(!plan.passes(angle,-angle,7),"inconsistent return direction rejected");
    require(!plan.passes(angle,angle,6),"wrong pole-pair mapping rejected");
    require(plan.passes(angle+.49f*plan.cycles/7,angle,7),"existing tolerance retained");
    require(!plan.passes(angle+.51f*plan.cycles/7,angle,7),"outside tolerance rejected");
    require(!plan.passes(NAN,angle,7) && !plan.passes(angle,INFINITY,7),"invalid observations refused");
  }
  const uint32_t began=UINT32_MAX-10000000;
  require(!coverage.expired(began+19999999u,began),"coverage waits only within deadline across wrap");
  require(coverage.expired(began+20000000u,began),"coverage deadline holds across clock wrap");
  require(!normal.expired(began+20000000u,began),"ordinary profile has no new deadline behavior");
  require(coverage.passes(3.57f,3.56f,7),"measured-sized four-turn displacement accepted with four-turn denominator");
  require(!normal.passes(3.57f,3.56f,7),"four-turn result cannot be mistaken for a three-turn result");
  std::cout<<"PASS: alignment cycle counts, unchanged pole-pair tolerance, finite inputs, timed coverage and clock wrap\n";
}
