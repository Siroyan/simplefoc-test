#include "GuardPeak.h"
#include <cassert>
#include <iostream>

int main() {
  GuardPeakTracker tracker;
  GuardPeakState first{100,50,70,3,2,2.5f,.2f,-.1f,.18f,.1f,.01f,.02f};
  assert(tracker.wants(3,100));tracker.observe(first);
  assert(tracker.wholeValid && !tracker.steadyValid && tracker.whole.iqApplied==.2f && tracker.whole.iqNext==-.1f);
  auto later=first;later.elapsedUs=800000;later.guard=2;
  assert(tracker.wants(2,800000));tracker.observe(later);
  assert(tracker.whole.elapsedUs==100 && tracker.steady.elapsedUs==800000);
  later.elapsedUs=900000;later.guard=-4;later.iqFiltered=-.2f;
  tracker.observe(later);assert(tracker.whole.guard==-4 && tracker.steady.iqFiltered==-.2f);
  later.elapsedUs=950000;tracker.observe(later);
  assert(!tracker.wants(-4,950000) && tracker.whole.elapsedUs==900000); // First maximum wins ties.
  later.guard=NAN;tracker.observe(later);assert(tracker.whole.guard==-4 && !tracker.wants(INFINITY,0));
  later.guard=4.2f;later.iqNext=NAN;tracker.observe(later);
  assert(std::isnan(tracker.whole.iqNext)); // Preserve faulty payload for diagnosis; do not hide the velocity peak.
  tracker.reset();assert(!tracker.wholeValid && !tracker.steadyValid);
  const uint32_t start=UINT32_MAX-100;
  later.elapsedUs=static_cast<uint32_t>(50-start);later.guard=1;tracker.observe(later);
  assert(tracker.whole.elapsedUs==151 && !tracker.steadyValid);
  std::cout<<"PASS: signed maxima, startup/steady boundary, first tie, reset, invalid guard and forensic payload, elapsed clock wrap\n";
}
