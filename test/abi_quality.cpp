#include "AbiQuality.h"
#include <cstdlib>
#include <iostream>
void require(bool condition, const char* message) {
  if (!condition) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
int main() {
  require(AbiQuality::stateFromPort(0)==0 && AbiQuality::stateFromPort(1U<<6)==2 &&
          AbiQuality::stateFromPort(1U<<7)==1 && AbiQuality::stateFromPort(~0U)==3,
          "PB6/PB7 single-port state mapping");
  AbiQuality q;
  const uint8_t cycle[4]={0,2,3,1};
  for (int offset=0;offset<4;++offset) {
    q.reset(cycle[offset]);
    require(q.currentState()==cycle[offset],"capture origin uses last processed ISR state");
    for (int n=1;n<=4000;++n) q.edge(cycle[(offset+n)%4]);
    require(q.count==4000 && q.invalid==0 && q.duplicate==0 && q.reversals==0,"positive turn from every phase");
    for (int n=1;n<=4000;++n) q.edge(cycle[(offset+4000-n)%4]);
    require(q.count==0 && q.callbacks==8000 && q.invalid==0 && q.duplicate==0 && q.reversals==1,
            "reverse turn, exact net count and one direction change");
  }
  for (uint8_t s=0;s<4;++s) {
    q.reset(s); q.edge(s^3);
    require(q.invalid==1 && q.count==0 && q.currentState()==(s^3),"two-bit changes flagged without inventing motion");
    q.edge(s^3); require(q.duplicate==1 && q.count==0,"duplicate callback flagged");
  }
  std::cout << "PASS: atomic port mapping, all valid transitions, 4000-count turns/reversal, invalid and duplicate transitions\n";
}
