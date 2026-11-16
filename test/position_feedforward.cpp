#include "PositionFeedforward.h"
#include <cstdlib>
#include <iostream>
void require(bool condition, const char* message) {
  if (!condition) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
int main() {
  PositionFeedforward f{-.0085f,.0253f,.03f};
  require(f.update(.3f,1,2000000)==0,"no Index means no correction");
  f.observeIndex(.1f,0); require(f.update(.3f,1,2000000)==0,"one Index is insufficient");
  f.observeIndex(.1f+6.28318530718f,0);
  require(f.armed && !f.invalid && f.update(.3f,1,800000)==0,"two consistent Index observations; startup gate");
  const float half=f.update(.3f,1,900000),full=f.update(.3f,1,1000000);
  require(std::abs(2*half-full)<1e-6f,"200ms blend");
  for (int n=-2000;n<=2000;++n) {
    f.update(n*.002f,1,1500000);
    require(!f.invalid && std::abs(f.correction)<.0268f,"fitted small correction amplitude");
    require(std::abs(f.mix(.44f,.45f))<=.450001f && std::abs(f.mix(-.44f,.45f))<=.450001f,"total current request remains bounded");
  }
  const float positive=f.update(.4f,1,1500000),negative=f.update(-.2f,-1,1500000);
  require(std::abs(positive-negative)<1e-6f,"direction-corrected Index-relative phase");
  f.observeIndex(.12f,1600000); require(f.invalid && f.mix(.2f,.45f)==0,"inconsistent later Index latches fault");
  f.reset(); f.observeIndex(.1f,1200000); f.observeIndex(.1f,1200000);
  require(f.update(.3f,1,1200000)==0 && std::abs(f.update(.3f,1,1400000))>0,"late Index acquisition blends from zero");
  f.update(NAN,1,1500000); require(f.invalid && f.mix(.2f,.45f)==0,"invalid angle cannot energize");
  PositionFeedforward capped{1,1,.03f}; capped.observeIndex(0,0);capped.observeIndex(0,0);
  require(std::abs(capped.update(0,1,2000000))<=.03f,"amplitude ceiling independent of coefficients");
  PositionFeedforward bad{NAN,.1f,.03f}; require(bad.invalid && bad.mix(.2f,.45f)==0,"invalid calibration faults");
  PositionFeedforward boundary{0,.02f,.03f};boundary.observeIndex(.1f,0);boundary.observeIndex(.1f+2*6.28318530718f/4000,0);
  require(boundary.armed && !boundary.invalid,"two-count boundary tolerates float rounding");
  boundary.observeIndex(.1f+3*6.28318530718f/4000,0);require(boundary.invalid,"three-count discrepancy is rejected");
  PositionFeedforward limit{0,.02f,.03f};require(limit.mix(.2f,0)==0 && limit.invalid,"invalid total limit faults");
  PositionFeedforward direction{0,.02f,.03f};direction.update(0,0,0);require(direction.invalid,"invalid direction faults");
  std::cout << "PASS: Index gating/consistency, 84th-order phase, blend, correction/total bounds, direction, invalid input latch\n";
}
