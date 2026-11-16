#pragma once
#include <cmath>
#include <cstdint>

// One additional electrical turn is reserved for the Index-coverage trial.
// The ordinary three-turn alignment and its pole-pair tolerance stay intact.
class AlignmentPlan {
 public:
  static constexpr int normalCycles=3;
  const int cycles;
  const uint32_t deadlineUs;
  explicit constexpr AlignmentPlan(bool coverage=false):cycles(coverage ? 4 : normalCycles),deadlineUs(coverage ? 20000000u : 0u) {}
  constexpr int steps() const { return 500*cycles; }
  bool expired(uint32_t now,uint32_t start) const { return deadlineUs && now-start>=deadlineUs; }
  bool passes(float forward,float reverse,int polePairs) const {
    constexpr float tau=6.28318530718f;
    return std::isfinite(forward) && std::isfinite(reverse) && polePairs>0 && forward*reverse>0 &&
      std::abs(std::abs(forward)*polePairs/cycles-tau)<=.5f &&
      std::abs(std::abs(reverse)*polePairs/cycles-tau)<=.5f;
  }
};
