#pragma once
#include <cmath>
#include <cstdint>

// Passive diagnostic only; this tracker never changes a control or stop limit.
struct GuardPeakState {
  uint32_t elapsedUs=0, sampleAgeUs=0, adcAgeUs=0;
  float guard=0, control=0, reference=0;
  float iqApplied=0, iqNext=0, iqFiltered=0, uq=0, ud=0, correction=0;
};

class GuardPeakTracker {
 public:
  GuardPeakState whole{}, steady{};
  bool wholeValid=false, steadyValid=false;
  uint32_t steadyStartUs=800000;
  void reset(uint32_t startUs=800000) {
    whole={};steady={};wholeValid=steadyValid=false;steadyStartUs=startUs;
  }
  bool wants(float guard,uint32_t elapsedUs) const {
    return std::isfinite(guard) &&
      (!wholeValid || std::abs(guard)>std::abs(whole.guard) ||
       (elapsedUs>=steadyStartUs && (!steadyValid || std::abs(guard)>std::abs(steady.guard))));
  }
  void observe(const GuardPeakState& state) {
    if (!std::isfinite(state.guard)) return;
    if (!wholeValid || std::abs(state.guard)>std::abs(whole.guard)) {
      whole=state;wholeValid=true;
    }
    if (state.elapsedUs>=steadyStartUs && (!steadyValid || std::abs(state.guard)>std::abs(steady.guard))) {
      steady=state;steadyValid=true;
    }
  }
};
static_assert(sizeof(GuardPeakTracker)<=112,"Peak diagnostics must retain the existing trace capacity");
