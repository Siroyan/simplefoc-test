#pragma once
#include <BoardCurrentObserver.h>

// Uses the same protected ADC snapshot as the trace and ObservationGuard.
// Hardware initialization and zero calibration remain owned by the observer.
class ObserverCurrentSense : public CurrentSense {
 public:
  ObserverCurrentSense(BoardCurrentObserver& source, const BoardCurrentObserver::Frame& snapshot)
      : observer(source), frame(snapshot) {
    // Sign/order independently checked with six static vectors before enabling.
    skip_align = true;
    gain_a = gain_b = gain_c = 1;
    offset_ia = offset_ib = offset_ic = 0;
    pinA = A_OP1_OUT; pinB = A_OP2_OUT; pinC = A_OP3_OUT;
  }
  int init() override {
    initialized = observer.ready && driver && driver_type == DriverType::BLDC;
    return initialized ? 1 : 0;
  }
  PhaseCurrent_s getPhaseCurrents() override {
    return {observer.current(frame.a, 0), observer.current(frame.b, 1), observer.current(frame.c, 2)};
  }
 private:
  BoardCurrentObserver& observer;
  const BoardCurrentObserver::Frame& frame;
};
