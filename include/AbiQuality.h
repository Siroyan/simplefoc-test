#pragma once
#include <cstdint>

// Small, always-on observer. It never changes the controller's encoder count.
class AbiQuality {
 public:
  volatile int32_t count = 0;
  volatile uint32_t callbacks = 0, invalid = 0, duplicate = 0, reversals = 0;
  static uint8_t stateFromPort(uint32_t portB) {
    return uint8_t(((portB >> 5) & 2) | ((portB >> 7) & 1)); // A=PB6, B=PB7.
  }
  void reset(uint8_t state) {
    previous = state & 3; direction = 0;
    count = 0; callbacks = invalid = duplicate = reversals = 0;
  }
  void edge(uint8_t state) {
    state &= 3; ++callbacks;
    static constexpr int8_t steps[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
    const uint8_t changed = previous ^ state;
    if (changed == 3) ++invalid;
    else if (!changed) ++duplicate;
    else {
      const int8_t step = steps[previous * 4 + state];
      count += step;
      if (direction && direction != step) ++reversals;
      direction = step;
    }
    previous = state;
  }
 private:
  uint8_t previous = 0;
  int8_t direction = 0;
};
