#pragma once
#include <cstdint>

// Only for the <=3s active experiment. At 170MHz, the uint32 cycle difference
// remains unambiguous for about 25s, including a CYCCNT wrap during the run.
struct ShortRunClock {
  static constexpr uint32_t cyclesPerUs=170;
  static uint32_t elapsedUs(uint32_t origin,uint32_t cycles) {
    return (cycles-origin)/cyclesPerUs;
  }
};
