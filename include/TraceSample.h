#pragma once
#include <cstdint>
struct TraceSample {
  uint32_t elapsedUs;
  uint16_t loopUs,windowAgeUs;
  // Joint tests save commanded position/velocity instead of redundant A/B edges.
  // Current payloads below must remain available in every current-control mode.
  union {uint32_t a;float jointTarget;};
  union {uint32_t b;float jointVelocityTarget;};
  float angle;
  union {float rawVelocity;float iqFiltered;};
  float filteredVelocity,windowVelocity;
  union {float target;float iqTarget;};
  float uq;
  union {float openFieldAngle;float ud;};
  union {float unslewedUq;float idFiltered;};
  uint32_t adc1Frames,adc2Frames;
  uint16_t adcAge,iaAdc,ibAdc,icAdc,vbusAdc,temperatureAdc;
};
static_assert(sizeof(TraceSample)==68,"Complete 3s trace must fit SRAM");
