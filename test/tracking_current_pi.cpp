#include "TrackingCurrentPI.h"
#include "MagnitudeSlew.h"
#include "VoltageVectorBound.h"
#include <cstdlib>
#include <iostream>
void require(bool condition, const char* message) {
  if (!condition) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
int main() {
  TrackingCurrentPI pi;
  MagnitudeSlew slew;
  pi.reset(0); slew.reset(0);
  float current = 0, voltage = 0;
  float reversePeak = 0;
  // Deliberately request more voltage than the ramp and ceiling admit, then
  // reverse. The integrator must not keep demanding positive voltage.
  for (uint32_t now = 100; now <= 400000; now += 100) {
    const float error = now < 200000 ? 1.0f : -1.0f;
    const float request = pi.request(error, now);
    VoltageVectorBound bound;
    bound.apply(request, 0, 0, .5f);
    const float next = slew.update(bound.q, now);
    pi.track(next);
    require(!pi.invalid && std::abs(next) <= .500001f, "finite bounded actuation");
    require(std::abs(next) <= std::abs(voltage) + .000301f, "3V/s build-up retained");
    if (now == 200000) reversePeak = next;
    voltage = next;
  }
  require(reversePeak < .11f, "reversal releases accumulated forward voltage immediately");
  require(voltage < -.49f, "opposite voltage can build after zero crossing");
  // Simple resistive/inductive plant, including the actual slew/vector pipeline.
  pi.reset(0); slew.reset(0); voltage = current = 0;
  for (uint32_t now = 100; now <= 900000; now += 100) {
    const float target = now < 500000 ? .15f : -.15f;
    VoltageVectorBound bound;
    bound.apply(pi.request(target - current, now), 0, 0, .5f);
    voltage = slew.update(bound.q, now);
    pi.track(voltage);
    current += .05f * (voltage / .5f - current); // R=.5, time constant 2ms.
    if (now == 400000 || now == 900000)
      require(std::abs(target - current) < .001f, "steady current tracking in both directions");
  }
  pi.reset(0xfffffff0U);
  require(std::isfinite(pi.request(.1f, 40)), "clock wrap");
  pi.track(.001f);
  pi.request(.1f, 600041);
  require(pi.invalid && pi.request(.1f, 600042) == 0, "long gap faults and latches");
  for (float value : {NAN, INFINITY, -INFINITY}) {
    pi.reset(0); require(pi.request(value, 100) == 0 && pi.invalid, "invalid measurement rejected");
    pi.reset(0); pi.request(.1f, 100); pi.track(value);
    require(pi.invalid, "invalid actuator feedback rejected");
  }
  pi.reset(0); pi.track(.6f); require(pi.invalid, "out of bound feedback rejected");
  MagnitudeSlew brake{3,12}; brake.reset(0);
  require(std::abs(brake.update(.5f,10000)-.03f)<1e-6f,"forward voltage still builds at 3V/s");
  require(brake.update(-.5f,20000)==0,"fast brake still crosses zero");
  require(std::abs(brake.update(-.5f,30000)+.12f)<1e-6f,"negative build-up at 12V/s");
  require(std::abs(brake.update(-.02f,40000)+.02f)<1e-6f,"braking releases immediately");
  for (float proportional : {.20f,.40f,.60f,.80f}) for (float gain : {50.0f,100.0f,200.0f}) for (float rate : {3.0f,12.0f,30.0f,100.0f}) {
    MagnitudeSlew actuator{rate,rate >= 30.0f ? rate : 12.0f}; actuator.reset(0); pi.reset(0); pi.P=proportional; pi.I=gain; current=0;
    float filtered=0;
    for (uint32_t now=100;now<=900000;now+=100) {
      const float target=now<500000 ? .45f : -.45f;
      filtered+=(current-filtered)/21.0f; // Actual 2ms current LPF at 100us updates.
      VoltageVectorBound bound; bound.apply(pi.request(target-filtered,now),0,0,.5f);
      const float applied=actuator.update(bound.q,now); pi.track(applied);
      current+=.05f*(applied/.5f-current);
      require(!pi.invalid && std::abs(applied)<=.500001f,"current plant remains bounded for compared rise rates");
      if (now==400000 || now==900000)
        require(std::abs(target-current)<.001f,"compared gains/rise rates track signed .45A in native plant");
    }
  }
  std::cout << "PASS: ramp-limited reversal antiwindup, bounded voltage, bidirectional plant tracking, clock wrap, invalid inputs and latch\n";
}
