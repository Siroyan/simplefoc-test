#include <Arduino.h>
#include <SimpleFOC.h>
#include <ManualDiagnostics.h>
#include <WindowVelocity.h>
#include <MotionWatchdog.h>
#include <MagnitudeSlew.h>
#include <BoardCurrentObserver.h>
#include <ObservationGuard.h>
#include <PhasePulseProfile.h>
#include <BoundedBLDCMotor.h>
#include <ObserverCurrentSense.h>
#include <AbiQuality.h>
#include <PositionFeedforward.h>
#include <RollingVelocity.h>
#include <CurrentProbe.h>
#include <AlignmentPlan.h>
#include <AdaptiveHarmonic.h>

constexpr int kDiagVersion = 84;
enum class CurrentTrial { none, fixed015, fixed025, velocity, velocityCompensated, velocityRolling, velocityProbe, staticProbe, staticProbe10, velocityFastIntegral, velocityReverse, velocityCoverage, velocityLongWindow, velocityLowerFilter, velocityDense, velocityDenseLowerP, velocityHigherI, velocityHigherIReverse, velocityStrongerI, velocityStrongerIReverse, velocityHigherIDense, velocityStrongerIDense, velocityAdaptive, velocityAdaptiveZero, velocityResponseProbe, velocityResponseProbeInverse, velocityAdaptiveFlat };
constexpr int kEncoderPpr = 1000; // Two measured index intervals: 4000 signed counts each.
constexpr int kMotorPolePairs = 7; // Motor datasheet: 12N14P.
constexpr float kBusVoltage = 11.1f;
constexpr float kVoltageLimit = 0.5f;
constexpr float kAlignVoltage = 0.7f;
constexpr float kTargetVelocity = 1.0f; // rad/s, approximately 9.55 rpm.
constexpr float kOpenTargetVelocity = 1.0f; // Compare open-loop at the same nominal speed as closed-loop.
constexpr float kOpenVoltageLimit = 0.5f; // Match closed-loop voltage ceiling; alignment remains 0.7V.
constexpr float kVelocityOutputRamp = 3.0f; // Best tested closed-loop profile; symmetric V/s slew limit.
constexpr uint32_t kRunMs = 3000;
constexpr int kAlignCycles = AlignmentPlan::normalCycles;
constexpr int kAlignmentSampleStride = 10;
static_assert(2 * (500 * kAlignCycles / kAlignmentSampleStride + 1) + 3 <= 320,
              "Alignment samples must fit the existing run buffer");
#ifndef DIAG_PWM_HZ
#define DIAG_PWM_HZ 25000
#endif
constexpr long kPwmFrequency = DIAG_PWM_HZ;
static_assert(kPwmFrequency == 25000 || kPwmFrequency == 12500, "Only compared PWM profiles are supported");
// Keep requested dead time at 800ns when changing frequency, not a constant fraction.
constexpr float kPwmDeadZone = 0.02f * (kPwmFrequency / 25000.0f);
// Observe the existing velocity call without advancing its estimator twice.
class DiagnosticEncoder : public Encoder {
 public:
  using Encoder::Encoder;
  float rawVelocity = 0;
  float legacyFilteredVelocity = 0;
  float correctedVelocity = 0;
  WindowVelocity window;
  RollingVelocity rolling;
  bool useRolling = false;
  float guardVelocity = 0;
  LowPassFilter guardFilter{.002f};
  LowPassFilter legacyFilter{0.05f};
  float getVelocity() override {
    rawVelocity = Encoder::getVelocity();
    legacyFilteredVelocity = legacyFilter(rawVelocity); // Diagnostic shadow only.
    const float angle = getAngle(); const uint32_t now = micros();
    const float windowValue = window.update(angle, now);
    if (!useRolling) return correctedVelocity = windowValue;
    guardVelocity = guardFilter(windowValue); // Original 2ms-window/2ms-LPF overspeed channel.
    return correctedVelocity = rolling.update(angle, now);
  }
};
DiagnosticEncoder encoder(PB6, PB7, kEncoderPpr); // No index search; align each boot.
BLDCDriver6PWM driver(PA8, PC13, PA9, PA12, PA10, PB15);
BoundedBLDCMotor motor(kMotorPolePairs);
volatile uint32_t edgesA = 0, edgesB = 0, indexPulses = 0;
struct IndexReference { uint32_t bootUs, pulse; float angle; };
volatile IndexReference indexReferences[8]{};
volatile uint8_t indexReferenceCount = 0;
AbiQuality abiQuality;
struct AbiSnapshot { int32_t count; uint32_t callbacks, invalid, duplicate, reversals; float encoderAngle; };
AbiSnapshot runAbiStart{}, runAbiEnd{};
bool ready = false, attempted = false, running = false;
bool openTest = false;
bool phasePulseTest = false;
bool currentIqTest = false;
bool currentVelocityTest = false;
bool currentFeedforwardTest = false;
bool currentProbeTest = false;
bool currentResponseProbeTest=false,currentResponseProbeInverseTest=false;
bool currentStaticProbeTest = false;
bool currentStaticProbe10Test = false;
bool currentFastIntegralTest = false;
bool currentReverseTest = false;
bool currentCoverageTest = false;
bool currentAdaptiveTest=false,currentAdaptiveZeroTest=false,currentAdaptiveFlatTest=false;
AdaptiveHarmonic adaptiveHarmonic,adaptiveHarmonicZero{0},adaptiveHarmonicFlat{.2f,.03f,0};
AdaptiveHarmonic& activeAdaptive() {return currentAdaptiveZeroTest ? adaptiveHarmonicZero : currentAdaptiveFlatTest ? adaptiveHarmonicFlat : adaptiveHarmonic;}
const char* alignmentFailure=nullptr;
float currentVelocityGoal=kTargetVelocity;
CurrentProbe currentProbe{20,.02f};
CurrentProbe responseCurrentProbe{17.5f,.02f};
CurrentProbe responseCurrentProbeInverse{17.5f,.02f,800000,200000,3.14159265f};
CurrentProbe staticCurrentProbe{20,.02f,400000,100000};
CurrentProbe staticCurrentProbe10{10,.02f,400000,100000};
CurrentProbe& activeCurrentProbe() { return currentResponseProbeTest ? (currentResponseProbeInverseTest ? responseCurrentProbeInverse : responseCurrentProbe) : currentStaticProbe10Test ? staticCurrentProbe10 : currentStaticProbeTest ? staticCurrentProbe : currentProbe; }
// Half the mean 84th-order equivalent-current coefficients independently fitted
// from window-age-02/04. Experimental hypothesis only; no absolute torque claim.
PositionFeedforward positionFeedforward{-.0085f, .0253f, .03f};
struct FeedforwardSample { uint32_t elapsedUs; float pid, correction, total; };
FeedforwardSample feedforwardTrace[16]{};
uint8_t feedforwardCount = 0;
uint32_t lastFeedforwardUs = 0;
bool alignmentOnlyTest = false;
uint16_t alignmentCount = 0;
uint32_t alignmentStartUs = 0;
float fixedIqGoal = 0;
bool asymmetricTest = false;
bool forwardOnlyTest = false;
float fixedUqTest = 0; // Nonzero only for the fixed-voltage, sensor-commutated diagnostic.
MagnitudeSlew magnitudeSlew{3.0f}; // Retain the tested low-reversal comparison profile.
float unslewedPidUq = 0;
float measuredVelocity = 0;
float lastGuardVelocity = 0, runGuardAbsPeak = 0;
LowPassFilter observedVelocity(0.05f);
uint32_t runStart = 0;
float startAngle = 0;
constexpr uint32_t kTracePeriodUs = 10000;
uint32_t tracePeriodUs = kTracePeriodUs;
constexpr uint16_t kTraceCapacity = 320; // Entire 3s run, dumped only after disabling output.
struct TraceSample {
  uint32_t elapsedUs;
  uint16_t loopUs, windowAgeUs; // Saturation at 65535 is explicit in TRACE_CONFIG.
  uint32_t a, b;
  float angle;
  // Mode-specific payloads share RAM; serialization uses explicit units and
  // emits zero for unavailable legacy fields in the current-control test.
  union { float rawVelocity; float iqFiltered; };
  float filteredVelocity, windowVelocity;
  union { float target; float iqTarget; };
  float uq;
  union { float openFieldAngle; float ud; };
  union { float unslewedUq; float idFiltered; };
  uint32_t adc1Frames, adc2Frames;
  uint16_t adcAge, iaAdc, ibAdc, icAdc, vbusAdc, temperatureAdc;
};
static_assert(sizeof(TraceSample) == 68, "Keep complete 3s trace within SRAM budget");
TraceSample trace[kTraceCapacity];
uint16_t traceNext = 0, traceCount = 0;
uint32_t traceStartUs = 0, lastTraceUs = 0, lastLoopUs = 0, loopUs = 0;
uint32_t windowStartUs = 0;
float windowAngle = 0, windowVelocity = 0, commandedVelocity = 0;
bool tracePending = false;
MotionWatchdog motionWatchdog;
BoardCurrentObserver currentObserver;
bool observeCurrents = false;
BoardCurrentObserver::Frame observedFrame{};
ObserverCurrentSense observerSense(currentObserver, observedFrame);
struct CurrentControlTrace { float iq, id, ud, target; } currentControlTrace{};
ObservationGuard observationGuard;
bool senseIdle = false;
bool senseAttempted = false;
uint32_t senseLastReport = 0;
ManualDiagnostics manual;
bool manualLocked = false;
uint32_t manualLastReport = 0, manualWindowUs = 0;
float manualWindowAngle = 0, manualWindowVelocity = 0;
uint32_t manualIntegralUs = 0;
float manualStartAngle = 0;
double manualCorrectedIntegral = 0, manualFilteredIntegral = 0, manualLegacyIntegral = 0;
float previousCorrected = 0, previousFiltered = 0, previousLegacy = 0;

AbiSnapshot readAbiSnapshot() {
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  const AbiSnapshot result{abiQuality.count, abiQuality.callbacks, abiQuality.invalid,
      abiQuality.duplicate, abiQuality.reversals, encoder.getSensorAngle()};
  __set_PRIMASK(mask); return result;
}

void beginTrace() {
  traceNext = traceCount = 0;
  traceStartUs = lastTraceUs = lastLoopUs = windowStartUs = micros();
  windowAngle = encoder.getAngle();
  windowVelocity = commandedVelocity = 0;
  encoder.window.reset(encoder.getAngle(), micros());
  encoder.rolling.reset(encoder.getAngle(), micros());
  if (currentAdaptiveTest) activeAdaptive().reset(static_cast<int>(motor.sensor_direction)*encoder.getAngle(),micros());
  encoder.guardVelocity = lastGuardVelocity = runGuardAbsPeak = 0;
  motionWatchdog.reset(encoder.getAngle(), micros());
  magnitudeSlew.reset(micros());
  unslewedPidUq = 0;
  currentControlTrace = {};
  if (currentVelocityTest || currentStaticProbeTest) motor.actuatorStats.reset(micros(),currentStaticProbeTest ? 500000 : 800000);
  runAbiStart = readAbiSnapshot();
  loopUs = 0;
}

void recordTrace(bool force, float uq) {
  const uint32_t now = micros();
  if (!force && now - lastTraceUs < tracePeriodUs) return;
  lastTraceUs = now;
  noInterrupts();
  const uint32_t a = edgesA, b = edgesB;
  interrupts();
  const uint32_t adcAge = observeCurrents ? micros() - observedFrame.us : 0;
  const uint32_t windowAge = now - windowStartUs;
  trace[traceNext] = {now - traceStartUs,
      static_cast<uint16_t>(loopUs > 65535 ? 65535 : loopUs),
      static_cast<uint16_t>(windowAge > 65535 ? 65535 : windowAge), a, b,
      encoder.getAngle(), {encoder.rawVelocity}, measuredVelocity,
      windowVelocity, {commandedVelocity}, uq, {openTest ? motor.shaft_angle : 0.0f},
      {asymmetricTest ? unslewedPidUq : 0.0f},
      observeCurrents ? observedFrame.adc1Frames : 0, observeCurrents ? observedFrame.adc2Frames : 0,
      static_cast<uint16_t>(adcAge > 65535 ? 65535 : adcAge),
      static_cast<uint16_t>(observeCurrents ? observedFrame.a : 0), static_cast<uint16_t>(observeCurrents ? observedFrame.b : 0),
      static_cast<uint16_t>(observeCurrents ? observedFrame.c : 0), static_cast<uint16_t>(observeCurrents ? observedFrame.vbus : 0),
      static_cast<uint16_t>(observeCurrents ? observedFrame.temperature : 0)};
  if (currentIqTest) {
    trace[traceNext].iqFiltered = currentControlTrace.iq;
    trace[traceNext].idFiltered = currentControlTrace.id;
    trace[traceNext].ud = currentControlTrace.ud;
    trace[traceNext].iqTarget = currentControlTrace.target;
  }
  traceNext = (traceNext + 1) % kTraceCapacity;
  if (traceCount < kTraceCapacity) ++traceCount;
}

void reportIndexReferences() {
  const auto quality = readAbiSnapshot();
  Serial.print("ABI_QUALITY callbacks="); Serial.print(quality.callbacks);
  Serial.print(" invalid="); Serial.print(quality.invalid);
  Serial.print(" duplicate="); Serial.print(quality.duplicate);
  Serial.print(" net_signed_count="); Serial.println(quality.count);
  if (traceCount && !alignmentOnlyTest) {
    Serial.print("ABI_QUALITY_RUN callbacks="); Serial.print(runAbiEnd.callbacks-runAbiStart.callbacks);
    Serial.print(" invalid="); Serial.print(runAbiEnd.invalid-runAbiStart.invalid);
    Serial.print(" duplicate="); Serial.print(runAbiEnd.duplicate-runAbiStart.duplicate);
    Serial.print(" reversals="); Serial.print(runAbiEnd.reversals-runAbiStart.reversals);
    Serial.print(" count_delta="); Serial.print(runAbiEnd.count-runAbiStart.count);
    Serial.print(" encoder_count_delta="); Serial.println((runAbiEnd.encoderAngle-runAbiStart.encoderAngle)*4000/_2PI, 3);
  }
  const uint8_t count = indexReferenceCount; // ISR publishes count after writing each immutable slot.
  if (!count) Serial.println("INDEX_REFERENCE none_observed=1");
  for (uint8_t n = 0; n < count; ++n) {
    Serial.print("INDEX_REFERENCE pulse="); Serial.print(indexReferences[n].pulse);
    Serial.print(" boot_us="); Serial.print(indexReferences[n].bootUs);
    Serial.print(" encoder_angle_rad="); Serial.println(indexReferences[n].angle, 6);
  }
}

void dumpTrace() {
  tracePending = false;
  Serial.println("TRACE_BEGIN: outputs disabled; velocities use encoder sign except closed-loop filtered velocity.");
  Serial.println("trace_us,loop_us,A_edges,B_edges,angle_rad,raw_rad_s,filtered_rad_s,window10ms_rad_s,target_rad_s,Uq_V,open_field_mechanical_rad,unslewed_pid_uq_V,adc1_frames,adc2_frames,adc_age_us,ia_adc,ib_adc,ic_adc,vbus_adc,temp_adc,Ud_V,iq_target_nominal_A,iq_filtered_nominal_A,id_filtered_nominal_A,window_age_us");
  for (uint16_t n = 0; n < traceCount; ++n) {
    const TraceSample& row = trace[(traceNext + kTraceCapacity - traceCount + n) % kTraceCapacity];
    Serial.print(row.elapsedUs); Serial.print(',');
    Serial.print(row.loopUs); Serial.print(',');
    Serial.print(row.a); Serial.print(','); Serial.print(row.b); Serial.print(',');
    Serial.print(row.angle, 6); Serial.print(',');
    Serial.print(currentIqTest ? 0.0f : row.rawVelocity, 6); Serial.print(',');
    Serial.print(row.filteredVelocity, 6); Serial.print(',');
    Serial.print(row.windowVelocity, 6); Serial.print(',');
    Serial.print(currentIqTest ? (currentVelocityTest ? currentVelocityGoal : 0.0f) : row.target, 6); Serial.print(','); Serial.print(row.uq, 4); Serial.print(',');
    Serial.print(currentIqTest ? 0.0f : row.openFieldAngle, 6); Serial.print(','); Serial.print(currentIqTest ? 0.0f : row.unslewedUq, 6); Serial.print(',');
    Serial.print(row.adc1Frames); Serial.print(','); Serial.print(row.adc2Frames); Serial.print(','); Serial.print(row.adcAge); Serial.print(',');
    Serial.print(row.iaAdc); Serial.print(','); Serial.print(row.ibAdc); Serial.print(','); Serial.print(row.icAdc); Serial.print(',');
    Serial.print(row.vbusAdc); Serial.print(','); Serial.print(row.temperatureAdc); Serial.print(',');
    Serial.print(currentIqTest ? row.ud : 0.0f, 6); Serial.print(',');
    Serial.print(currentIqTest ? row.iqTarget : 0.0f, 6); Serial.print(',');
    Serial.print(currentIqTest ? row.iqFiltered : 0.0f, 6); Serial.print(',');
    Serial.print(currentIqTest ? row.idFiltered : 0.0f, 6); Serial.print(',');
    Serial.println(row.windowAgeUs);
  }
  Serial.println("TRACE_END");
  reportIndexReferences();
  if (currentProbeTest) {
    const auto& probe=activeCurrentProbe();
    Serial.print("CURRENT_PROBE_SUMMARY calls="); Serial.print(probe.calls);
    Serial.print(" clipped_calls="); Serial.print(probe.clipped);
    Serial.print(" peak_nominal_A="); Serial.print(probe.peak,6);
    Serial.print(" invalid="); Serial.println(probe.invalid ? 1 : 0);
    Serial.println("CURRENT_PROBE_SAMPLES elapsed_us,pid_nominal_A,probe_nominal_A,total_nominal_A");
    for (uint8_t n=0;n<feedforwardCount;++n) {
      const auto& row=feedforwardTrace[n];
      Serial.print(row.elapsedUs);Serial.print(',');Serial.print(row.pid,6);Serial.print(',');
      Serial.print(row.correction,6);Serial.print(',');Serial.println(row.total,6);
    }
  }
  if (currentFeedforwardTest) {
    Serial.print("CURRENT_FF_SUMMARY seen_indices="); Serial.print(positionFeedforward.seenIndices);
    Serial.print(" armed="); Serial.print(positionFeedforward.armed ? 1 : 0);
    Serial.print(" active_calls="); Serial.print(positionFeedforward.activeCalls);
    Serial.print(" peak_requested_A="); Serial.print(positionFeedforward.maxRequested, 6);
    Serial.print(" invalid="); Serial.println(positionFeedforward.invalid ? 1 : 0);
    Serial.println("CURRENT_FF_SAMPLES elapsed_us,pid_nominal_A,correction_nominal_A,total_nominal_A");
    for (uint8_t n = 0; n < feedforwardCount; ++n) {
      const auto& row = feedforwardTrace[n];
      Serial.print(row.elapsedUs); Serial.print(','); Serial.print(row.pid, 6); Serial.print(',');
      Serial.print(row.correction, 6); Serial.print(','); Serial.println(row.total, 6);
    }
  }
  if (currentAdaptiveTest) {
    const auto& adaptive=activeAdaptive();
    Serial.print("ADAPTIVE_SUMMARY calls=");Serial.print(adaptive.calls);
    Serial.print(" learned_calls=");Serial.print(adaptive.learned);
    Serial.print(" clipped_calls=");Serial.print(adaptive.clipped);
    Serial.print(" cosine_nominal_A=");Serial.print(adaptive.cosineA,6);
    Serial.print(" sine_nominal_A=");Serial.print(adaptive.sineA,6);
    Serial.print(" peak_nominal_A=");Serial.print(adaptive.peakA,6);
    Serial.print(" invalid=");Serial.println(adaptive.invalid ? 1 : 0);
    Serial.println("ADAPTIVE_SAMPLES elapsed_us,pid_nominal_A,correction_nominal_A,total_nominal_A");
    for (uint8_t n=0;n<feedforwardCount;++n) {
      const auto& row=feedforwardTrace[n];
      Serial.print(row.elapsedUs);Serial.print(',');Serial.print(row.pid,6);Serial.print(',');
      Serial.print(row.correction,6);Serial.print(',');Serial.println(row.total,6);
    }
  }
}
void reportPwm() {
  Serial.print("PWM hz="); Serial.print(kPwmFrequency);
  Serial.print(" dead_zone="); Serial.print(kPwmDeadZone, 6);
  Serial.print(" system_clock_Hz="); Serial.print(SystemCoreClock);
  Serial.print(" tim1_cr1="); Serial.print(TIM1->CR1);
  Serial.print(" tim1_psc="); Serial.print(TIM1->PSC);
  Serial.print(" tim1_arr="); Serial.print(TIM1->ARR);
  Serial.print(" tim1_bdtr="); Serial.print(TIM1->BDTR);
  Serial.print(" rcc_cfgr="); Serial.print(RCC->CFGR);
  Serial.print(" gate_inputs_low="); Serial.println(BoardCurrentObserver::gatesOff() ? 1 : 0);
  Serial.print("FIRMWARE diag_version="); Serial.println(kDiagVersion);
}
uint8_t abiState() { return AbiQuality::stateFromPort(GPIOB->IDR); }
void onA() {
  ++edgesA;
  const uint8_t state = abiState(); abiQuality.edge(state);
  if (manual.active) manual.edge(state);
  encoder.handleA();
}
void onB() {
  ++edgesB;
  const uint8_t state = abiState(); abiQuality.edge(state);
  if (manual.active) manual.edge(state);
  encoder.handleB();
}
void onIndex() {
  ++indexPulses;
  const uint8_t slot = indexReferenceCount;
  if (slot < 8) {
    // Public accessor is one atomic 32-bit counter read plus float scaling;
    // do not call update()/handleIndex(), which would alter interrupt/counter state.
    indexReferences[slot].angle = encoder.getSensorAngle();
    indexReferences[slot].bootUs = micros();
    indexReferences[slot].pulse = indexPulses;
    indexReferenceCount = slot + 1;
  }
  manual.index(edgesA, edgesB);
}

void endManual() {
  noInterrupts();
  const bool wasActive = manual.active;
  manual.active = false;
  interrupts();
  if (wasActive) {
    Serial.print("MANUAL_INTEGRALS angle_rad,corrected_input_rad,filtered_rad,legacy_filtered_rad: ");
    Serial.print(encoder.getAngle() - manualStartAngle, 6); Serial.print(',');
    Serial.print(manualCorrectedIntegral, 6); Serial.print(',');
    Serial.print(manualFilteredIntegral, 6); Serial.print(','); Serial.println(manualLegacyIntegral, 6);
    manual.dump(Serial);
  }
}

void startManual() {
  if (running || manual.active) {
    Serial.println("Rejected: stop current operation before manual capture.");
    return;
  }
  if (ready) driver.disable();
  attempted = manualLocked = true;
  encoder.update();
  encoder.useRolling = false; encoder.window.periodUs = 10000;
  encoder.window.reset(encoder.getAngle(), micros());
  manualWindowAngle = encoder.getAngle();
  manualWindowUs = micros();
  manualIntegralUs = manualWindowUs;
  manualStartAngle = manualWindowAngle;
  manualCorrectedIntegral = manualFilteredIntegral = manualLegacyIntegral = 0;
  previousCorrected = previousFiltered = previousLegacy = 0;
  manualLastReport = millis();
  noInterrupts();
  manual.begin(abiState());
  interrupts();
  Serial.println("MANUAL_BEGIN: gates OFF; all drive commands locked until reset; x=end capture.");
  Serial.println("CONFIG encoder_ppr=1000 counts_per_turn=4000 estimator=window10ms; raw=legacy_shadow");
  reportPwm();
  Serial.println("MANUAL ms,angle_rad,raw_rad_s,filtered_rad_s,window_rad_s,signed_count,invalid,duplicate,reversals,A_edges,B_edges,index");
}

void startSenseIdle() {
  if (running || manual.active || senseAttempted) {
    Serial.println("Rejected: sense init requires idle and one initialization per boot.");
    return;
  }
  if (ready) motor.disable();
  senseAttempted = true;
  attempted = manualLocked = true;
  Serial.println("SENSE_BEGIN: gates OFF; all drive commands locked; initialize ADC observer only.");
  const bool ok = ready && currentObserver.beginDisabled(driver);
  if (ready) driver.disable();
  senseIdle = ok;
  Serial.print("SENSE_INIT ready="); Serial.print(ok ? 1 : 0);
  Serial.print(" gates_off="); Serial.println(BoardCurrentObserver::gatesOff() ? 1 : 0);
  reportPwm();
  if (!ok) return;
  Serial.print("SENSE_OFFSETS_ADC ");
  for (int n = 0; n < 3; ++n) { if (n) Serial.print(','); Serial.print(currentObserver.offset[n], 3); }
  Serial.println();
  Serial.println("SENSE_CONFIG shunt_ohm=.003 gain=-64/7 ADC_ref_V=3.3_nominal VBUS_divider=187/18; no current control or current limit.");
  Serial.println("SENSE ms,adc1_frames,adc2_frames,age_us,a_adc,b_adc,c_adc,vbus_adc,temp_adc,ia_est_A,ib_est_A,ic_est_A,vbus_est_V,gates_off");
}

void reportSenseIdle() {
  if (!senseIdle || millis() - senseLastReport < 200) return;
  senseLastReport = millis();
  const auto frame = currentObserver.read();
  const uint32_t age = micros() - frame.us;
  if (age > 2000 || !BoardCurrentObserver::gatesOff()) {
    if (ready) driver.disable();
    senseIdle = false;
    Serial.println("SENSE_FAULT: stale ADC or gates not OFF; drive locked.");
    return;
  }
  Serial.print("SENSE,"); Serial.print(senseLastReport); Serial.print(',');
  Serial.print(frame.adc1Frames); Serial.print(','); Serial.print(frame.adc2Frames); Serial.print(','); Serial.print(age); Serial.print(',');
  Serial.print(frame.a); Serial.print(','); Serial.print(frame.b); Serial.print(','); Serial.print(frame.c); Serial.print(',');
  Serial.print(frame.vbus); Serial.print(','); Serial.print(frame.temperature); Serial.print(',');
  Serial.print(currentObserver.current(frame.a, 0), 4); Serial.print(',');
  Serial.print(currentObserver.current(frame.b, 1), 4); Serial.print(',');
  Serial.print(currentObserver.current(frame.c, 2), 4); Serial.print(',');
  Serial.print(BoardCurrentObserver::busVoltage(frame.vbus), 4); Serial.println(",1");
}

void reportManual() {
  const uint32_t now = micros();
  const double integralDt = (now - manualIntegralUs) * 1e-6;
  manualCorrectedIntegral += (previousCorrected + encoder.correctedVelocity) * 0.5 * integralDt;
  manualFilteredIntegral += (previousFiltered + measuredVelocity) * 0.5 * integralDt;
  manualLegacyIntegral += (previousLegacy + encoder.legacyFilteredVelocity) * 0.5 * integralDt;
  manualIntegralUs = now;
  previousCorrected = encoder.correctedVelocity;
  previousFiltered = measuredVelocity;
  previousLegacy = encoder.legacyFilteredVelocity;
  if (now - manualWindowUs >= 10000) {
    const float angle = encoder.getAngle();
    manualWindowVelocity = (angle - manualWindowAngle) * 1e6f / (now - manualWindowUs);
    manualWindowAngle = angle;
    manualWindowUs = now;
  }
  if (millis() - manualLastReport < 20) return;
  manualLastReport = millis();
  noInterrupts();
  const int32_t count = manual.count;
  const uint32_t invalid = manual.invalid, duplicate = manual.duplicate, reversals = manual.reversals;
  const uint32_t a = edgesA, b = edgesB, index = indexPulses;
  interrupts();
  Serial.print("MANUAL,"); Serial.print(manualLastReport); Serial.print(',');
  Serial.print(encoder.getAngle(), 6); Serial.print(',');
  Serial.print(encoder.rawVelocity, 6); Serial.print(',');
  Serial.print(measuredVelocity, 6); Serial.print(',');
  Serial.print(manualWindowVelocity, 6); Serial.print(','); Serial.print(count); Serial.print(',');
  Serial.print(invalid); Serial.print(','); Serial.print(duplicate); Serial.print(',');
  Serial.print(reversals); Serial.print(','); Serial.print(a); Serial.print(',');
  Serial.print(b); Serial.print(','); Serial.println(index);
}

void stopMotor(const char* reason) {
  const bool wasRunning = running;
  const uint32_t stoppedElapsedUs = micros() - traceStartUs;
  const float stoppedDisplacement = encoder.getAngle() - startAngle;
  const float stoppedUq = motor.voltage.q;
  motor.actuatorStats.armed = false;
  if (wasRunning) runAbiEnd = readAbiSnapshot();
  if (ready) motor.disable();
  running = false;
  if (wasRunning) {
    recordTrace(true, stoppedUq);
    tracePending = true;
  }
  Serial.print("STOP: "); Serial.println(reason);
  if (wasRunning) {
    Serial.print("RUN_REFERENCE start_angle_rad="); Serial.print(startAngle, 6);
    Serial.print(" zero_electric_angle_rad="); Serial.print(motor.zero_electric_angle, 6);
    Serial.print(" sensor_direction="); Serial.print(static_cast<int>(motor.sensor_direction));
    Serial.print(" open_field_start_rad="); Serial.println(openTest ? _3PI_2 / kMotorPolePairs : 0.0f, 6);
    Serial.print("RUN_SUMMARY elapsed_us,displacement_rad: ");
    Serial.print(stoppedElapsedUs); Serial.print(','); Serial.println(stoppedDisplacement, 6);
    Serial.print("STOP_DETAIL elapsed_us,filtered_rad_s,raw_rad_s,window10ms_rad_s: ");
    Serial.print(stoppedElapsedUs); Serial.print(',');
    Serial.print(measuredVelocity, 6); Serial.print(',');
    Serial.print(encoder.rawVelocity, 6); Serial.print(',');
    Serial.println(windowVelocity, 6);
    if (currentVelocityTest || currentStaticProbeTest) {
      Serial.print("STOP_GUARD source="); Serial.print(encoder.useRolling ? "original_window2ms_LPF2ms_shadow" : "controller_filtered_velocity");
      Serial.print(" last_observed_rad_s="); Serial.print(lastGuardVelocity, 6);
      Serial.print(" run_abs_peak_rad_s="); Serial.println(runGuardAbsPeak, 6);
      const auto& stats=motor.actuatorStats;
      Serial.print("CURRENT_ACTUATOR_STATS start_us="); Serial.print(stats.ignoreStartupUs);
      Serial.print(" covered_us="); Serial.print(stats.coveredUs);
      Serial.print(" calls="); Serial.print(stats.calls);
      const float duration=stats.coveredUs*1e-6f;
      Serial.print(" q_slew_limited_fraction="); Serial.print(stats.coveredUs ? stats.slewUs/static_cast<float>(stats.coveredUs) : 0, 6);
      Serial.print(" q_vector_limited_fraction="); Serial.print(stats.coveredUs ? stats.vectorUs/static_cast<float>(stats.coveredUs) : 0, 6);
      Serial.print(" mean_q_error_nominal_A="); Serial.print(duration ? stats.errorIntegral/duration : 0, 6);
      Serial.print(" rms_q_error_nominal_A="); Serial.print(duration ? sqrtf(stats.errorSquaredIntegral/duration) : 0, 6);
      Serial.print(" mean_requested_minus_applied_q_V="); Serial.print(duration ? stats.differenceIntegral/duration : 0, 6);
      Serial.print(" max_requested_applied_q_difference_V="); Serial.println(stats.maxDifference, 6);
    }
  }
}

bool updateObservedFrame() {
  if (!observeCurrents) return true;
  observedFrame = currentObserver.read();
  const uint32_t now = micros();
  return observationGuard.check(now, now - observedFrame.us,
      static_cast<int32_t>(observedFrame.adc2Frames - observedFrame.adc1Frames),
      observedFrame.a, observedFrame.b, observedFrame.c, BoardCurrentObserver::busVoltage(observedFrame.vbus)) == ObservationGuard::none;
}

// Let the rotor settle at both ends, rather than measuring during a fast sweep.
// Keep the same 0.5 electrical-radian pole-pair tolerance as SimpleFOC.
void recordAlignment(float commandAngle, uint32_t leg) {
  if (!alignmentOnlyTest || alignmentCount >= kTraceCapacity) return;
  // Reuse the run buffer: this command never performs the subsequent run.
  auto& row = trace[alignmentCount++];
  row.elapsedUs = micros() - alignmentStartUs;
  row.loopUs = leg;
  row.angle = encoder.getAngle(); row.openFieldAngle = commandAngle;
  row.iaAdc = observedFrame.a; row.ibAdc = observedFrame.b; row.icAdc = observedFrame.c;
  row.vbusAdc = observedFrame.vbus;
}

void dumpAlignment() {
  Serial.print("ALIGN_TRACE_CONFIG sample_step_stride="); Serial.print(kAlignmentSampleStride);
  Serial.print(" moving_samples_per_leg="); Serial.print(500 * kAlignCycles / kAlignmentSampleStride + 1);
  Serial.print(" expected_samples="); Serial.println(2 * (500 * kAlignCycles / kAlignmentSampleStride + 1) + 3);
  Serial.println("ALIGN_TRACE_BEGIN: outputs disabled; command is electrical field displacement; leg 0/1=forward/reverse, 2/3=settled endpoints.");
  Serial.println("align_us,leg,field_displacement_rad,encoder_angle_rad,ia_adc,ib_adc,ic_adc,vbus_adc");
  for (uint16_t n = 0; n < alignmentCount; ++n) {
    const auto& row = trace[n];
    Serial.print(row.elapsedUs); Serial.print(','); Serial.print(row.loopUs); Serial.print(',');
    Serial.print(row.openFieldAngle, 6); Serial.print(','); Serial.print(row.angle, 6); Serial.print(',');
    Serial.print(row.iaAdc); Serial.print(','); Serial.print(row.ibAdc); Serial.print(',');
    Serial.print(row.icAdc); Serial.print(','); Serial.println(row.vbusAdc);
  }
  Serial.println("TRACE_END");
  reportIndexReferences();
}

bool alignSlowly() {
  const AlignmentPlan plan(currentCoverageTest);
  const uint32_t alignmentBeganUs=micros();
  alignmentFailure=nullptr;
  Serial.print("ALIGN_PROFILE cycles=");Serial.print(plan.cycles);
  Serial.print(" step_ms=4 voltage_V=.7 max_alignment_us=");Serial.println(plan.deadlineUs);
  auto waitWithStop = [alignmentBeganUs,&plan](uint32_t duration) {
    const uint32_t start = millis();
    while (millis() - start < duration) {
      if (plan.expired(micros(),alignmentBeganUs)) {alignmentFailure="coverage alignment exceeded 20 seconds";return false;}
      encoder.update();
      if (!updateObservedFrame()) return false;
      if (Serial.available() && Serial.read() == 's') return false;
      delay(1);
    }
    return true;
  };
  motor.setPhaseVoltage(kAlignVoltage, 0, _3PI_2);
  if (!waitWithStop(700)) return false;
  const float before = encoder.getAngle();
  alignmentCount = 0; alignmentStartUs = micros();
  recordAlignment(0, 2);
  for (int step = 0; step <= plan.steps(); ++step) {
    motor.setPhaseVoltage(kAlignVoltage, 0, _3PI_2 + _2PI * step / 500.0f);
    if (!waitWithStop(4)) return false;
    if (step % kAlignmentSampleStride == 0) recordAlignment(_2PI * step / 500.0f, 0);
  }
  if (!waitWithStop(700)) return false;
  const float middle = encoder.getAngle();
  recordAlignment(_2PI * plan.cycles, 2);
  for (int step = plan.steps(); step >= 0; --step) {
    motor.setPhaseVoltage(kAlignVoltage, 0, _3PI_2 + _2PI * step / 500.0f);
    if (!waitWithStop(4)) return false;
    if (step % kAlignmentSampleStride == 0) recordAlignment(_2PI * step / 500.0f, 1);
  }
  if (!waitWithStop(700)) return false;
  const float after = encoder.getAngle();
  recordAlignment(0, 3);
  const float forward = middle - before, reverse = middle - after;
  Serial.print("ALIGN measured radians forward/reverse: ");
  Serial.print(forward, 5); Serial.print('/'); Serial.println(reverse, 5);
  motor.pp_check_result = plan.passes(forward,reverse,kMotorPolePairs);
  if (!motor.pp_check_result) return false;
  motor.sensor_direction = forward > 0 ? Direction::CW : Direction::CCW;
  motor.zero_electric_angle = _normalizeAngle(
      static_cast<int>(motor.sensor_direction) * kMotorPolePairs * encoder.getMechanicalAngle());
  motor.setPhaseVoltage(0, 0, 0);
  return true;
}

// The library's custom callback runs after its single shaft-velocity read and
// uses its existing motion downsampling. PID gains and voltage bounds are kept.
float asymmetricVelocityControl(FOCMotor* controlled) {
  if (!running) return 0;
  if (fixedUqTest > 0) {
    controlled->shaft_velocity_sp = 0; // This diagnostic has no speed target.
    unslewedPidUq = 0; // PID is not called in this mode.
    return magnitudeSlew.update(fixedUqTest, micros());
  }
  controlled->shaft_velocity_sp = controlled->target;
  unslewedPidUq = controlled->PID_velocity(controlled->target - controlled->shaft_velocity);
  return magnitudeSlew.update(unslewedPidUq, micros(), forwardOnlyTest);
}

float probedCurrentVelocityControl(FOCMotor* controlled) {
  if (!running) return 0;
  const uint32_t elapsedUs=micros()-traceStartUs;
  controlled->shaft_velocity_sp=currentStaticProbeTest ? 0 : controlled->target;
  const float pid=currentStaticProbeTest ? controlled->target : controlled->PID_velocity(controlled->target-controlled->shaft_velocity);
  auto& probe=activeCurrentProbe();
  const float total=probe.mix(pid,elapsedUs,controlled->current_limit);
  if (feedforwardCount<16 && (!feedforwardCount || elapsedUs-lastFeedforwardUs>=(currentStaticProbeTest ? 61000u : 173000u))) {
    feedforwardTrace[feedforwardCount++]={elapsedUs,pid,probe.value,total};lastFeedforwardUs=elapsedUs;
  }
  return total;
}

float adaptiveCurrentVelocityControl(FOCMotor* controlled) {
  if (!running) return 0;
  const uint32_t now=micros(),elapsedUs=now-traceStartUs;
  controlled->shaft_velocity_sp=controlled->target;
  const float error=controlled->target-controlled->shaft_velocity;
  const float pid=controlled->PID_velocity(error);
  auto& adaptive=activeAdaptive();
  const float total=adaptive.update(static_cast<int>(controlled->sensor_direction)*encoder.getAngle(),error,
      controlled->shaft_velocity,pid,controlled->current_limit,controlled->target<0 ? -1 : 1,now);
  if (feedforwardCount<16 && (!feedforwardCount || elapsedUs-lastFeedforwardUs>=173000)) {
    feedforwardTrace[feedforwardCount++]={elapsedUs,pid,adaptive.correction,total};lastFeedforwardUs=elapsedUs;
  }
  return total;
}

float compensatedCurrentVelocityControl(FOCMotor* controlled) {
  if (!running) return 0;
  const uint32_t elapsedUs = micros() - traceStartUs;
  const uint8_t count = indexReferenceCount;
  for (uint8_t n = positionFeedforward.seenIndices; n < count; ++n)
    positionFeedforward.observeIndex(indexReferences[n].angle, elapsedUs);
  positionFeedforward.update(encoder.getAngle(), static_cast<int>(controlled->sensor_direction), elapsedUs);
  controlled->shaft_velocity_sp = controlled->target;
  const float pid = controlled->PID_velocity(controlled->target-controlled->shaft_velocity);
  const float total = positionFeedforward.mix(pid, controlled->current_limit);
  if (feedforwardCount < 16 && (!feedforwardCount || elapsedUs-lastFeedforwardUs >= 200000)) {
    feedforwardTrace[feedforwardCount++] = {elapsedUs, pid, positionFeedforward.correction, total};
    lastFeedforwardUs = elapsedUs;
  }
  return total;
}

void startTest(bool openLoop = false, bool asymmetric = false, bool forwardOnly = false, float fixedUq = 0, float pGain = 0.20f, float velocityFilter = 0.05f, bool currentCapture = false, bool phasePulse = false, CurrentTrial currentTrial = CurrentTrial::none, bool alignmentSurvey = false) {
  if (!ready || attempted || manualLocked) {
    Serial.println("Rejected: unavailable or already attempted; reset for another test.");
    return;
  }
  attempted = true;
  switch (currentTrial) {
    case CurrentTrial::none: fixedIqGoal = 0; break;
    case CurrentTrial::fixed015: fixedIqGoal = .15f; break;
    case CurrentTrial::fixed025: fixedIqGoal = .25f; break;
    case CurrentTrial::velocity: fixedIqGoal = 0; break;
    case CurrentTrial::velocityCompensated: fixedIqGoal = 0; break;
    case CurrentTrial::velocityRolling: fixedIqGoal = 0; break;
    case CurrentTrial::velocityProbe: fixedIqGoal = 0; break;
    case CurrentTrial::staticProbe: fixedIqGoal = .10f; break;
    case CurrentTrial::staticProbe10: fixedIqGoal = .10f; break;
    case CurrentTrial::velocityFastIntegral: fixedIqGoal = 0; break;
    case CurrentTrial::velocityReverse: fixedIqGoal = 0; break;
    case CurrentTrial::velocityCoverage: fixedIqGoal = 0; break;
    case CurrentTrial::velocityLongWindow: fixedIqGoal = 0; break;
    case CurrentTrial::velocityLowerFilter: fixedIqGoal = 0; break;
    case CurrentTrial::velocityDense: fixedIqGoal = 0; break;
    case CurrentTrial::velocityDenseLowerP: fixedIqGoal = 0; break;
    case CurrentTrial::velocityHigherI: fixedIqGoal = 0; break;
    case CurrentTrial::velocityHigherIReverse: fixedIqGoal = 0; break;
    case CurrentTrial::velocityStrongerI: fixedIqGoal = 0; break;
    case CurrentTrial::velocityStrongerIReverse: fixedIqGoal = 0; break;
    case CurrentTrial::velocityHigherIDense: fixedIqGoal = 0; break;
    case CurrentTrial::velocityStrongerIDense: fixedIqGoal = 0; break;
    case CurrentTrial::velocityAdaptive: fixedIqGoal = 0; break;
    case CurrentTrial::velocityAdaptiveZero: fixedIqGoal = 0; break;
    case CurrentTrial::velocityResponseProbe: fixedIqGoal = 0; break;
    case CurrentTrial::velocityResponseProbeInverse: fixedIqGoal = 0; break;
    case CurrentTrial::velocityAdaptiveFlat: fixedIqGoal = 0; break;
    default: stopMotor("invalid nominal current profile"); return;
  }
  currentIqTest = currentTrial != CurrentTrial::none;
  currentResponseProbeInverseTest=currentTrial == CurrentTrial::velocityResponseProbeInverse;
  currentResponseProbeTest=currentTrial == CurrentTrial::velocityResponseProbe || currentResponseProbeInverseTest;
  currentAdaptiveZeroTest=currentTrial == CurrentTrial::velocityAdaptiveZero;
  currentAdaptiveFlatTest=currentTrial == CurrentTrial::velocityAdaptiveFlat;
  currentAdaptiveTest=currentTrial == CurrentTrial::velocityAdaptive || currentAdaptiveZeroTest || currentAdaptiveFlatTest;
  const bool higherIReverseTrial=currentTrial == CurrentTrial::velocityHigherIReverse;
  const bool strongerIReverseTrial=currentTrial == CurrentTrial::velocityStrongerIReverse;
  const bool strongerITrial=currentTrial == CurrentTrial::velocityStrongerI || strongerIReverseTrial || currentTrial == CurrentTrial::velocityStrongerIDense;
  const bool higherITrial=currentTrial == CurrentTrial::velocityHigherI || higherIReverseTrial || strongerITrial || currentTrial == CurrentTrial::velocityHigherIDense || currentAdaptiveTest || currentResponseProbeTest;
  currentCoverageTest = currentTrial == CurrentTrial::velocityCoverage;
  currentReverseTest = currentTrial == CurrentTrial::velocityReverse || currentCoverageTest || higherIReverseTrial || strongerIReverseTrial;
  currentVelocityGoal=currentReverseTest ? -kTargetVelocity : kTargetVelocity;
  const bool lowerPTrial=currentTrial == CurrentTrial::velocityDenseLowerP;
  const bool denseTrial=currentTrial == CurrentTrial::velocityDense || lowerPTrial || currentTrial == CurrentTrial::velocityHigherIDense || currentTrial == CurrentTrial::velocityStrongerIDense;
  currentFastIntegralTest = currentTrial == CurrentTrial::velocityFastIntegral || currentReverseTest || currentTrial == CurrentTrial::velocityLongWindow || currentTrial == CurrentTrial::velocityLowerFilter || denseTrial || higherITrial;
  currentVelocityTest = currentTrial == CurrentTrial::velocity || currentTrial == CurrentTrial::velocityCompensated || currentTrial == CurrentTrial::velocityRolling || currentTrial == CurrentTrial::velocityProbe || currentFastIntegralTest;
  encoder.useRolling = currentTrial == CurrentTrial::velocityRolling || currentTrial == CurrentTrial::velocityProbe || currentFastIntegralTest;
  encoder.rolling.longWindow = currentTrial == CurrentTrial::velocityLongWindow;
  encoder.guardFilter = LowPassFilter(.002f); // Age through alignment, matching the first-call reset of the controller LPF.
  currentFeedforwardTest = currentTrial == CurrentTrial::velocityCompensated;
  currentStaticProbe10Test = currentTrial == CurrentTrial::staticProbe10;
  currentStaticProbeTest = currentTrial == CurrentTrial::staticProbe || currentStaticProbe10Test;
  currentProbeTest = currentTrial == CurrentTrial::velocityProbe || currentStaticProbeTest || currentResponseProbeTest;
  const bool projectTracking=currentVelocityTest || currentStaticProbeTest;
  tracePeriodUs=currentStaticProbeTest ? 4000 : denseTrial ? 2000 : kTracePeriodUs;
  currentProbe.reset();staticCurrentProbe.reset();staticCurrentProbe10.reset();
  responseCurrentProbe.reset();responseCurrentProbeInverse.reset();
  positionFeedforward.reset(); feedforwardCount = 0; lastFeedforwardUs = 0;
  encoder.window.periodUs = currentVelocityTest ? 2000 : 10000;
  alignmentOnlyTest = alignmentSurvey;
  phasePulseTest = phasePulse;
  observeCurrents = currentCapture || currentIqTest;
  if (observeCurrents) {
    if (senseAttempted) { stopMotor("ADC already attempted; reset required"); return; }
    senseAttempted = true;
    Serial.println("CURRENT_INIT: driver disabled, calibrate observer before alignment; no current control.");
    if (!currentObserver.beginDisabled(driver)) { stopMotor("ADC observer initialization failed"); return; }
    observedFrame = currentObserver.read();
    observationGuard.reset(BoardCurrentObserver::busVoltage(observedFrame.vbus));
    Serial.print("SENSE_OFFSETS_ADC ");
    for (int n = 0; n < 3; ++n) { if (n) Serial.print(','); Serial.print(currentObserver.offset[n], 3); }
    Serial.println();
    Serial.print("CURRENT_CONFIG observer_only="); Serial.print(currentIqTest ? 0 : 1);
    Serial.print(" bus_low_V="); Serial.print(observationGuard.busLow, 4);
    Serial.print(" bus_high_V="); Serial.print(observationGuard.busHigh, 4);
    Serial.println(" deviation_delay_us=20000 max_age_us=2000 nominal_ADC_ref_V=3.3 shunt_ohm=.003 gain=-64/7");
    if (!updateObservedFrame()) { stopMotor(ObservationGuard::reason(observationGuard.fault)); return; }
  }
  openTest = openLoop;
  fixedUqTest = fixedUq;
  asymmetricTest = asymmetric || fixedUqTest > 0;
  forwardOnlyTest = asymmetric && forwardOnly;
  if (currentIqTest) {
    observerSense.linkDriver(&driver);
    if (!observerSense.init()) { stopMotor("current adapter not ready"); return; }
    motor.linkCurrentSense(&observerSense);
    motor.updateTorqueControlType(TorqueControlType::foc_current);
    motor.controller = currentVelocityTest ? MotionControlType::velocity : MotionControlType::torque;
    motor.updateCurrentLimit(currentVelocityTest ? .45f : currentStaticProbeTest ? .15f : fixedIqGoal); // Nominal setpoint bound only.
    if (currentVelocityTest) {
      // Explicit current-mode gains; CONTROL records the actual trial settings.
      motor.PID_velocity.P = lowerPTrial ? .10f : currentFastIntegralTest ? .15f : .20f;
      motor.PID_velocity.I = strongerITrial ? 9.60f : higherITrial ? 7.20f : currentFastIntegralTest ? 4.80f : 1.20f;
      motor.PID_velocity.D = 0;
      motor.PID_velocity.output_ramp = 0; // Project d/q voltage limits remain active.
      motor.LPF_velocity.Tf = currentFeedforwardTest ? .005f : currentTrial == CurrentTrial::velocityLowerFilter ? .001f : .002f; // Freeze rejected f at its measured reference profile.
    }
    motor.PID_current_q.P = motor.PID_current_d.P = .20f;
    motor.PID_current_q.I = motor.PID_current_d.I = 50.0f;
    motor.PID_current_q.D = motor.PID_current_d.D = 0;
    // Current-velocity comparison uses project slew and immediate release.
    motor.PID_current_q.output_ramp = motor.PID_current_d.output_ramp = projectTracking ? 0.0f : 3.0f;
    motor.asymmetricCurrentVoltage = projectTracking;
    motor.trackingCurrentControl = projectTracking;
    if (projectTracking) {
      motor.qPI.P = .60f; motor.qPI.I = currentStaticProbeTest || currentFastIntegralTest ? 200.0f : 100.0f;
      motor.qSlew.riseRateVPerS = motor.qSlew.negativeRiseRateVPerS = currentFeedforwardTest ? 30.0f : 100.0f;
    }
    if (projectTracking) {
      // The project PI uses the library's filtered dq measurement and supplies
      // actuator feedback. Leave the unused stock PI at zero in this mode.
      motor.PID_current_q.P = motor.PID_current_d.P = 0;
      motor.PID_current_q.I = motor.PID_current_d.I = 0;
    }
    motor.LPF_current_q.Tf = motor.LPF_current_d.Tf = .002f;
    if (currentFeedforwardTest) {
      motor.controller = MotionControlType::custom;
      motor.linkCustomMotionControl(compensatedCurrentVelocityControl);
    } else if (currentProbeTest) {
      motor.controller = MotionControlType::custom;
      motor.linkCustomMotionControl(probedCurrentVelocityControl);
    } else if (currentAdaptiveTest) {
      motor.controller = MotionControlType::custom;
      motor.linkCustomMotionControl(adaptiveCurrentVelocityControl);
    }
  }
  if (asymmetricTest) {
    motor.PID_velocity.P = pGain;
    motor.controller = MotionControlType::custom;
    motor.linkCustomMotionControl(asymmetricVelocityControl);
    motor.PID_velocity.output_ramp = 0; // MagnitudeSlew applies build-up externally.
    motor.LPF_velocity.Tf = velocityFilter;
    motor.PID_velocity.I = 0.15f;
  }
  Serial.print("CONFIG encoder_ppr=1000 counts_per_turn=4000 estimator_window_us=");
  Serial.print(encoder.window.periodUs); Serial.println(" trace_window_us=10000 raw=legacy_shadow");
  if (encoder.useRolling) {
    Serial.print("ESTIMATOR controller=rolling_LS");Serial.print(encoder.rolling.horizonUs()/1000);Serial.print("ms min_sample_us=");Serial.print(encoder.rolling.minimumSampleUs());
    Serial.print(" warmup_span_us=");Serial.print(encoder.rolling.warmupUs());Serial.println(" guard=original_window2ms_LPF2ms guard_limit_rad_s=5");
  }
  reportPwm();
  if (currentIqTest) {
    if (currentVelocityTest) {
      Serial.print("DRIVE current_velocity=1 speed_target_rad_s=");Serial.print(currentVelocityGoal,3);
      Serial.println(" nominal_iq_limit_A=.45 current_control=on legacy_raw_velocity=not_recorded");
    }
    else {
      Serial.print("DRIVE fixed_iq_nominal_A="); Serial.print(fixedIqGoal, 4);
      Serial.println(" ramp_us=300000 speed_target=none speed_PID=off current_control=on legacy_raw_velocity=not_recorded");
    }
    Serial.print("CURRENT_LOOP q_P=");
    Serial.print(projectTracking ? motor.qPI.P : motor.PID_current_q.P, 2);
    Serial.print(" q_I=");
    Serial.print(projectTracking ? motor.qPI.I : motor.PID_current_q.I, 1);
    Serial.print(" d_P=.20 d_I=50 D=0 q_voltage_build_up_V_s=");
    Serial.print(projectTracking ? motor.qSlew.riseRateVPerS : 3.0f, 1);
    Serial.print(" current_LPF_s="); Serial.print(motor.LPF_current_q.Tf, 4);
    Serial.print(" vector_voltage_limit_V=.5 release=");
    Serial.println(projectTracking ? "immediate reversal=via_zero; nominal current scaling" : "symmetric; nominal current scaling");
    if (projectTracking) Serial.println("CURRENT_ENGINE project_tracking_PI=1 actuator_feedback=applied_dq stock_current_PI=unused");
    if (projectTracking) {
      Serial.print("CURRENT_SLEW positive_q_V_s="); Serial.print(motor.qSlew.riseRateVPerS, 1);
      Serial.print(" negative_q_V_s="); Serial.print(motor.qSlew.negativeRiseRateVPerS, 1);
      Serial.println(" d_V_s=3 release=immediate reversal=via_zero");
    }
    Serial.print("CURRENT_TELEMETRY running_stream_CSV=off trace=RAM period_us=");Serial.print(tracePeriodUs);Serial.println(" stop_and_commands=enabled");
    if (currentFeedforwardTest)
      Serial.println("CURRENT_FF_CONFIG order=84 cos_A=-.0085 sin_A=.0253 correction_bound_A=.03 total_bound_A=.45 gate_us=800000 blend_us=200000 requires_two_matching_Index=1");
    if (currentProbeTest) {
      const auto& probe=activeCurrentProbe();
      Serial.print("CURRENT_PROBE frequency_Hz="); Serial.print(probe.frequencyHz,2);
      Serial.print(" phase_rad=");Serial.print(probe.phaseRad,6);
      Serial.print(" amplitude_nominal_A="); Serial.print(probe.amplitudeA,4);
      Serial.print(" start_us=");Serial.print(probe.gateUs);Serial.print(" blend_us=");Serial.print(probe.blendUs);
      Serial.print(" total_bound_A=");Serial.print(motor.current_limit,4);
      Serial.print(" stationary_attempt=");Serial.print(currentStaticProbeTest ? 1 : 0);Serial.println(" excitation_only=1");
    }
  }
  if (fixedUqTest > 0) {
    Serial.print("DRIVE fixed_uq_V="); Serial.print(fixedUqTest, 4);
    Serial.println(" speed_PID=off speed_target=none sensor_commutation=on");
  }
  if (asymmetricTest) {
    Serial.print("LIMITER asymmetric: magnitude build-up "); Serial.print(magnitudeSlew.riseRateVPerS, 3);
    Serial.print("V/s, immediate release, reversal via zero, bound ");
    Serial.println(forwardOnlyTest ? "0..0.5V" : "+/-0.5V");
    Serial.print("OUTPUT_POLICY forward_only="); Serial.println(forwardOnlyTest ? 1 : 0);
  }
  if (currentAdaptiveTest) {
    Serial.print("ADAPTIVE_CONFIG order=84 learning_rate=");Serial.print(activeAdaptive().learningRate,4);
    Serial.print(" correction_bound_nominal_A=");Serial.print(activeAdaptive().boundA,6);
    Serial.print(" response_lag_rad=");Serial.print(activeAdaptive().responseLagRad,6);
    Serial.print(" gate_us=800000 blend_us=200000 total_limit_nominal_A=");Serial.print(motor.current_limit,4);
    Serial.println(" anchor=run_start per_run_reset=1");
  }
  Serial.print("CONTROL P="); Serial.print(motor.PID_velocity.P, 4);
  Serial.print(" I="); Serial.print(motor.PID_velocity.I, 4);
  Serial.print(" D="); Serial.print(motor.PID_velocity.D, 4);
  Serial.print(" output_ramp_V_s="); Serial.print(motor.PID_velocity.output_ramp, 4);
  Serial.print(" velocity_LPF_s="); Serial.println(motor.LPF_velocity.Tf, 4);
  Serial.print("TRACE_CONFIG period_us=");Serial.print(tracePeriodUs);
  Serial.println(" capacity=320 loop_us_saturation=65535 window_age_us_saturation=65535 window_span_us_nominal=10000");
  if (denseTrial) Serial.println("TRACE_SCOPE retained_tail_only=1 nominal_span_us=640000 whole_run_guard_stats=1 velocity_window_us=10000");
  if (phasePulseTest) {
    Serial.println("DRIVE phase_pulse=1 amplitude_V=.10 rise_V_s=3 pulse_us=150000 rest_us=100000 count=6 total_us=1500000 speed_PID=off speed_target=none Uq_column=unused");
    if (!motor.init()) { stopMotor("motor init failed"); return; }
    encoder.update();
    startAngle = encoder.getAngle();
    runStart = millis();
    beginTrace();
    running = true;
    Serial.println("RUN PHASE: U+/U-/V+/V-/W+/W- static vectors, max .10V, 1.5 seconds; velocity and ADC/VBUS guards retained.");
    return;
  }
  if (openTest) {
    motor.controller = MotionControlType::velocity_openloop;
    motor.velocity_limit = kOpenTargetVelocity;
    motor.motion_downsample = 0;
    if (!motor.init()) { stopMotor("motor init failed"); return; }
    Serial.println("ALIGN OPEN: establish starting field before speed ramp.");
    if (!alignSlowly()) { stopMotor("open-loop starting alignment failed"); return; }
    motor.updateVoltageLimit(kOpenVoltageLimit);
    // Continue from the exact electrical field used at the end of alignment.
    motor.shaft_angle = _3PI_2 / kMotorPolePairs;
    encoder.update();
    startAngle = encoder.getAngle();
    runStart = millis();
    beginTrace();
    running = true;
    Serial.print("RUN OPEN: ramp speed to "); Serial.print(kOpenTargetVelocity, 3);
    Serial.print(" rad/s at "); Serial.print(kOpenVoltageLimit, 3);
    Serial.println(" V; stop after 3 seconds.");
    return;
  }
  Serial.print("ALIGNING: slow 0.7 V, ");Serial.print(currentCoverageTest ? 4 : kAlignCycles);
  Serial.println(" electrical turns each way, with settling.");
  if (!motor.init()) { stopMotor("motor init failed"); return; }
  if (!alignSlowly()) {
    stopMotor(observeCurrents && observationGuard.fault != ObservationGuard::none ? ObservationGuard::reason(observationGuard.fault) : alignmentFailure ? alignmentFailure : "slow alignment aborted or pole-pair check failed");
    return;
  }
  if (alignmentOnlyTest) {
    stopMotor("alignment survey complete");
    dumpAlignment();
    return;
  }
  if (!motor.initFOC()) { stopMotor("FOC alignment failed"); return; }
  if (!motor.pp_check_result) { stopMotor("pole-pair check failed"); return; }
  motor.updateVoltageLimit(kVoltageLimit);
  if (currentIqTest) {
    motor.current_sp = motor.target = 0;
    motor.PID_current_q.reset(); motor.PID_current_d.reset();
    // Clear filter state after alignment while preserving the selected trial.
    motor.LPF_current_q = LowPassFilter(motor.LPF_current_q.Tf);
    motor.LPF_current_d = LowPassFilter(motor.LPF_current_d.Tf);
    motor.vectorBoundEnabled = true;
    motor.qSlew.reset(micros()); motor.dSlew.reset(micros());
    motor.qPI.reset(micros()); motor.dPI.reset(micros());
    Serial.print("CURRENT_ACTIVE q_LPF_s="); Serial.print(motor.LPF_current_q.Tf, 4);
    Serial.print(" d_LPF_s="); Serial.print(motor.LPF_current_d.Tf, 4);
    Serial.print(" velocity_LPF_s="); Serial.print(motor.LPF_velocity.Tf, 4);
    Serial.print(" q_P=");Serial.print(motor.trackingCurrentControl ? motor.qPI.P : motor.PID_current_q.P,4);
    Serial.print(" q_I=");Serial.print(motor.trackingCurrentControl ? motor.qPI.I : motor.PID_current_q.I,4);
    Serial.print(" velocity_P=");Serial.print(motor.PID_velocity.P,4);
    Serial.print(" velocity_I=");Serial.println(motor.PID_velocity.I,4);
  }
  // initFOC's cached angle can age while UART metadata is emitted. Refresh
  // before the reference and velocity-window reset, without reading velocity.
  encoder.update();
  startAngle = encoder.getAngle();
  runStart = millis();
  beginTrace();
  running = true;
  if (currentStaticProbeTest) Serial.println("RUN CURRENT: 1-second stationary attempt, nominal Iq .10A plus .02A probe, setpoint cap .15A; .03rad displacement stop; all existing guards retained.");
  else if (currentVelocityTest) {
    Serial.print("RUN CURRENT VELOCITY: target ");Serial.print(currentVelocityGoal,3);
    Serial.println("rad/s, nominal Iq limit +/-.45A, vector cap .5V; existing speed/stall/3s guards.");
  }
  else if (currentIqTest) Serial.println("RUN CURRENT: 300ms ramp to nominal Iq target; vector cap .5V; existing speed/stall/3s guards.");
  else if (fixedUqTest > 0) Serial.println("RUN FIXED: ramp to fixed Uq at 3V/s; no speed target; same 5rad/s and stall guards; max 3 seconds.");
  else Serial.println("RUN: target 1 rad/s for 3 seconds, voltage limited to 0.5 V.");
}

void setup() {
  const uint32_t gates[] = {PA8, PC13, PA9, PA12, PA10, PB15};
  for (uint32_t pin : gates) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);
  encoder.pullup = Pullup::USE_EXTERN;
  encoder.init();
  abiQuality.reset(abiState());
  encoder.enableInterrupts(onA, onB);
  pinMode(PB8, INPUT);
  attachInterrupt(digitalPinToInterrupt(PB8), onIndex, RISING);
  driver.voltage_power_supply = kBusVoltage;
  driver.voltage_limit = kBusVoltage;
  driver.pwm_frequency = kPwmFrequency;
  driver.dead_zone = kPwmDeadZone;
  ready = driver.init();
  if (ready) {
    driver.disable();
    BoardCurrentObserver::configureGatePullDowns();
  }
  motor.linkDriver(&driver);
  motor.linkSensor(&encoder);
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_limit = kAlignVoltage;
  motor.voltage_sensor_align = kAlignVoltage;
  motor.velocity_limit = 2.0f;
  motor.PID_velocity.P = 0.20f;
  motor.PID_velocity.I = 0.15f;
  motor.PID_velocity.D = 0;
  motor.PID_velocity.output_ramp = kVelocityOutputRamp;
  motor.LPF_velocity.Tf = 0.05f;
  motor.motion_downsample = 4;
  Serial.println(ready ? "READY: gates OFF. t=closed-loop, a=P.20, b=P.10, d=P.10 Tf.02, p=forward-only, o=open-loop, u=fixed .20V, v=fixed .25V, s=stop; one test per boot."
                       : "ERROR: PWM driver init failed; no test possible.");
  Serial.println("Current feedback is disabled at boot; r/n/m enable nominal current control. Supply current limit must remain enabled.");
  Serial.print("DIAG v"); Serial.print(kDiagVersion);
  Serial.println(": m=current velocity, g=rolling speed, q/Q=forward/reverse candidate, w=velocity current probe, z/y=1s static 20/10Hz probes, f=Index correction, r/n=nominal .15/.25A Iq; e=alignment survey; l=observed open-loop; k=phase pulses; j=observed b; i=ADC-only.");
  Serial.println("C=reverse candidate with 4-cycle alignment/20s alignment deadline; H=forward candidate with 8ms rolling speed; run ceiling remains 3s.");
  Serial.println("L=forward candidate with velocity LPF 1ms, rolling speed 4ms; speed guard retains window2ms/LPF2ms.");
  Serial.println("D=forward candidate with 2ms RAM trace, last ~.64s only; all drive settings and guards unchanged.");
  Serial.println("E=D with velocity P reduced from .15 to .10, I remains 4.80; bounds and guards unchanged.");
  Serial.println("I=forward candidate with velocity I 7.20 instead of 4.80, P .15; full 3s RAM trace and unchanged bounds/guards.");
  Serial.println("R=I with target -1rad/s; q/Q remain forward/reverse I4.80 references.");
  Serial.println("B=I with velocity I 9.60 instead of 7.20; same speed, bounds and guards.");
  Serial.println("S=B with target -1rad/s; M/N=forward I7.20/I9.60 with 2ms RAM tails; same bounds/guards.");
  Serial.println("A/Z=I7.20 with bounded per-run adaptive order84 correction / zero-learning comparison; .03A correction and existing total/voltage/velocity limits.");
  Serial.println("T/U=I7.20 with 17.5Hz/.02A known probe at phase 0/pi; adaptive correction off, existing bounds/guards.");
  Serial.println("F=A with the adaptive response-lag hypothesis 0rad instead of .785398rad; same correction and total bounds.");
  reportPwm();
  Serial.println("ms,running,A_edges,B_edges,index,angle_rad,velocity_rad_s,Uq_V");
}

void loop() {
  while (Serial.available()) {
    const char command = Serial.read();
    if (command == 's') { stopMotor("serial stop"); endManual(); senseIdle = false; }
    else if (command == 'h') startManual();
    else if (command == 'x') endManual();
    else if (command == 'c') reportPwm();
    else if (command == 'i') startSenseIdle();
    else if (command == 'j') startTest(false, true, false, 0, 0.10f, 0.05f, true);
    else if (command == 'k') startTest(false, false, false, 0, 0.20f, 0.05f, true, true);
    else if (command == 'r') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::fixed015);
    else if (command == 'n') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::fixed025);
    else if (command == 'm') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocity);
    else if (command == 'f') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityCompensated);
    else if (command == 'g') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityRolling);
    else if (command == 'w') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityProbe);
    else if (command == 'z') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::staticProbe);
    else if (command == 'y') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::staticProbe10);
    else if (command == 'q') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityFastIntegral);
    else if (command == 'Q') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityReverse);
    else if (command == 'C') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityCoverage);
    else if (command == 'H') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityLongWindow);
    else if (command == 'L') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityLowerFilter);
    else if (command == 'D') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityDense);
    else if (command == 'E') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityDenseLowerP);
    else if (command == 'I') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityHigherI);
    else if (command == 'R') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityHigherIReverse);
    else if (command == 'B') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityStrongerI);
    else if (command == 'S') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityStrongerIReverse);
    else if (command == 'M') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityHigherIDense);
    else if (command == 'N') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityStrongerIDense);
    else if (command == 'A') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityAdaptive);
    else if (command == 'Z') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityAdaptiveZero);
    else if (command == 'T') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityResponseProbe);
    else if (command == 'U') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityResponseProbeInverse);
    else if (command == 'F') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::velocityAdaptiveFlat);
    else if (command == 'e') startTest(false, false, false, 0, 0.20f, 0.05f, true, false, CurrentTrial::none, true);
    else if (command == 'l') startTest(true, false, false, 0, 0.20f, 0.05f, true);
    else if (command == 't') startTest();
    else if (command == 'a') startTest(false, true);
    else if (command == 'b') startTest(false, true, false, 0, 0.10f);
    else if (command == 'd') startTest(false, true, false, 0, 0.10f, 0.02f);
    else if (command == 'p') startTest(false, true, true);
    else if (command == 'o') startTest(true);
    else if (command == 'u') startTest(false, false, false, 0.20f);
    else if (command == 'v') startTest(false, false, false, 0.25f);
  }
  encoder.update();
  if (openTest || phasePulseTest || !running) measuredVelocity = observedVelocity(encoder.getVelocity());
  if (manual.active) { reportManual(); return; }
  if (running) {
    if (!updateObservedFrame()) {
      stopMotor(ObservationGuard::reason(observationGuard.fault));
      if (tracePending) dumpTrace();
      return;
    }
    const uint32_t loopNow = micros();
    loopUs = loopNow - lastLoopUs;
    lastLoopUs = loopNow;
    const uint32_t windowUs = loopNow - windowStartUs;
    if (windowUs >= 10000) {
      const float angle = encoder.getAngle();
      windowVelocity = (angle - windowAngle) * 1e6f / windowUs;
      windowAngle = angle;
      windowStartUs = loopNow;
    }
    const uint32_t elapsed = millis() - runStart;
    if (phasePulseTest) {
      commandedVelocity = 0;
      const auto voltage = PhasePulseProfile::at(micros() - traceStartUs);
      const float center = kBusVoltage / 2;
      driver.setPwm(center + voltage.a, center + voltage.b, center + voltage.c);
    } else if (openTest) {
      commandedVelocity = kOpenTargetVelocity * _constrain(elapsed / 800.0f, 0.0f, 1.0f);
      motor.move(commandedVelocity);
      motor.loopFOC();
    } else if (currentIqTest) {
      commandedVelocity = currentVelocityTest ? currentVelocityGoal : 0;
      const float appliedTarget = motor.current_sp;
      motor.loopFOC();
      currentControlTrace = {motor.current.q, motor.current.d, motor.voltage.d, appliedTarget};
      motor.move(currentVelocityTest ? currentVelocityGoal : nominalIqRamp(fixedIqGoal, micros() - traceStartUs));
      measuredVelocity = motor.shaft_velocity;
    } else {
      commandedVelocity = fixedUqTest > 0 ? 0.0f : kTargetVelocity;
      motor.loopFOC();
      motor.move(commandedVelocity);
      measuredVelocity = motor.shaft_velocity;
    }
    recordTrace(false, motor.voltage.q);
    const float monitoredVelocity = encoder.useRolling ? encoder.guardVelocity : openTest || phasePulseTest ? measuredVelocity : motor.shaft_velocity;
    lastGuardVelocity = monitoredVelocity;
    if (isfinite(monitoredVelocity)) runGuardAbsPeak = fmax(runGuardAbsPeak, fabs(monitoredVelocity));
    const bool stalled = !phasePulseTest && motionWatchdog.stalled(encoder.getAngle(), micros());
    if (currentAdaptiveTest && activeAdaptive().invalid) stopMotor("invalid adaptive correction or unexpected adaptive control gap");
    else if (currentProbeTest && activeCurrentProbe().invalid) stopMotor("invalid current probe");
    else if (currentFeedforwardTest && positionFeedforward.invalid) stopMotor("invalid position correction or inconsistent Index");
    else if (currentIqTest && motor.vectorBound.invalid) stopMotor("non-finite current voltage vector");
    else if (asymmetricTest && magnitudeSlew.invalid) stopMotor("non-finite PID output");
    else if (!isfinite(monitoredVelocity) || !isfinite(measuredVelocity) || fabs(monitoredVelocity) > 5.0f)
      stopMotor("unexpected velocity");
    else if (stalled) stopMotor("shaft movement stalled");
    else if (currentStaticProbeTest && fabs(encoder.getAngle()-startAngle)>.03f) stopMotor("movement during stationary current probe");
    else if (currentStaticProbeTest && micros()-traceStartUs>=1000000) stopMotor("1-second current probe complete");
    else if (phasePulseTest && micros() - traceStartUs >= PhasePulseProfile::durationUs)
      stopMotor("1.5-second phase-pulse complete");
    else if (elapsed >= kRunMs) stopMotor("3-second test complete");
    else if (!phasePulseTest && elapsed >= 1200 && fabs(encoder.getAngle() - startAngle) < 0.03f)
      stopMotor("no shaft movement");
  } else encoder.update();
  if (tracePending) dumpTrace();
  reportSenseIdle();
  if (running && currentIqTest) return; // Stop polling and RAM trace remain active; avoid UART stalls in current control.
  static uint32_t lastReport = 0;
  const uint32_t now = millis();
  if (now - lastReport < 200) return;
  lastReport = now;
  noInterrupts();
  const uint32_t a = edgesA, b = edgesB, i = indexPulses;
  interrupts();
  Serial.print(now); Serial.print(','); Serial.print(running); Serial.print(',');
  Serial.print(a); Serial.print(','); Serial.print(b); Serial.print(',');
  Serial.print(i); Serial.print(','); Serial.print(encoder.getAngle(), 5); Serial.print(',');
  Serial.print(measuredVelocity, 5); Serial.print(',');
  Serial.println(running ? motor.voltage.q : 0.0f, 4);
}
