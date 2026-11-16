#include <BoardCurrentObserver.h>
#include <current_sense/hardware_specific/stm32/stm32_mcu.h>
#include <current_sense/hardware_specific/stm32/stm32_adc_utils.h>
#include <current_sense/hardware_specific/stm32/b_g431/b_g431_hal.h>

#if !defined(ARDUINO_B_G431B_ESC1)
#error This observer is specific to B-G431B-ESC1 and SimpleFOC 2.4.0.
#endif
static_assert(OPAMP_USE_INTERNAL_CHANNEL == 1, "DMA layout requires internal OPAMP channels");

// SimpleFOC's board-specific DMA arrays (internal OPAMP channels enabled).
extern volatile uint16_t adcBuffer1[4];
extern volatile uint16_t adcBuffer2[2];
namespace {
volatile uint32_t adc1Frames = 0, adc2Frames = 0;
volatile BoardCurrentObserver::Frame latest = {};
}

// ADC2 completes its two current samples before ADC1's four-sample scan ends.
// Copy only at ADC1 scan completion; keep ISR work to a small RAM snapshot.
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adc) {
  if (adc->Instance == ADC2) { ++adc2Frames; return; }
  if (adc->Instance != ADC1) return;
  ++adc1Frames;
  latest.a = adcBuffer1[0];
  latest.b = adcBuffer2[0]; latest.c = adcBuffer2[1];
  latest.temperature = adcBuffer1[2]; latest.vbus = adcBuffer1[3];
  latest.us = micros();
  latest.adc2Frames = adc2Frames;
  latest.adc1Frames = adc1Frames;
}

BoardCurrentObserver::Frame BoardCurrentObserver::read() const {
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  Frame result{latest.us, latest.adc1Frames, latest.adc2Frames,
               latest.a, latest.b, latest.c, latest.vbus, latest.temperature};
  __set_PRIMASK(mask);
  return result;
}

bool BoardCurrentObserver::gatesOff() {
  if (TIM1->CCER & 0x555u) return false; // CH1/1N, CH2/2N, CH3/3N enables.
  for (uint32_t pin : {PA8, PC13, PA9, PA12, PA10, PB15})
    if (digitalRead(pin)) return false;
  return true;
}

void BoardCurrentObserver::configureGatePullDowns() {
  // Variant PinMap_TIM uses pull-ups. With CCER disabled the AF outputs float
  // to HIGH. Keep AF selection and timer settings, but bias disabled inputs LOW.
  MODIFY_REG(GPIOA->PUPDR, (3u << 16) | (3u << 18) | (3u << 20) | (3u << 24),
                         (2u << 16) | (2u << 18) | (2u << 20) | (2u << 24));
  MODIFY_REG(GPIOB->PUPDR, 3u << 30, 2u << 30);
  MODIFY_REG(GPIOC->PUPDR, 3u << 26, 2u << 26);
}

bool BoardCurrentObserver::beginDisabled(BLDCDriver6PWM& driver) {
  ready = false;
  driver.disable();
  auto* dp = static_cast<STM32DriverParams*>(driver.params);
  if (!gatesOff() || !dp || !dp->timers_handle[0] || dp->timers_handle[0]->Instance != TIM1) return false;
  const uint32_t initialPsc = TIM1->PSC, initialArr = TIM1->ARR, initialDtg = TIM1->BDTR & 255;
  params = _configureADCLowSide(driver.params, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
  driver.disable();
  if (!params || params == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return false;

  // 2.4.0's B-G431 _driverSyncLowSide dereferences params->adc_handle,
  // although _configureADCLowSide leaves it null. Use its TIM1 sync sequence
  // directly, avoiding the unused ADC lookup. No dependency source is edited.
  auto* cp = static_cast<Stm32CurrentSenseParams*>(params);
  if (!IS_TIM_REPETITION_COUNTER_INSTANCE(TIM1) || cp->timer_handle != dp->timers_handle[0]) return false;
  stm32_pause(dp);
  Stm32AdcInterruptConfig configuration{};
  const uint32_t needsInterrupt = _initTimerInterruptDownsampling(cp, dp, configuration);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);
  stm32_resume(dp);
  driver.disable();
  if (needsInterrupt || !gatesOff() || TIM1->PSC != initialPsc || TIM1->ARR != initialArr ||
      (TIM1->BDTR & 255) != initialDtg) return false;

  delay(50);
  const Frame first = read();
  delay(20);
  Frame frame = read();
  if (!first.adc1Frames || !first.adc2Frames || frame.adc1Frames <= first.adc1Frames ||
      frame.adc2Frames <= first.adc2Frames || micros() - frame.us > 2000) return false;

  uint32_t sums[3] = {};
  for (uint16_t n = 0; n < 2000; ++n) {
    frame = read();
    if (!gatesOff() || micros() - frame.us > 2000 || frame.a < 10 || frame.a > 4085 ||
        frame.b < 10 || frame.b > 4085 || frame.c < 10 || frame.c > 4085) return false;
    sums[0] += frame.a; sums[1] += frame.b; sums[2] += frame.c;
    delay(1);
  }
  for (uint8_t n = 0; n < 3; ++n) offset[n] = sums[n] / 2000.0f;
  const float bus = busVoltage(frame.vbus);
  ready = gatesOff() && bus > 9.5f && bus < 12.5f;
  driver.disable();
  return ready;
}
