#include <Arduino.h>
#include <SimpleFOC.h>

constexpr int kEncoderPpr = 1024;
constexpr int kMotorPolePairs = 7; // Motor datasheet: 12N14P.
constexpr float kBusVoltage = 11.1f;
constexpr float kVoltageLimit = 0.5f;
constexpr float kAlignVoltage = 0.7f;
constexpr float kTargetVelocity = 1.0f; // rad/s, approximately 9.55 rpm.
constexpr uint32_t kRunMs = 3000;
constexpr int kAlignCycles = 3;
Encoder encoder(PB6, PB7, kEncoderPpr); // No index search; align each boot.
BLDCDriver6PWM driver(PA8, PC13, PA9, PA12, PA10, PB15);
BLDCMotor motor(kMotorPolePairs);
volatile uint32_t edgesA = 0, edgesB = 0, indexPulses = 0;
bool ready = false, attempted = false, running = false;
bool openTest = false;
float measuredVelocity = 0;
LowPassFilter observedVelocity(0.05f);
uint32_t runStart = 0;
float startAngle = 0;
void onA() { ++edgesA; encoder.handleA(); }
void onB() { ++edgesB; encoder.handleB(); }
void onIndex() { ++indexPulses; }

void stopMotor(const char* reason) {
  if (ready) motor.disable();
  running = false;
  Serial.print("STOP: "); Serial.println(reason);
}

// Let the rotor settle at both ends, rather than measuring during a fast sweep.
// Keep the same 0.5 electrical-radian pole-pair tolerance as SimpleFOC.
bool alignSlowly() {
  auto waitWithStop = [](uint32_t duration) {
    const uint32_t start = millis();
    while (millis() - start < duration) {
      encoder.update();
      if (Serial.available() && Serial.read() == 's') return false;
      delay(1);
    }
    return true;
  };
  motor.setPhaseVoltage(kAlignVoltage, 0, _3PI_2);
  if (!waitWithStop(700)) return false;
  const float before = encoder.getAngle();
  for (int step = 0; step <= 500 * kAlignCycles; ++step) {
    motor.setPhaseVoltage(kAlignVoltage, 0, _3PI_2 + _2PI * step / 500.0f);
    if (!waitWithStop(4)) return false;
  }
  if (!waitWithStop(700)) return false;
  const float middle = encoder.getAngle();
  for (int step = 500 * kAlignCycles; step >= 0; --step) {
    motor.setPhaseVoltage(kAlignVoltage, 0, _3PI_2 + _2PI * step / 500.0f);
    if (!waitWithStop(4)) return false;
  }
  if (!waitWithStop(700)) return false;
  const float after = encoder.getAngle();
  const float forward = middle - before, reverse = middle - after;
  Serial.print("ALIGN measured radians forward/reverse: ");
  Serial.print(forward, 5); Serial.print('/'); Serial.println(reverse, 5);
  motor.pp_check_result = forward * reverse > 0 &&
      fabs(fabs(forward) * kMotorPolePairs / kAlignCycles - _2PI) <= 0.5f &&
      fabs(fabs(reverse) * kMotorPolePairs / kAlignCycles - _2PI) <= 0.5f;
  if (!motor.pp_check_result) return false;
  motor.sensor_direction = forward > 0 ? Direction::CW : Direction::CCW;
  motor.zero_electric_angle = _normalizeAngle(
      static_cast<int>(motor.sensor_direction) * kMotorPolePairs * encoder.getMechanicalAngle());
  motor.setPhaseVoltage(0, 0, 0);
  return true;
}

void startTest(bool openLoop = false) {
  if (!ready || attempted) {
    Serial.println("Rejected: unavailable or already attempted; reset for another test.");
    return;
  }
  attempted = true;
  openTest = openLoop;
  if (openTest) {
    motor.controller = MotionControlType::velocity_openloop;
    motor.velocity_limit = 3.0f;
    motor.motion_downsample = 0;
    if (!motor.init()) { stopMotor("motor init failed"); return; }
    Serial.println("ALIGN OPEN: establish starting field before speed ramp.");
    if (!alignSlowly()) { stopMotor("open-loop starting alignment failed"); return; }
    motor.updateVoltageLimit(kAlignVoltage);
    // Continue from the exact electrical field used at the end of alignment.
    motor.shaft_angle = _3PI_2 / kMotorPolePairs;
    encoder.update();
    startAngle = encoder.getAngle();
    runStart = millis();
    running = true;
    Serial.println("RUN OPEN: ramp speed to 3 rad/s at 0.7 V; stop after 3 seconds.");
    return;
  }
  Serial.println("ALIGNING: slow 0.7 V, 3 electrical turns each way, with settling.");
  if (!motor.init()) { stopMotor("motor init failed"); return; }
  if (!alignSlowly()) { stopMotor("slow alignment aborted or pole-pair check failed"); return; }
  if (!motor.initFOC()) { stopMotor("FOC alignment failed"); return; }
  if (!motor.pp_check_result) { stopMotor("pole-pair check failed"); return; }
  motor.updateVoltageLimit(kVoltageLimit);
  startAngle = encoder.getAngle();
  runStart = millis();
  running = true;
  Serial.println("RUN: target 1 rad/s for 3 seconds, voltage limited to 0.5 V.");
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
  encoder.enableInterrupts(onA, onB);
  pinMode(PB8, INPUT);
  attachInterrupt(digitalPinToInterrupt(PB8), onIndex, RISING);
  driver.voltage_power_supply = kBusVoltage;
  driver.voltage_limit = kBusVoltage;
  driver.pwm_frequency = 25000;
  driver.dead_zone = 0.02f;
  ready = driver.init();
  if (ready) driver.disable();
  motor.linkDriver(&driver);
  motor.linkSensor(&encoder);
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_limit = kAlignVoltage;
  motor.voltage_sensor_align = kAlignVoltage;
  motor.velocity_limit = 2.0f;
  motor.PID_velocity.P = 0.08f;
  motor.PID_velocity.I = 0.15f;
  motor.PID_velocity.D = 0;
  motor.PID_velocity.output_ramp = 0.3f;
  motor.LPF_velocity.Tf = 0.05f;
  motor.motion_downsample = 4;
  Serial.println(ready ? "READY: gates OFF. t=closed-loop, o=open-loop, s=stop; one test per boot."
                       : "ERROR: PWM driver init failed; no test possible.");
  Serial.println("No phase-current sensing. Supply current limit must remain enabled.");
  Serial.println("ms,running,A_edges,B_edges,index,angle_rad,velocity_rad_s,Uq_V");
}

void loop() {
  while (Serial.available()) {
    const char command = Serial.read();
    if (command == 's') stopMotor("serial stop");
    else if (command == 't') startTest();
    else if (command == 'o') startTest(true);
  }
  encoder.update();
  if (openTest || !running) measuredVelocity = observedVelocity(encoder.getVelocity());
  if (running) {
    const uint32_t elapsed = millis() - runStart;
    if (openTest) {
      motor.move(3.0f * _constrain(elapsed / 800.0f, 0.0f, 1.0f));
      motor.loopFOC();
    } else {
      motor.loopFOC();
      motor.move(kTargetVelocity);
      measuredVelocity = motor.shaft_velocity;
    }
    const float monitoredVelocity = openTest ? measuredVelocity : motor.shaft_velocity;
    if (!isfinite(monitoredVelocity) || fabs(monitoredVelocity) > 5.0f)
      stopMotor("unexpected velocity");
    else if (elapsed >= kRunMs) stopMotor("3-second test complete");
    else if (elapsed >= 1200 && fabs(encoder.getAngle() - startAngle) < 0.03f)
      stopMotor("no shaft movement");
  } else encoder.update();
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
