#include "MotorDriver.h"

#include "DRV8874Driver.h"
#include "RobotConfig.h"

namespace {

enum class MotorDirection : uint8_t { Stopped, Forward, Reverse };

MotorDirection leftDirection = MotorDirection::Stopped;
MotorDirection rightDirection = MotorDirection::Stopped;
MotorDriverType selectedDriver = MotorDriverType::DRV8874;
MotorDriverStatus driverStatus = MotorDriverStatus::DRV8874_NOT_CONFIGURED;
const DRV8874MotorPins drv8874LeftPins = {
    MyMINIConfig::DRV8874_LEFT_ENABLE_PWM,
    MyMINIConfig::DRV8874_LEFT_PHASE,
    MyMINIConfig::DRV8874_LEFT_NSLEEP,
    MyMINIConfig::DRV8874_NSLEEP_HARDWIRED_HIGH,
    MyMINIConfig::DRV8874_LEFT_MOTOR_INVERTED,
};
const DRV8874MotorPins drv8874RightPins = {
    MyMINIConfig::DRV8874_RIGHT_ENABLE_PWM,
    MyMINIConfig::DRV8874_RIGHT_PHASE,
    MyMINIConfig::DRV8874_RIGHT_NSLEEP,
    MyMINIConfig::DRV8874_NSLEEP_HARDWIRED_HIGH,
    MyMINIConfig::DRV8874_RIGHT_MOTOR_INVERTED,
};
DRV8874Driver drv8874;

void writeTb6612Motor(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin,
                      bool inverted, int speed,
                      MotorDirection& currentDirection) {
  speed = constrain(speed, -100, 100);

  if (speed == 0 || speed == 1 || speed == -1) {
    // STBY is hard-wired HIGH on this board. LOW/LOW/HIGH is Hi-Z;
    // HIGH/HIGH/HIGH is short brake in the TB6612FNG truth table.
    const bool brake = speed != 0;
    analogWrite(pwmPin, 0);
    digitalWrite(in1Pin, brake ? HIGH : LOW);
    digitalWrite(in2Pin, brake ? HIGH : LOW);
    analogWrite(pwmPin, 4095);  // Constant HIGH, never a low duty brake.
    currentDirection = MotorDirection::Stopped;
    return;
  }

  bool forward = speed > 0;
  if (inverted) forward = !forward;
  const MotorDirection nextDirection = forward ? MotorDirection::Forward
                                                : MotorDirection::Reverse;
  if (nextDirection != currentDirection) {
    analogWrite(pwmPin, 0);
  }

  digitalWrite(in1Pin, forward ? HIGH : LOW);
  digitalWrite(in2Pin, forward ? LOW : HIGH);
  const uint16_t pwm = map(abs(speed), 0, 100, 0, 4095);
  analogWrite(pwmPin, pwm);
  currentDirection = nextDirection;
}

void beginTb6612() {
  using namespace MyMINIConfig;

  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  analogWriteResolution(12);
  // Preserve the legacy TB6612FNG output frequency exactly. The DRV8874
  // implementation uses its separately documented configuration value.
  analogWriteFreq(20000);
}

void writeTb6612Both(int sl, int sr) {
  using namespace MyMINIConfig;

  // Preserve this established logical-left/logical-right polarity and the
  // existing TB6612 board routing exactly; DRV8874 has its own pin objects.
  writeTb6612Motor(RIGHT_MOTOR_PWM, RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2,
                   MOTOR_LEFT_INVERT, sl, leftDirection);
  writeTb6612Motor(LEFT_MOTOR_PWM, LEFT_MOTOR_IN1, LEFT_MOTOR_IN2,
                   MOTOR_RIGHT_INVERT, sr, rightDirection);
}

} // namespace

bool select_motor_driver(MotorDriverType type) {
  selectedDriver = type;
  // Once DRV8874 is selected, never report the prior TB6612 state as ready.
  // robot_begin() will promote this to DRV8874_READY only after initialization.
  if (type == MotorDriverType::DRV8874) {
    driverStatus = MotorDriverStatus::DRV8874_NOT_CONFIGURED;
  }
  return true;
}

MotorDriverType selected_motor_driver() {
  return selectedDriver;
}

MotorDriverStatus motor_driver_status() {
  return driverStatus;
}

bool motor_driver_ready() {
  return driverStatus == MotorDriverStatus::TB6612_READY ||
         driverStatus == MotorDriverStatus::DRV8874_READY;
}

void motor_begin() {
  if (selectedDriver == MotorDriverType::DRV8874) {
    if (drv8874.begin(drv8874LeftPins, drv8874RightPins)) {
      driverStatus = MotorDriverStatus::DRV8874_READY;
    } else {
      driverStatus = MotorDriverStatus::DRV8874_NOT_CONFIGURED;
      Serial.println(F("DRV8874_NOT_CONFIGURED: invalid DRV8874 pin configuration; outputs disabled."));
    }
  } else {
    beginTb6612();
    driverStatus = MotorDriverStatus::TB6612_READY;
  }
  motor(0, 0);
}

void motor(int sl, int sr) {
  sl = constrain(sl, -100, 100);
  sr = constrain(sr, -100, 100);
  if (!((sl == 0 && sr == 0) ||
        (sl == 1 && sr == 1) || (sl == -1 && sr == -1))) {
    // Preserve one-wheel pivots by braking a zero wheel. Small PID/turn
    // outputs retain their sign and use the first real drive value, 2.
    if (sl == 0) sl = 1;
    else if (sl == 1) sl = 2;
    else if (sl == -1) sl = -2;
    if (sr == 0) sr = 1;
    else if (sr == 1) sr = 2;
    else if (sr == -1) sr = -2;
  }
  if (selectedDriver == MotorDriverType::DRV8874) {
    drv8874.setBoth(static_cast<int16_t>(sl), static_cast<int16_t>(sr));
    return;
  }
  writeTb6612Both(sl, sr);
}

void motor_stop() {
  motor(1, 1);
}

void set_motor_brake_at_zero(bool enabled) {
  // Compatibility API: zero now always means coast for both drivers.
  (void)enabled;
}

bool get_motor_brake_at_zero() {
  return false;
}
