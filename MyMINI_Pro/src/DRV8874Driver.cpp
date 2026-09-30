#include "DRV8874Driver.h"

#include "RobotConfig.h"

namespace {

bool pinAssigned(uint8_t pin) {
  return pin != MyMINIConfig::DRV8874_PIN_UNASSIGNED;
}

bool sameDrivePin(const DRV8874MotorPins& left,
                  const DRV8874MotorPins& right) {
  const uint8_t drivePins[] = {left.enablePwmPin, left.phasePin,
                               right.enablePwmPin, right.phasePin};
  for (uint8_t first = 0; first < 4; ++first) {
    if (!pinAssigned(drivePins[first])) return true;
    for (uint8_t second = first + 1; second < 4; ++second) {
      if (drivePins[first] == drivePins[second]) return true;
    }
  }
  return false;
}

bool sleepConflictsWithDrive(const DRV8874MotorPins& left,
                             const DRV8874MotorPins& right) {
  const uint8_t sleepPins[] = {left.sleepPin, right.sleepPin};
  const bool hardwired[] = {left.sleepHardwiredHigh, right.sleepHardwiredHigh};
  const uint8_t drivePins[] = {left.enablePwmPin, left.phasePin,
                               right.enablePwmPin, right.phasePin};
  for (uint8_t index = 0; index < 2; ++index) {
    const uint8_t sleepPin = sleepPins[index];
    if (hardwired[index]) {
      if (pinAssigned(sleepPin)) return true;
      continue;
    }
    if (!pinAssigned(sleepPin)) return true;
    for (uint8_t drivePin : drivePins) {
      if (sleepPin == drivePin) return true;
    }
  }
  return false;
}

void setSleepLow(const DRV8874MotorPins& left,
                 const DRV8874MotorPins& right) {
  if (left.sleepHardwiredHigh && right.sleepHardwiredHigh) return;
  if (!left.sleepHardwiredHigh) {
    pinMode(left.sleepPin, OUTPUT);
    digitalWrite(left.sleepPin, LOW);
  }
  if (!right.sleepHardwiredHigh &&
      (left.sleepHardwiredHigh || right.sleepPin != left.sleepPin)) {
    pinMode(right.sleepPin, OUTPUT);
    digitalWrite(right.sleepPin, LOW);
  }
}

void setSleepHigh(const DRV8874MotorPins& left,
                  const DRV8874MotorPins& right) {
  if (left.sleepHardwiredHigh && right.sleepHardwiredHigh) return;
  if (!left.sleepHardwiredHigh) digitalWrite(left.sleepPin, HIGH);
  if (!right.sleepHardwiredHigh &&
      (left.sleepHardwiredHigh || right.sleepPin != left.sleepPin)) {
    digitalWrite(right.sleepPin, HIGH);
  }
}

}  // namespace

bool DRV8874Driver::begin(const DRV8874MotorPins& left,
                          const DRV8874MotorPins& right) {
  ready_ = false;
  leftCommand_ = 0;
  rightCommand_ = 0;
  if (!validConfiguration(left, right)) {
    return false;
  }

  leftPins_ = left;
  rightPins_ = right;

  // For GPIO nSLEEP, hold both bridges asleep before configuring inputs.
  // PMODE and IMODE are latched while nSLEEP rises; their board-level values
  // must already be set. A confirmed hardware-high nSLEEP is never driven.
  setSleepLow(leftPins_, rightPins_);
  pinMode(leftPins_.enablePwmPin, OUTPUT);
  pinMode(leftPins_.phasePin, OUTPUT);
  pinMode(rightPins_.enablePwmPin, OUTPUT);
  pinMode(rightPins_.phasePin, OUTPUT);
  digitalWrite(leftPins_.phasePin, LOW);
  digitalWrite(rightPins_.phasePin, LOW);

  analogWriteResolution(12);
  analogWriteFreq(MyMINIConfig::MOTOR_PWM_FREQUENCY_HZ);
  analogWrite(leftPins_.enablePwmPin, 0);
  analogWrite(rightPins_.enablePwmPin, 0);
  if (!leftPins_.sleepHardwiredHigh || !rightPins_.sleepHardwiredHigh) {
    delay(1);  // Meets the DRV8874 maximum 1 ms tSLEEP before wake.
    setSleepHigh(leftPins_, rightPins_);
    delay(1);  // Meets the DRV8874 maximum 1 ms tWAKE before accepting drive.
  }

  ready_ = true;
  stopAll();  // PH/EN: EN=0 is normal low-side brake, never sleep.
  return true;
}

bool DRV8874Driver::validConfiguration(const DRV8874MotorPins& left,
                                       const DRV8874MotorPins& right) {
  return !sameDrivePin(left, right) && !sleepConflictsWithDrive(left, right);
}

void DRV8874Driver::set(DRV8874Driver::Motor motor,
                        int16_t commandPercent) {
  if (!ready_) return;
  if (motor == Motor::Left) {
    writeMotor_(leftPins_, commandPercent, leftCommand_);
  } else {
    writeMotor_(rightPins_, commandPercent, rightCommand_);
  }
}

void DRV8874Driver::setBoth(int16_t leftPercent, int16_t rightPercent) {
  if (!ready_) return;
  writeMotor_(leftPins_, leftPercent, leftCommand_);
  writeMotor_(rightPins_, rightPercent, rightCommand_);
}

void DRV8874Driver::stopAll() {
  if (!ready_) return;
  writeMotor_(leftPins_, 0, leftCommand_);
  writeMotor_(rightPins_, 0, rightCommand_);
}

int16_t DRV8874Driver::constrainCommand_(int16_t commandPercent) {
  return constrain(commandPercent, -static_cast<int16_t>(MyMINIConfig::MOTOR_COMMAND_MAX),
                   static_cast<int16_t>(MyMINIConfig::MOTOR_COMMAND_MAX));
}

void DRV8874Driver::writeMotor_(const DRV8874MotorPins& pins,
                                int16_t commandPercent,
                                int16_t& storedCommand) {
  const int16_t command = constrainCommand_(commandPercent);
  if (command == 0) {
    // Table 3, PH/EN mode: nSLEEP=1, EN=0 drives both outputs low (brake).
    // Do not take nSLEEP low for normal stopping because that enters Hi-Z sleep.
    analogWrite(pins.enablePwmPin, 0);
    storedCommand = 0;
    return;
  }

  bool forward = command > 0;
  if (pins.invertDirection) forward = !forward;
  analogWrite(pins.enablePwmPin, 0);
  digitalWrite(pins.phasePin, forward ? HIGH : LOW);
  const uint16_t pwm = map(abs(command), 0,
                           static_cast<int>(MyMINIConfig::MOTOR_COMMAND_MAX),
                           0, static_cast<int>(MyMINIConfig::MOTOR_PWM_MAX));
  analogWrite(pins.enablePwmPin, pwm);
  storedCommand = command;
}
