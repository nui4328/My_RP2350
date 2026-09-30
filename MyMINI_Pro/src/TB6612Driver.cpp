#include "TB6612Driver.h"

#include "MotorDriver.h"

using namespace MyMINIConfig;

void TB6612Driver::begin() {
  motor_begin();
  leftCommand_ = 0;
  rightCommand_ = 0;
}

void TB6612Driver::set(Motor motor, int16_t commandPercent) {
  commandPercent = constrainCommand_(commandPercent);
  if (motor == Motor::Left) {
    leftCommand_ = commandPercent;
  } else {
    rightCommand_ = commandPercent;
  }
  ::motor(static_cast<int>(leftCommand_), static_cast<int>(rightCommand_));
}

void TB6612Driver::setBoth(int16_t leftPercent, int16_t rightPercent) {
  leftCommand_ = constrainCommand_(leftPercent);
  rightCommand_ = constrainCommand_(rightPercent);
  ::motor(static_cast<int>(leftCommand_), static_cast<int>(rightCommand_));
}

void TB6612Driver::stopAll() {
  setBoth(0, 0);
}

int16_t TB6612Driver::constrainCommand_(int16_t commandPercent) {
  if (commandPercent > static_cast<int16_t>(MOTOR_COMMAND_MAX)) {
    return MOTOR_COMMAND_MAX;
  }
  if (commandPercent < -static_cast<int16_t>(MOTOR_COMMAND_MAX)) {
    return -static_cast<int16_t>(MOTOR_COMMAND_MAX);
  }
  return commandPercent;
}
