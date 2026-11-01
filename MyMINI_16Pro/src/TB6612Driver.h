#pragma once

#include <Arduino.h>
#include "RobotConfig.h"

class TB6612Driver {
public:
  enum class Motor : uint8_t { Left, Right };

  void begin();
  void set(Motor motor, int16_t commandPercent);
  void setBoth(int16_t leftPercent, int16_t rightPercent);
  void stopAll();

  int16_t leftCommand() const { return leftCommand_; }
  int16_t rightCommand() const { return rightCommand_; }

private:
  static int16_t constrainCommand_(int16_t commandPercent);

  int16_t leftCommand_ = 0;
  int16_t rightCommand_ = 0;
};
