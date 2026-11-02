#pragma once

#include <Arduino.h>

#include "RobotConfig.h"

// One PH/EN DRV8874 module drives one motor. PMODE must be tied to GND in the
// hardware for this class; IMODE, VREF and IPROPI deliberately have no GPIO
// fields because their circuit configuration is board-specific.
struct DRV8874MotorPins {
  uint8_t enablePwmPin;
  uint8_t phasePin;
  uint8_t sleepPin;
  // True only when the schematic confirms nSLEEP is hard-wired high. In that
  // case sleepPin must be DRV8874_PIN_UNASSIGNED and no GPIO is touched.
  bool sleepHardwiredHigh;
  // Set only after a wheel-off-ground direction check. This is motor wiring
  // polarity, independent of which driver is selected.
  bool invertDirection = false;
};

class DRV8874Driver {
public:
  enum class Motor : uint8_t { Left, Right };

  // Independent coast/drive requires separate GPIO nSLEEP pins.
  static bool validConfiguration(const DRV8874MotorPins& left,
                                 const DRV8874MotorPins& right);
  bool begin(const DRV8874MotorPins& left, const DRV8874MotorPins& right);
  void set(Motor motor, int16_t commandPercent);
  void setBoth(int16_t leftPercent, int16_t rightPercent);
  void stopAll();

  bool ready() const { return ready_; }
  int16_t leftCommand() const { return leftCommand_; }
  int16_t rightCommand() const { return rightCommand_; }

private:
  static int16_t constrainCommand_(int16_t commandPercent);
  void writeMotor_(const DRV8874MotorPins& pins, int16_t commandPercent,
                   int16_t& storedCommand, bool& asleep);

  DRV8874MotorPins leftPins_ = {MyMINIConfig::DRV8874_PIN_UNASSIGNED,
                                MyMINIConfig::DRV8874_PIN_UNASSIGNED,
                                MyMINIConfig::DRV8874_PIN_UNASSIGNED, false,
                                false};
  DRV8874MotorPins rightPins_ = {MyMINIConfig::DRV8874_PIN_UNASSIGNED,
                                 MyMINIConfig::DRV8874_PIN_UNASSIGNED,
                                 MyMINIConfig::DRV8874_PIN_UNASSIGNED, false,
                                 false};
  bool ready_ = false;
  bool leftAsleep_ = true;
  bool rightAsleep_ = true;
  int16_t leftCommand_ = 0;
  int16_t rightCommand_ = 0;
};
