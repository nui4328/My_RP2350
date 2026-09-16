#pragma once

#include <stdint.h>

class Servo {
 public:
  inline static bool forceAttachFailure = false;

  int attach(uint8_t pin, int minimumPulseUs = 1000, int maximumPulseUs = 2000) {
    if (forceAttachFailure) return -1;
    pin_ = pin;
    minimumPulseUs_ = minimumPulseUs;
    maximumPulseUs_ = maximumPulseUs;
    attached_ = true;
    return pin;
  }

  void detach() { attached_ = false; }

  void write(int degrees) { degrees_ = degrees; }

  bool attached() const { return attached_; }

 private:
  uint8_t pin_ = 255;
  int minimumPulseUs_ = 1000;
  int maximumPulseUs_ = 2000;
  int degrees_ = 0;
  bool attached_ = false;
};
