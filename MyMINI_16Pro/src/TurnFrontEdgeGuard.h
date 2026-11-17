#pragma once

#include <stdint.h>

// Latch the selected turn exit before clearing the front edges. The selected
// sensor may leave black during cleanup and must not need to trigger again.
class TurnFrontEdgeGuard {
 public:
  explicit TurnFrontEdgeGuard(uint16_t whiteThreshold)
      : whiteThreshold_(whiteThreshold) {}

  bool targetReached() const { return targetReached_; }

  bool update(bool selectedExitConfirmed, uint16_t leftRaw, uint16_t rightRaw,
               uint16_t leftFiltered, uint16_t rightFiltered) {
    targetReached_ = targetReached_ || selectedExitConfirmed;
    // f_line reads filtered samples; raw-only release could hand it stale black.
    return targetReached_ && leftRaw >= whiteThreshold_ &&
        rightRaw >= whiteThreshold_ && leftFiltered >= whiteThreshold_ &&
        rightFiltered >= whiteThreshold_;
  }

 private:
  uint16_t whiteThreshold_;
  bool targetReached_ = false;
};
