#pragma once
#include <stdint.h>

// Sensor normalized 0..1000, black < 500. A candidate has at least two
// black sensors in the chosen outer four, confirmed two of three frames.
class LineSideExit {
 public:
  explicit LineSideExit(bool highSide, bool edgePairOnly = false)
      : highSide_(highSide), edgePairOnly_(edgePairOnly) {}
  bool sample(const uint16_t* normalized) {
    uint8_t count = 0;
    const uint8_t start = highSide_ ? 12u : 0u;
    for (uint8_t i = start; i < start + 4u; ++i)
      if (normalized[i] < 500u) ++count;
    const uint8_t edge = highSide_ ? 14u : 0u;
    history_[next_] = edgePairOnly_
        ? normalized[edge] < 500u && normalized[edge + 1u] < 500u
        : count >= 2u;
    next_ = (next_ + 1u) % 3u;
    return static_cast<uint8_t>(history_[0]) + history_[1] + history_[2] >= 2;
  }
  // Once detected, require the selected outer pair to clear together.
  bool sampleClear(const uint16_t* normalized) {
    const uint8_t start = highSide_ ? 14u : 0u;
    bool clear = true;
    for (uint8_t i = start; i < start + 2u; ++i)
      if (normalized[i] < 600u) clear = false;
    clearHistory_[clearNext_] = clear;
    clearNext_ = (clearNext_ + 1u) % 3u;
    return static_cast<uint8_t>(clearHistory_[0]) + clearHistory_[1] + clearHistory_[2] >= 2;
  }
 private:
  bool highSide_;
  bool edgePairOnly_;
  bool history_[3] = {false, false, false};
  uint8_t next_ = 0;
  bool clearHistory_[3] = {false, false, false};
  uint8_t clearNext_ = 0;
};
