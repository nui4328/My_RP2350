#pragma once

#include <stdint.h>

// Normal f_line front/center-sensor crossings with stopPull=0 only. Feed
// one sample per fresh frame after black confirmation. For a single cl/cr,
// true means that sensor is white; for pairs, both must be white together.
// Four of the latest six samples confirm release.
class LineCrossingWhiteFilter {
 public:
  static bool required(bool reverseDirection, bool frontExit, bool centerExit,
                       uint8_t stopPull) {
    return !reverseDirection && (frontExit || centerExit) && stopPull == 0u;
  }

  bool update(bool selectedWhite) {
    if (history_[index_]) --whiteCount_;
    history_[index_] = selectedWhite;
    if (selectedWhite) ++whiteCount_;
    index_ = static_cast<uint8_t>((index_ + 1u) % 6u);
    return whiteCount_ >= 4u;
  }

 private:
  bool history_[6] = {};
  uint8_t index_ = 0u;
  uint8_t whiteCount_ = 0u;
};
