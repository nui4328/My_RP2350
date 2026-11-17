#pragma once

#include <stdint.h>

// Normal f_line front-sensor crossings with stopPull=0 only. Feed one sample
// per fresh frame after black confirmation; true means both exit sensors are
// white in that same frame. Four of the latest six samples confirm release.
class LineCrossingWhiteFilter {
 public:
  bool update(bool bothWhite) {
    if (history_[index_]) --whiteCount_;
    history_[index_] = bothWhite;
    if (bothWhite) ++whiteCount_;
    index_ = static_cast<uint8_t>((index_ + 1u) % 6u);
    return whiteCount_ >= 4u;
  }

 private:
  bool history_[6] = {};
  uint8_t index_ = 0u;
  uint8_t whiteCount_ = 0u;
};
