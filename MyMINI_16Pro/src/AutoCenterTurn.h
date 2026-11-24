#pragma once
#include <stdint.h>
#include <math.h>

// Centered position uses front-array weights -50..50. The center must
// clear first; then the nearest connected black group can reach +/-10.
class AutoCenterTurn {
 public:
  bool update(const uint16_t* v, bool rawEdgesWhite, uint32_t elapsedMs) {
    if (!cleared_) {
      clearFrames_ = (v[7] >= 600u && v[8] >= 600u) ? clearFrames_ + 1u : 0u;
      if (clearFrames_ >= 2u) cleared_ = true;
      return false;
    }
    static const int8_t weights[16] = {-50,-43,-37,-30,-23,-17,-10,-3,3,10,17,23,30,37,43,50};
    bool found = false;
    float nearest = 1000.0f, position = 0.0f;
    for (uint8_t i = 0; i < 16;) {
      if (v[i] >= 500u) { ++i; continue; }
      int32_t weighted = 0; uint32_t total = 0;
      do { const uint16_t strength = 1000u - v[i];
           weighted += static_cast<int32_t>(strength) * weights[i];
           total += strength; ++i;
      } while (i < 16u && v[i] < 500u);
      const float candidate = static_cast<float>(weighted) / total;
      if (fabsf(candidate) < nearest) { nearest = fabsf(candidate); position = candidate; found = true; }
    }
    if (found) foundNew_ = true;
    const bool centered = found && foundNew_ && elapsedMs >= 40u &&
        fabsf(position) <= 10.0f && rawEdgesWhite && v[0] >= 600u && v[15] >= 600u;
    history_[next_] = centered; next_ = (next_ + 1u) % 3u;
    return static_cast<uint8_t>(history_[0]) + history_[1] + history_[2] >= 2u;
  }
  bool foundNew() const { return foundNew_; }
 private:
  bool cleared_ = false, foundNew_ = false;
  uint8_t clearFrames_ = 0, next_ = 0;
  bool history_[3] = {false, false, false};
};
