#pragma once

// Only bias steering while the existing branch guard has identified the
// opposite wing and the trusted main line remains visible.
inline float steerAwayFromOppositeBranch(float correction, int selectedSide,
                                        int blockedSide, bool mainLineFound) {
  if (!mainLineFound || (selectedSide != -1 && selectedSide != 1) ||
      blockedSide != -selectedSide) return correction;
  constexpr float kMinimumAwayCorrection = 8.0f;
  if (selectedSide > 0 && correction < kMinimumAwayCorrection)
    return kMinimumAwayCorrection;
  if (selectedSide < 0 && correction > -kMinimumAwayCorrection)
    return -kMinimumAwayCorrection;
  return correction;
}
