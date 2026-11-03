#include <MyMINI_ProInternal.h>
#include <LineFollower.h>
extern bool scanColorSensorReady;
extern String order_color[6];

// From START toward the six transverse bands. Follow the guide line near
// F11/F12, matching positoin_error = 80 in the line-following API.
namespace {
constexpr int8_t kLineWeights[16] = {
    -50, -43, -37, -30, -23, -17, -10, -3,
      3,  10,  17,  23,  30,  37,  43, 50};
constexpr uint8_t kLinePosition = 80;
constexpr float kTargetPosition = static_cast<float>(kLinePosition) - 50.0f;
// Course geometry: START to first band 17 cm; each band 2 cm wide with a
// 5 cm clear gap. There is no wheel encoder, so these are diagnostic values.
constexpr float kStartToFirstBandCm = 17.0f;
constexpr float kBandWidthCm = 2.0f;
constexpr float kFirstCenterFromStartCm =
    kStartToFirstBandCm + kBandWidthCm * 0.5f;
constexpr float kBandGapCm = 5.0f;
constexpr float kBandPitchCm = kBandWidthCm + kBandGapCm;
constexpr float kFirstToLastCenterCm = 5.0f * kBandPitchCm;
constexpr bool kWhiteFixedLast = false; // Use true only when WHITE is fixed at band 6.
constexpr float kScanKi = 0.03f;
constexpr uint32_t kTcsIntegrationMs = 50; // Configured in color.ino.
constexpr uint32_t kColorPollMs = 30; // Poll between sensor conversions to catch short bands.
constexpr uint32_t kFirstBandSampleMs = 60; // Keep stationary samples independent.
constexpr uint32_t kMinBandDwellMs = 60; // At least two 50 ms sensor conversions.
constexpr uint8_t kMinBandSamples = 3;
constexpr uint32_t kColorGapToleranceMs = 75; // Allow one unknown poll within a band.
constexpr uint8_t kMaxColorWindows = 24;
constexpr uint8_t kEarlySampleCount = 80;
constexpr uint8_t kTailSampleCount = 32;
// The next band center is 7 cm away. Rearm after leaving the 2 cm band,
// even if a colored floor reading prevents us from seeing a WHITE sample.
constexpr float kRearmDistanceMm = 40.0f;
// Two fresh 50 ms conversions within a 2 cm band need about 20 cm/s or less.
// Polling faster does not shorten the sensor's integration time.
constexpr float kRecommendedMaxCmPerSec =
    kBandWidthCm * 1000.0f / (2.0f * kTcsIntegrationMs);
// Old 101 ms / 16x readings saturated at C=44032. These are initial estimates
// for 50 ms / 4x (~1/8 exposure); tune from new stationary C measurements.
constexpr uint16_t kWhiteClearMin = 4000;
// On-track samples: BLACK C=864..1292, GREEN as low as C=1508.
constexpr uint16_t kBlackClearMax = 1400;
constexpr uint16_t kSaturatedClear = 21504; // (256 - 0xEB) * 1024.
constexpr uint32_t kLineGapTimeoutMs = 1300;
constexpr uint32_t kAlignTimeoutMs = 3500;
constexpr uint32_t kScanTimeoutMs = 25000;

const char* const kColorNames[6] = {
    "RED", "GREEN", "YELLOW", "BLUE", "WHITE", "BLACK"};

struct ColorWindow {
  int8_t color;
  uint8_t samples;
  uint8_t status; // 0 = brief, 1 = accepted, 2 = duplicate.
  uint32_t startMs;
  uint32_t endMs;
  float startMm;
  float endMm;
};

struct TailColorSample {
  uint16_t distanceMm;
  uint16_t clear;
  uint16_t red;
  uint16_t green;
  uint16_t blue;
  int8_t label;
};

void printColorSample(const __FlashStringHelper* prefix,
                      const TailColorSample& sample) {
  Serial.print(prefix);
  Serial.print(sample.distanceMm * 0.1f, 1);
  Serial.print(F(" C="));
  Serial.print(sample.clear);
  Serial.print(F(" R="));
  Serial.print(sample.red);
  Serial.print(F(" G="));
  Serial.print(sample.green);
  Serial.print(F(" B="));
  Serial.print(sample.blue);
  Serial.print(F(" class="));
  Serial.println(sample.label == -2 ? "SATURATED" :
                 sample.label >= 0 ? kColorNames[sample.label] : "UNKNOWN");
}

struct GeometryResult {
  float bestScore;
  float secondScore;
  float pitchMm;
  uint8_t whiteSlot;
  uint8_t picks[5];
};

float windowCenterMm(const ColorWindow& window) {
  return (window.startMm + window.endMm) * 0.5f;
}

void scoreGeometry(const ColorWindow* windows, uint8_t count,
                   const uint8_t picks[5],
                   bool firstWhiteCandidate, GeometryResult& result) {
  float gapMm[4];
  for (uint8_t i = 0; i < 4; ++i)
    gapMm[i] = windowCenterMm(windows[picks[i + 1]]) -
               windowCenterMm(windows[picks[i]]);
  for (uint8_t whiteSlot = 0; whiteSlot < 6; ++whiteSlot) {
    if (kWhiteFixedLast && whiteSlot != 5) continue;
    if (whiteSlot == 0 && !firstWhiteCandidate) continue;
    float weightedGap = 0.0f;
    float weightSquared = 0.0f;
    for (uint8_t i = 0; i < 4; ++i) {
      const float pitchCount = whiteSlot == i + 1 ? 2.0f : 1.0f;
      weightedGap += pitchCount * gapMm[i];
      weightSquared += pitchCount * pitchCount;
    }
    const float pitchMm = weightedGap / weightSquared;
    if (pitchMm < 45.0f || pitchMm > 105.0f) continue;
    const float firstRatio = windowCenterMm(windows[picks[0]]) / pitchMm;
    // With WHITE first, the first visible color should be about one band
    // pitch from the scan start. With WHITE last, it should appear near the
    // start. Leave 0.70..0.80 unresolved instead of guessing an endpoint.
    if (whiteSlot == 0 && (firstRatio < 0.80f || firstRatio > 1.55f))
      continue;
    if (whiteSlot == 5 && !kWhiteFixedLast && firstRatio > 0.70f)
      continue;
    if (whiteSlot > 0 && whiteSlot < 5 && firstRatio > 1.10f) continue;
    float score = 0.0f;
    bool plausible = true;
    for (uint8_t i = 0; i < 4; ++i) {
      const float pitchCount = whiteSlot == i + 1 ? 2.0f : 1.0f;
      const float error =
          (gapMm[i] - pitchCount * pitchMm) / (pitchCount * pitchMm);
      if (fabsf(error) > 0.35f) {
        plausible = false;
        break;
      }
      score += error * error;
    }
    if (!plausible) continue;
    if (whiteSlot == 0) {
      const float error = firstRatio - 1.0f;
      score += 0.1f * error * error;
    } else if (whiteSlot == 5 && !kWhiteFixedLast) {
      // A WHITE floor before band 1 can masquerade as a WHITE first band.
      // Treat first/last as ambiguous when neither starting offset is clear.
      score += 0.1f * firstRatio * firstRatio;
    }
    for (uint8_t i = 0; i < 5; ++i) {
      const ColorWindow& selected = windows[picks[i]];
      if (selected.samples == 1) score += 0.02f;
      // A short duplicate can fit two different WHITE slots equally well.
      // Prefer the better observed window of that color, but let a much
      // better geometric fit override this evidence.
      uint8_t strongestSamples = selected.samples;
      for (uint8_t j = 0; j < count; ++j)
        if (windows[j].color == selected.color &&
            windows[j].samples > strongestSamples)
          strongestSamples = windows[j].samples;
      const uint8_t sampleDeficit = strongestSamples - selected.samples;
      // The 30 ms polls can repeat one 50 ms conversion. Prefer the longer
      // observed window when two positions fit the same 7 cm band spacing.
      score += 0.030f * (sampleDeficit > 6 ? 6 : sampleDeficit);
    }
    bool sameOrder = whiteSlot == result.whiteSlot;
    if (sameOrder)
      for (uint8_t i = 0; i < 5; ++i)
        if (windows[picks[i]].color != windows[result.picks[i]].color)
          sameOrder = false;
    if (score < result.bestScore) {
      if (!sameOrder) result.secondScore = result.bestScore;
      result.bestScore = score;
      result.pitchMm = pitchMm;
      result.whiteSlot = whiteSlot;
      for (uint8_t i = 0; i < 5; ++i) result.picks[i] = picks[i];
    } else if (!sameOrder && score < result.secondScore) {
      result.secondScore = score;
    }
  }
}

void searchGeometry(const ColorWindow* windows, uint8_t count,
                    bool firstWhiteCandidate, uint8_t depth,
                    uint8_t nextIndex, uint8_t usedMask, uint8_t picks[5],
                    GeometryResult& result) {
  if (depth == 5) {
    scoreGeometry(windows, count, picks, firstWhiteCandidate, result);
    return;
  }
  for (uint8_t i = nextIndex; i < count; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1u << windows[i].color);
    if (usedMask & bit) continue;
    picks[depth] = i;
    searchGeometry(windows, count, firstWhiteCandidate, depth + 1,
                   i + 1, usedMask | bit, picks, result);
  }
}

bool inferOrderFromWindows(const ColorWindow* observed, uint8_t observedCount,
                           int8_t first, uint8_t order[6]) {
  ColorWindow windows[kMaxColorWindows + 1] = {};
  uint8_t count = 0;
  if ((first >= 0 && first < 4) || first == 5) {
    windows[count].color = first;
    windows[count].samples = 3;
    ++count;
  }
  for (uint8_t i = 0; i < observedCount; ++i) {
    if (observed[i].color < 0 || observed[i].color == 4) continue;
    windows[count++] = observed[i];
  }
  if (count < 5) return false;
  GeometryResult result = {1000.0f, 1000.0f, 0.0f, 255, {}};
  uint8_t picks[5] = {};
  searchGeometry(windows, count, first == 4, 0, 0, 0, picks, result);
  if (result.whiteSlot == 255 || result.bestScore > 0.20f ||
      result.secondScore < result.bestScore + 0.05f) {
    Serial.print(F("SCAN: geometry unresolved best="));
    Serial.print(result.bestScore, 3);
    Serial.print(F(" second="));
    Serial.print(result.secondScore, 3);
    Serial.print(F(" white band="));
    if (result.whiteSlot == 255) Serial.println(F("none"));
    else Serial.println(result.whiteSlot + 1);
    return false;
  }
  uint8_t visible = 0;
  for (uint8_t slot = 0; slot < 6; ++slot)
    order[slot] = slot == result.whiteSlot ? 4 :
        static_cast<uint8_t>(windows[result.picks[visible++]].color);
  Serial.print(kWhiteFixedLast ? F("SCAN: fixed WHITE band=") :
               F("SCAN: distance reconstruction WHITE band="));
  Serial.print(result.whiteSlot + 1);
  Serial.print(F(" pitch="));
  Serial.print(result.pitchMm * 0.1f, 1);
  Serial.print(F(" cm score="));
  Serial.println(result.bestScore, 3);
  for (uint8_t i = 0; i < 5; ++i) {
    const ColorWindow& window = windows[result.picks[i]];
    Serial.print(F("SCAN: selected "));
    Serial.print(kColorNames[window.color]);
    Serial.print(F(" at "));
    Serial.print(windowCenterMm(window) * 0.1f, 1);
    Serial.print(F(" cm from "));
    Serial.print(window.samples);
    Serial.println(F(" sample(s)"));
  }
  return true;
}

int8_t colorIndex(const String& name) {
  for (uint8_t i = 0; i < 6; ++i) {
    if (name == kColorNames[i]) return static_cast<int8_t>(i);
  }
  return -1;
}

int8_t scanColorFromTcs(uint16_t whiteClearReference = 0) {
  // The old 44032 reading was saturation. Keep WHITE as background here and
  // infer its band position from spacing after five visible colors are found.
  const uint32_t r = tcs0.r;
  const uint32_t g = tcs0.g;
  const uint32_t b = tcs0.b;
  // Red can be darker than BLACK in the clear channel on this course.
  // At C=788..1013 its R channel was 3-4 times G/B; never label that BLACK.
  const uint32_t redMaxClear = whiteClearReference >= 8000u ?
      static_cast<uint32_t>(whiteClearReference) * 55u / 100u : 10000u;
  if (tcs0.c > 200u && tcs0.c <= redMaxClear &&
      r * 100u >= g * 118u && r * 100u >= b * 150u) return 0;
  if (tcs0.c <= kBlackClearMax) return 5;
  // A moving 2 cm black stripe can blend with the bright floor during the
  // 50 ms integration. Its neutral RGB ratios stay floor-like, but C drops.
  if (whiteClearReference >= 8000u &&
      static_cast<uint32_t>(tcs0.c) * 100u <=
          static_cast<uint32_t>(whiteClearReference) * 35u &&
      r * 100u >= g * 80u && r * 100u <= g * 115u &&
      b * 100u >= g * 60u && b * 100u <= g * 90u) return 5;
  const int8_t named = colorIndex(tcs0.getColor());
  if (named >= 0 && named < 4) {
    // Yellow and the white floor shared saturated C. A dim edge at 37% of
    // the floor reference was a false YELLOW near the WHITE band; the real
    // YELLOW windows on this course were at least about 43% of that floor.
    // Also reject a loose fallback when blue is too strong for yellow.
    if (named != 2) return named;
    const uint32_t yellowMinClear = whiteClearReference >= 8000u ?
        static_cast<uint32_t>(whiteClearReference) * 42u / 100u : 5000u;
    if (tcs0.c >= yellowMinClear &&
        tcs0.b <= 0.40f * (tcs0.r + tcs0.g)) return named;
  }
  // A mixed edge frame may be UNKNOWN to the calibrated classifier while its
  // strongest channel still identifies the band. Check chroma before using
  // clear brightness as a WHITE/floor fallback (BLUE C=5306..5815 on track).
  if (r * 100u >= g * 125u && r * 100u >= b * 125u) return 0;
  if (g * 100u >= r * 135u && g * 100u >= b * 112u) return 1;
  if (b * 100u >= r * 108u && b * 100u >= g * 90u) return 3;
  if (tcs0.c >= kWhiteClearMin) return 4;
  return -1;
}

// Pick the narrow black group closest to the previously tracked line. A wide
// patch or a white gap does not provide a trustworthy steering position.
bool scanLinePosition(float previous, float& position) {
  const CalibrationData& cal = calibrationManager.data();
  const uint16_t* raw = sensorArrays.frontFiltered();
  float bestDistance = 1000.0f;
  bool found = false;
  for (uint8_t i = 0; i < 16;) {
    uint16_t strength[16] = {};
    uint8_t start = i;
    for (; i < 16; ++i) {
      const int32_t range = static_cast<int32_t>(cal.frontMax[i]) - cal.frontMin[i];
      if (range <= 0) return false;
      const int32_t mapped = constrain(
          (static_cast<int32_t>(raw[i]) - cal.frontMin[i]) * 1000L / range,
          0L, 1000L);
      if (mapped >= 500) break;
      strength[i] = static_cast<uint16_t>(1000 - mapped);
    }
    const uint8_t end = i;
    if (end > start && end - start <= 5) {
      int32_t sumWeight = 0;
      int32_t sumStrength = 0;
      for (uint8_t j = start; j < end; ++j) {
        sumWeight += static_cast<int32_t>(kLineWeights[j]) * strength[j];
        sumStrength += strength[j];
      }
      const float center = static_cast<float>(sumWeight) / sumStrength;
      const float distance = fabsf(center - previous);
      if (distance < bestDistance) {
        position = center;
        bestDistance = distance;
        found = true;
      }
    }
    if (i == start) ++i; // Skip the white channel separating black groups.
  }
  return found;
}

void driveScanPid(float position, float& previousError, float& integral,
                  uint32_t& previousUs, int leftSpeed, int rightSpeed,
                  float kp, float kd, int& appliedLeft, int& appliedRight) {
  const uint32_t nowUs = micros();
  const float dt = constrain((nowUs - previousUs) * 0.000001f, 0.001f, 0.05f);
  previousUs = nowUs;
  const float error = position - kTargetPosition;
  integral = constrain(integral + error * dt, -100.0f, 100.0f);
  const float maxCorrection =
      min(50.0f, 0.7f * static_cast<float>(min(leftSpeed, rightSpeed)));
  const float correction = constrain(
      kp * error + kScanKi * integral + kd * (error - previousError) / dt,
      -maxCorrection, maxCorrection);
  previousError = error;
  int left = constrain(static_cast<int>(lroundf(leftSpeed + correction)), 2, 100);
  int right = constrain(static_cast<int>(lroundf(rightSpeed - correction)), 2, 100);
  motor(left, right);
  appliedLeft = left;
  appliedRight = right;
}

// Read while stationary over the middle of band 1. A WHITE reading is only a
// candidate; its band position is checked again after five visible colors.
bool readFirstBand(int8_t& first) {
  first = -1;
  uint8_t stable = 0;
  bool saturationReported = false;
  for (uint8_t attempt = 0; attempt < 8; ++attempt) {
    delay(kFirstBandSampleMs);
    if (!tcs0.readFast()) continue;
    const bool saturated = tcs0.c >= kSaturatedClear;
    if (saturated && !saturationReported) {
      saturationReported = true;
      Serial.println(F("SCAN: first-band sample saturated; WHITE is only a candidate"));
    }
    const int8_t sample = saturated ? 4 : scanColorFromTcs();
    if (sample < 0) continue;
    if (sample == first) ++stable;
    else {
      first = sample;
      stable = 1;
    }
    if (stable >= 2) return true;
  }
  return false;
}

// Decide WHITE only after all five visible colors have been observed. Try
// each possible 7/14 cm interval and choose a clearly smoother speed profile.
// This allows acceleration while rejecting a guess when timing is ambiguous.
bool inferWhiteSlotFromSpacing(const uint32_t times[5], uint32_t startMs,
                               bool firstWhiteCandidate, uint8_t& slot) {
  uint32_t gaps[4];
  uint32_t sorted[4];
  for (uint8_t i = 0; i < 4; ++i) {
    gaps[i] = times[i + 1] - times[i];
    if (gaps[i] == 0) return false;
    sorted[i] = gaps[i];
  }
  for (uint8_t i = 0; i < 4; ++i)
    for (uint8_t j = i + 1; j < 4; ++j)
      if (sorted[j] < sorted[i]) {
        const uint32_t temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
  const float pitchMs = (sorted[1] + sorted[2]) * 0.5f;
  if (pitchMs <= 0.0f) return false;
  if (firstWhiteCandidate) {
    const float firstDelayRatio = (times[0] - startMs) / pitchMs;
    if (firstDelayRatio < 0.55f || firstDelayRatio > 1.8f) return false;
    float previousSpeed = kBandPitchCm * 1000.0f /
                          static_cast<float>(times[0] - startMs);
    float score = 0.0f;
    for (uint8_t i = 0; i < 4; ++i) {
      const float speed = kBandPitchCm * 1000.0f / gaps[i];
      const float change = logf(speed / previousSpeed);
      score += change * change;
      previousSpeed = speed;
    }
    Serial.print(F("SCAN: WHITE-first timing score="));
    Serial.println(score, 3);
    if (score > 1.0f) return false;
    slot = 0;
    return true;
  }

  float bestScore = 1000.0f;
  float secondScore = 1000.0f;
  uint8_t bestSlot = 255;
  for (uint8_t candidate = 1; candidate <= 5; ++candidate) {
    float previousSpeed = 0.0f;
    float score = 0.0f;
    bool implausible = false;
    for (uint8_t i = 0; i < 4; ++i) {
      const float distanceCm = candidate == i + 1 ?
          2.0f * kBandPitchCm : kBandPitchCm;
      const float speed = distanceCm * 1000.0f / gaps[i];
      if (i > 0) {
        const float ratio = speed > previousSpeed ?
            speed / previousSpeed : previousSpeed / speed;
        if (ratio > 2.5f) implausible = true;
        const float change = logf(speed / previousSpeed);
        score += change * change;
      }
      previousSpeed = speed;
    }
    if (implausible) continue;
    if (score < bestScore) {
      secondScore = bestScore;
      bestScore = score;
      bestSlot = candidate;
    } else if (score < secondScore) {
      secondScore = score;
    }
  }
  Serial.print(F("SCAN: WHITE timing best band="));
  Serial.print(bestSlot == 255 ? 0 : bestSlot + 1);
  Serial.print(F(" score="));
  Serial.print(bestScore, 3);
  Serial.print(F(" next="));
  Serial.println(secondScore, 3);
  if (bestSlot == 255 || bestScore > 0.8f ||
      bestScore > 0.65f * secondScore) return false;
  slot = bestSlot;
  return true;
}
} // namespace

uint8_t scannedColorOrder[6] = {};
uint8_t scannedColorCount = 0;

bool scanSixColorBands(int leftSpeed, int rightSpeed, float kp, float kd,
                       float maxDistanceCm) {
  scannedColorCount = 0;
  for (uint8_t i = 0; i < 6; ++i) order_color[i] = "";
  if (leftSpeed < 2 || leftSpeed > 100 || rightSpeed < 2 || rightSpeed > 100 ||
      !(kp >= 0.0f && kp <= 10.0f) || !(kd >= 0.0f && kd <= 10.0f) ||
      !(maxDistanceCm > 0.0f && maxDistanceCm <= 100.0f)) {
    Serial.println(F("SCAN: speed 2..100, Kp/Kd 0..10, distance >0..100 cm required"));
    motor(1, 1);
    return false;
  }
  if (!scanColorSensorReady || !calibrationManager.frontValid() ||
      !tcs0.allColorsCalibrated()) {
    Serial.println(F("SCAN: TCS setup, front line, or new color calibration missing"));
    motor(1, 1);
    return false;
  }

  motor(1, 1);
  int8_t first = -1;
  if (!readFirstBand(first)) {
    Serial.println(F("SCAN: cannot read band 1; place TCS at its center"));
    return false;
  }
  const bool firstWhiteCandidate = first == 4;
  uint16_t whiteClearReference = firstWhiteCandidate ? tcs0.c : 0u;
  if (!firstWhiteCandidate) {
    scannedColorOrder[0] = static_cast<uint8_t>(first);
    scannedColorCount = 1;
  }
  uint32_t startMs = millis();
  uint32_t lastLineMs = startMs;
  uint32_t nextColorMs = startMs;
  uint32_t previousPidUs = micros();
  float lastPosition = kTargetPosition;
  float previousError = 0.0f;
  float integral = 0.0f;
  uint8_t alignedFrames = 0;
  bool aligned = false;
  bool armed = false;
  uint8_t seenMask = firstWhiteCandidate ? 0u : static_cast<uint8_t>(1u << first);
  uint8_t duplicateLoggedMask = 0u;
  ColorWindow colorWindows[kMaxColorWindows] = {};
  TailColorSample earlySamples[kEarlySampleCount] = {};
  uint8_t earlyCount = 0;
  TailColorSample tailSamples[kTailSampleCount] = {};
  uint8_t tailNext = 0;
  uint8_t tailCount = 0;
  uint16_t scanReadFailures = 0;
  uint16_t tailReadFailures = 0;
  uint16_t mixedChannelSamples = 0;
  bool previousTcsValid = false;
  uint16_t previousTcsC = 0;
  uint16_t previousTcsR = 0;
  uint16_t previousTcsG = 0;
  uint16_t previousTcsB = 0;
  uint16_t saturatedSamples = 0;
  float firstSaturatedMm = -1.0f;
  float lastSaturatedMm = -1.0f;
  uint8_t colorWindowCount = 0;
  uint8_t droppedColorWindows = 0;
  int8_t candidateColor = -1;
  uint8_t candidateSamples = 0;
  uint8_t candidateStatus = 0;
  uint32_t candidateStartMs = 0;
  uint32_t candidateLastMs = 0;
  float candidateStartMm = 0.0f;
  float candidateLastMm = 0.0f;
  auto finishColorWindow = [&]() {
    if (candidateColor < 0) return;
    if (colorWindowCount < kMaxColorWindows) {
      ColorWindow& window = colorWindows[colorWindowCount++];
      window.color = candidateColor;
      window.samples = candidateSamples;
      window.status = candidateStatus;
      window.startMs = candidateStartMs;
      window.endMs = candidateLastMs;
      window.startMm = candidateStartMm;
      window.endMm = candidateLastMm;
    } else if (droppedColorWindows < 255) {
      ++droppedColorWindows;
    }
    candidateColor = -1;
  };
  uint32_t previousBandMs = startMs;
  uint32_t previousDistanceUs = micros();
  float estimatedDistanceMm = 0.0f;
  float lastAcceptedDistanceMm = 0.0f;
  float previousBandDistanceMm = 0.0f;
  const float targetDistanceMm = maxDistanceCm * 10.0f;
  const float distanceScale = get_line_distance_scale();
  int appliedLeft = 0;
  int appliedRight = 0;
  uint32_t observedBandMs[5] = {};
  if (!firstWhiteCandidate) observedBandMs[0] = startMs;
  bool speedWarningPrinted = false;

  Serial.print(F("SCAN: band 1 sample = "));
  Serial.print(kColorNames[first]);
  Serial.print(F(" C="));
  Serial.println(tcs0.c);
  if (firstWhiteCandidate)
    Serial.println(kWhiteFixedLast ?
        F("SCAN: initial WHITE may be floor; WHITE is fixed at band 6") :
        F("SCAN: WHITE at band 1 is only a candidate until all five colors are seen"));
  Serial.print(F("SCAN: motor L/R="));
  Serial.print(leftSpeed);
  Serial.print('/');
  Serial.print(rightSpeed);
  Serial.print(F(" Kp="));
  Serial.print(kp, 3);
  Serial.print(F(" Kd="));
  Serial.println(kd, 4);
  Serial.print(F("SCAN: first center about "));
  Serial.print(kFirstCenterFromStartCm, 0);
  Serial.print(F(" cm from START if 17 cm marks the near edge; next "));
  Serial.print(kFirstToLastCenterCm, 0);
  Serial.print(F(" cm at line position "));
  Serial.println(kLinePosition);
  Serial.print(F("SCAN: distance limit="));
  Serial.print(maxDistanceCm, 1);
  Serial.print(F(" cm; model scale="));
  Serial.println(distanceScale, 2);
  // Start the distance timing after Serial diagnostics, just before motion.
  startMs = millis();
  lastLineMs = startMs;
  nextColorMs = startMs;
  previousPidUs = micros();
  previousDistanceUs = previousPidUs;
  previousBandMs = startMs;
  if (!firstWhiteCandidate) observedBandMs[0] = startMs;
  while (true) {
    const uint32_t nowMs = millis();
    const uint32_t nowUs = micros();
    const uint32_t elapsedUs = nowUs - previousDistanceUs;
    previousDistanceUs = nowUs;
    const float forwardCommand = max(0.0f,
        (static_cast<float>(appliedLeft) + appliedRight) * 0.5f);
    estimatedDistanceMm += LINE_MM_PER_SECOND_AT_100 * distanceScale *
        (forwardCommand / 100.0f) * (elapsedUs * 0.000001f);
    if (!armed && estimatedDistanceMm - lastAcceptedDistanceMm >= kRearmDistanceMm)
      armed = true;
    if (estimatedDistanceMm >= targetDistanceMm) {
      motor(1, 1);
      Serial.print(F("SCAN: distance limit reached at estimated "));
      Serial.print(estimatedDistanceMm * 0.1f, 1);
      Serial.println(F(" cm"));
      break;
    }
    if (nowMs - startMs > kScanTimeoutMs ||
        (!aligned && nowMs - startMs > kAlignTimeoutMs)) {
      motor(1, 1);
      Serial.println(F("SCAN: timeout"));
      break;
    }

    if (sensorArrays.update(micros())) {
      float position = lastPosition;
      if (scanLinePosition(lastPosition, position)) {
        lastLineMs = nowMs;
        lastPosition = position;
        driveScanPid(position, previousError, integral, previousPidUs,
                     leftSpeed, rightSpeed, kp, kd, appliedLeft, appliedRight);
        if (!aligned) {
          if (fabsf(position - kTargetPosition) <= 7.0f) {
            if (++alignedFrames >= 3) aligned = true;
          } else {
            alignedFrames = 0;
          }
        }
      } else {
        integral = 0.0f;
        previousError = 0.0f;
        previousPidUs = micros();
        motor(leftSpeed, rightSpeed); // Cross one short white/wide band.
        appliedLeft = leftSpeed;
        appliedRight = rightSpeed;
      }
    }
    if (nowMs - lastLineMs > kLineGapTimeoutMs) {
      motor(1, 1);
      Serial.println(F("SCAN: guide line lost"));
      break;
    }
    if (!aligned || nowMs < nextColorMs) continue;
    nextColorMs = nowMs + kColorPollMs;
    if (!tcs0.readFast()) {
      if (scanReadFailures < 65535) ++scanReadFailures;
      if (estimatedDistanceMm >= targetDistanceMm - 150.0f &&
          tailReadFailures < 65535) ++tailReadFailures;
      continue;
    }
    // readFast() fetches C, R, G and B in four I2C transactions. Around a
    // conversion boundary, C can still be the old frame while one or more
    // RGB channels have changed. Such frames produced a false YELLOW on
    // WHITE tape and a BLUE reading with the old WHITE clear value.
    const bool mixedChannels = previousTcsValid &&
        tcs0.c < kSaturatedClear && previousTcsC < kSaturatedClear &&
        tcs0.c == previousTcsC &&
        (static_cast<uint32_t>(tcs0.r) * 5u <
             static_cast<uint32_t>(previousTcsR) * 4u ||
         static_cast<uint32_t>(tcs0.r) * 5u >
             static_cast<uint32_t>(previousTcsR) * 6u ||
         static_cast<uint32_t>(tcs0.g) * 5u <
             static_cast<uint32_t>(previousTcsG) * 4u ||
         static_cast<uint32_t>(tcs0.g) * 5u >
             static_cast<uint32_t>(previousTcsG) * 6u ||
         static_cast<uint32_t>(tcs0.b) * 5u <
             static_cast<uint32_t>(previousTcsB) * 4u ||
         static_cast<uint32_t>(tcs0.b) * 5u >
             static_cast<uint32_t>(previousTcsB) * 6u);
    if (mixedChannels) {
      if (mixedChannelSamples < 65535) ++mixedChannelSamples;
      continue;
    }
    previousTcsValid = true;
    previousTcsC = tcs0.c;
    previousTcsR = tcs0.r;
    previousTcsG = tcs0.g;
    previousTcsB = tcs0.b;
    const bool saturated = tcs0.c >= kSaturatedClear;
    if (saturated) {
      if (saturatedSamples < 65535) ++saturatedSamples;
      if (firstSaturatedMm < 0.0f) firstSaturatedMm = estimatedDistanceMm;
      lastSaturatedMm = estimatedDistanceMm;
    }

    // A bright white stripe can saturate a frame. Ignore that color reading,
    // but keep following the line and enforcing the distance limit.
    const int8_t measured = saturated ? -2 :
        scanColorFromTcs(whiteClearReference);
    if (measured == 4 && tcs0.c > whiteClearReference)
      whiteClearReference = tcs0.c;
    const TailColorSample sample = {
        static_cast<uint16_t>(estimatedDistanceMm), tcs0.c,
        tcs0.r, tcs0.g, tcs0.b, measured};
    if (estimatedDistanceMm < targetDistanceMm - 150.0f &&
        earlyCount < kEarlySampleCount)
      earlySamples[earlyCount++] = sample;
    if (estimatedDistanceMm >= targetDistanceMm - 150.0f) {
      tailSamples[tailNext] = sample;
      tailNext = (tailNext + 1u) % kTailSampleCount;
      if (tailCount < kTailSampleCount) ++tailCount;
    }
    int8_t band = measured;
    if (measured == 4 || saturated) band = -1; // WHITE/floor cannot be separated.

    if (candidateColor >= 0 &&
        (band >= 0 && band != candidateColor ||
         nowMs - candidateLastMs > kColorGapToleranceMs))
      finishColorWindow();
    if (band < 0) continue;
    if (candidateColor < 0) {
      if (!armed) continue;
      candidateColor = band;
      candidateSamples = 1;
      candidateStatus = 0;
      candidateStartMs = nowMs;
      candidateStartMm = estimatedDistanceMm;
    } else if (candidateSamples < 255) {
      ++candidateSamples;
    }
    candidateLastMs = nowMs;
    candidateLastMm = estimatedDistanceMm;
    if (candidateStatus != 0 || candidateSamples < kMinBandSamples ||
        nowMs - candidateStartMs < kMinBandDwellMs) continue;
    const uint8_t bit = static_cast<uint8_t>(1u << band);
    if (seenMask & bit) {
      candidateStatus = 2;
      if (!(duplicateLoggedMask & bit)) {
        duplicateLoggedMask |= bit;
        Serial.print(F("SCAN: duplicate ignored: "));
        Serial.println(kColorNames[band]);
      }
      continue;
    }
    candidateStatus = 1;
    armed = false;
    lastAcceptedDistanceMm = estimatedDistanceMm;
    seenMask |= bit;
    observedBandMs[scannedColorCount] =
        candidateStartMs + (nowMs - candidateStartMs) / 2;
    scannedColorOrder[scannedColorCount++] = static_cast<uint8_t>(band);
    Serial.print(F("SCAN: visible reading "));
    Serial.print(scannedColorCount);
    Serial.print(F(" = "));
    Serial.print(kColorNames[band]);
    Serial.print(F(" C="));
    Serial.print(tcs0.c);
    Serial.print(F(" distance="));
    Serial.print(estimatedDistanceMm * 0.1f, 1);
    Serial.println(F(" cm"));
    if (previousBandMs != 0 && nowMs > previousBandMs) {
      // The model covers skipped bands too; assuming one 7 cm pitch would
      // under-report speed when a colored band was missed.
      const float modelSpeed =
          (estimatedDistanceMm - previousBandDistanceMm) * 100.0f /
          static_cast<float>(nowMs - previousBandMs);
      Serial.print(F("SCAN: estimated speed "));
      Serial.print(modelSpeed, 1);
      Serial.println(F(" cm/s from distance model"));
      if (modelSpeed > kRecommendedMaxCmPerSec && !speedWarningPrinted) {
        speedWarningPrinted = true;
        Serial.print(F("SCAN: may miss 2 cm bands; aim below "));
        Serial.print(kRecommendedMaxCmPerSec, 1);
        Serial.println(F(" cm/s"));
      }
    }
    previousBandMs = nowMs;
    previousBandDistanceMm = estimatedDistanceMm;
  }

  motor(1, 1);
  finishColorWindow();
  Serial.print(F("SCAN: stopped; estimated travel="));
  Serial.print(estimatedDistanceMm * 0.1f, 1);
  Serial.println(F(" cm"));
  Serial.print(F("SCAN: elapsed="));
  Serial.print(millis() - startMs);
  Serial.println(F(" ms"));
  Serial.print(F("SCAN: saturated samples="));
  Serial.print(saturatedSamples);
  if (saturatedSamples) {
    Serial.print(F(" from "));
    Serial.print(firstSaturatedMm * 0.1f, 1);
    Serial.print(F(" to "));
    Serial.print(lastSaturatedMm * 0.1f, 1);
    Serial.print(F(" cm"));
  }
  Serial.println();
  Serial.print(F("SCAN: bright floor C reference="));
  Serial.println(whiteClearReference);
  Serial.print(F("SCAN: mixed-channel samples ignored="));
  Serial.println(mixedChannelSamples);
  Serial.print(F("SCAN: early TCS samples="));
  Serial.print(earlyCount);
  Serial.print(F(" total read failures="));
  Serial.println(scanReadFailures);
  for (uint8_t i = 0; i < earlyCount; ++i)
    printColorSample(F("SCAN: early d="), earlySamples[i]);
  Serial.print(F("SCAN: final 15 cm TCS samples="));
  Serial.print(tailCount);
  Serial.print(F(" read failures="));
  Serial.println(tailReadFailures);
  for (uint8_t i = 0; i < tailCount; ++i) {
    const uint8_t index =
        (tailNext + kTailSampleCount - tailCount + i) % kTailSampleCount;
    printColorSample(F("SCAN: tail d="), tailSamples[index]);
  }
  for (uint8_t i = 0; i < colorWindowCount; ++i) {
    const ColorWindow& window = colorWindows[i];
    Serial.print(F("SCAN: color window "));
    Serial.print(kColorNames[window.color]);
    Serial.print(F(" duration="));
    Serial.print(window.endMs - window.startMs);
    Serial.print(F(" ms samples="));
    Serial.print(window.samples);
    Serial.print(F(" distance="));
    Serial.print(window.startMm * 0.1f, 1);
    Serial.print('-');
    Serial.print(window.endMm * 0.1f, 1);
    Serial.print(F(" cm "));
    Serial.println(window.status == 1 ? F("accepted") :
                   window.status == 2 ? F("duplicate") : F("brief transition"));
  }
  if (droppedColorWindows) {
    Serial.print(F("SCAN: color windows omitted="));
    Serial.println(droppedColorWindows);
  }
  uint8_t reconstructedOrder[6] = {};
  if (inferOrderFromWindows(colorWindows, colorWindowCount, first,
                            reconstructedOrder)) {
    for (uint8_t i = 0; i < 6; ++i)
      scannedColorOrder[i] = reconstructedOrder[i];
    scannedColorCount = 6;
  } else if (scannedColorCount == 5) {
    if (kWhiteFixedLast && seenMask == 0x2f) {
      scannedColorOrder[5] = 4;
      scannedColorCount = 6;
      Serial.println(F("SCAN: WHITE fixed at band 6 after five visible colors"));
    } else if (!kWhiteFixedLast) {
      Serial.println(F("SCAN: WHITE position unresolved; center TCS on band 1 before scanning"));
    }
  }
  Serial.print(F("SCAN: "));
  Serial.print(scannedColorCount);
  Serial.print(F("/6; order: "));
  for (uint8_t i = 0; i < scannedColorCount; ++i) {
    if (i) Serial.print(F(", "));
    Serial.print(kColorNames[scannedColorOrder[i]]);
  }
  Serial.println();
  if (scannedColorCount == 6) {
    for (uint8_t i = 0; i < 6; ++i)
      order_color[i] = kColorNames[scannedColorOrder[i]];
  }
  return scannedColorCount == 6;
}
