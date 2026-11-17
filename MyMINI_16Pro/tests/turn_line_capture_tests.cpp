#include "LineTurnCapture.h"

#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <limits>

static unsigned checks = 0;
#define CHECK(value) do { ++checks; if (!(value)) { \
  std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #value); \
  std::exit(1); } } while (0)

static bool near(float actual, float expected) {
  return fabsf(actual - expected) < 0.0001f;
}

static void handoffTests() {
  TurnLineHandoff handoff;
  CHECK(!handoff.consume(1u, 0u, true));
  handoff.arm(42u, 1000u);
  CHECK(handoff.consume(42u, 2000u, true));
  CHECK(!handoff.consume(42u, 2000u, true));
  // A stop or manual drive increments the sequence, even for identical values.
  handoff.arm(42u, 1000u);
  CHECK(!handoff.consume(43u, 2000u, true));
  handoff.arm(42u, 1000u);
  CHECK(!handoff.consume(42u, 2000u, false));
  CHECK(!handoff.consume(42u, 2000u, true));
  handoff.arm(42u, 1000u);
  CHECK(handoff.consume(42u, 251000u, true));
  handoff.arm(42u, 1000u);
  CHECK(!handoff.consume(42u, 251001u, true));
  handoff.arm(42u, 1000u);
  handoff.clear();
  CHECK(!handoff.consume(42u, 2000u, true));
  handoff.arm(UINT32_MAX, UINT32_MAX - 1000u);
  CHECK(handoff.consume(UINT32_MAX, 1000u, true));
  handoff.arm(UINT32_MAX, UINT32_MAX - 1000u);
  CHECK(!handoff.consume(0u, 1000u, true));
}

static void adaptiveSpeedTests() {
  for (int sign : {-1, 1}) {
    LineTurnCapture centered(200), middle(200), outer(200), edge(200);
    CHECK(near(centered.update(10000u, true, sign * 10.0f), 0.80f));
    CHECK(near(middle.update(10000u, true, sign * 20.0f), 0.55f));
    CHECK(near(outer.update(10000u, true, sign * 30.0f), 0.30f));
    CHECK(near(edge.update(10000u, true, sign * 100.0f), 0.30f));
  }
  // A larger offset must never command a faster capture, on either side.
  float previous = 0.80f;
  for (int error = 0; error <= 100; ++error) {
    LineTurnCapture left(200), right(200);
    const float factor = right.update(10000u, true, float(error));
    CHECK(factor >= 0.2999f && factor <= 0.8001f);
    CHECK(factor <= previous + 0.0001f);
    CHECK(near(left.update(10000u, true, -float(error)), factor));
    previous = factor;
  }
  LineTurnCapture zeroRamp(0);
  CHECK(near(zeroRamp.update(1u, true, 50.0f), 1.0f));
  CHECK(near(zeroRamp.update(2u, false, 0.0f), 1.0f));
}

static void immediateRampTests() {
  LineTurnCapture capture(200);
  CHECK(near(capture.update(10000u, true, 0.0f), 0.80f));
  const float secondFrame = capture.update(20000u, true, 0.0f);
  CHECK(secondFrame > 0.80f && secondFrame < 0.90f);
  CHECK(near(capture.update(60000u, true, 0.0f), 0.90f));
  CHECK(near(capture.update(110000u, true, 0.0f), 1.0f));
  // Capture is one startup phase, not a new speed policy for later corners.
  CHECK(near(capture.update(120000u, true, 50.0f), 1.0f));

  // A rapid sweep through center no longer delays the ramp.
  LineTurnCapture fastCrossing(200);
  fastCrossing.update(10000u, true, -8.0f);
  const float fastSecond = fastCrossing.update(20000u, true, 0.0f);
  CHECK(fastSecond > 0.80f);
  CHECK(fastCrossing.update(30000u, true, 8.0f) > fastSecond);
  CHECK(near(fastCrossing.update(60000u, true, 8.0f), 0.90f));

  // No +/-10 centering requirement: an offset of 20 also starts immediately.
  LineTurnCapture offCenter(200);
  CHECK(near(offCenter.update(10000u, true, 20.0f), 0.55f));
  CHECK(near(offCenter.update(60000u, true, 20.0f), 0.775f));
  CHECK(near(offCenter.update(110000u, true, 20.0f), 1.0f));

  // A sensor-frame gap over 50 ms no longer restarts startup confirmation.
  LineTurnCapture gap(200);
  gap.update(10000u, true, 0.0f);
  CHECK(gap.update(80000u, true, 0.0f) > 0.90f);
  CHECK(near(gap.update(110000u, true, 0.0f), 1.0f));

  LineTurnCapture minimumRamp(1);
  minimumRamp.update(10000u, true, 0.0f);
  CHECK(near(minimumRamp.update(10250u, true, 0.0f), 0.90f));
  CHECK(near(minimumRamp.update(10500u, true, 0.0f), 1.0f));

  LineTurnCapture oddRamp(3);
  oddRamp.update(10000u, true, 0.0f);
  CHECK(near(oddRamp.update(10750u, true, 0.0f), 0.90f));
  CHECK(near(oddRamp.update(11500u, true, 0.0f), 1.0f));
}

static void lossAndRecaptureTests() {
  LineTurnCapture capture(200);
  capture.update(10000u, true, 0.0f);
  CHECK(near(capture.update(60000u, true, 15.0f), 0.90f));
  CHECK(near(capture.update(70000u, true, 30.0f), 0.30f));
  CHECK(capture.update(80000u, true, 20.0f) > 0.30f);
  CHECK(near(capture.update(120000u, true, 20.0f), 0.65f));
  CHECK(near(capture.update(130000u, false, 0.0f), 0.0f));
  CHECK(near(capture.update(140000u, true, 0.0f), 0.80f));
  CHECK(near(capture.update(190000u, true, 0.0f), 0.90f));
  CHECK(near(capture.update(200000u, true,
      std::numeric_limits<float>::quiet_NaN()), 0.0f));
  CHECK(near(capture.update(210000u, true, 20.0f), 0.55f));
  CHECK(near(capture.update(260000u, true, 20.0f), 0.775f));
  CHECK(near(capture.update(310000u, true, 20.0f), 1.0f));
}

static void wraparoundTests() {
  LineTurnCapture capture(200);
  const uint32_t start = UINT32_MAX - 25000u;
  capture.update(start, true, 0.0f);
  CHECK(near(capture.update(start + 50000u, true, 0.0f), 0.90f));
  CHECK(near(capture.update(start + 100000u, true, 0.0f), 1.0f));
}

int main() {
  handoffTests();
  adaptiveSpeedTests();
  immediateRampTests();
  lossAndRecaptureTests();
  wraparoundTests();
  std::printf("PASS: %u checks (handoff, capture, immediate ramp, loss, wraparound)\n", checks);
}
