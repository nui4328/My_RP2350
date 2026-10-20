"""Compile the line helpers and replay edge loss on both sides."""

from pathlib import Path
import subprocess
import tempfile


staged_source = Path(__file__).resolve().parent / "LineFollower.cpp"
source_path = staged_source if staged_source.exists() else (
    Path(__file__).resolve().parents[1] / "src" / "LineFollower.cpp"
)
source = source_path.read_text(encoding="utf-8")


def section(start: str, end: str) -> str:
    first = source.index(start)
    return source[first:source.index(end, first)]


assert "const RawLineSideObservation rawSide = observeRawLineSide(blackStrength);" in source
assert "const int8_t recoverySide = updateLineRecoveryDirection(" in source
assert "lineFound && snapTrackedErrorToVisibleLine(" in source
assert "const bool rawDominatesSelected = preferDominantRawLine(" in source
assert "outsideBlack > 0u && !rawDominatesSelected" in source
assert "correction = keepCorrectionTowardVisibleLine(" in source
assert "linePositionWithEdgeRecovery(" in source
assert "recoverGuardedRawEdge(" in source
assert "correction = preventOppositeLineRecovery(" in source

harness = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
using std::max;
using std::min;

namespace MyMINIConfig { constexpr uint8_t SENSOR_COUNT = 16; }
constexpr int8_t LINE_WEIGHT[16] = {
  -50, -43, -37, -30, -23, -17, -10, -3,
     3,  10,  17,  23,  30,  37,  43, 50
};
constexpr int16_t TRACK_WINDOW_RADIUS = 22;
constexpr uint8_t BRANCH_STABLE_FRAMES = 2u;
constexpr uint8_t BRANCH_RELEASE_FRAMES = 2u;
constexpr uint8_t BRANCH_MAX_MAIN_WIDTH = 4u;
constexpr uint8_t BRANCH_MIN_WING_BLACK = 2u;
constexpr uint32_t LINE_RECOVERY_SIDE_MAX_AGE_US = 250000UL;
__TRACK_SELECTOR__
__RECOVERY_SELECTOR__
__SIDE_HELPERS__
__BRANCH_HELPERS__
__HANDOFF_HELPER__

void fill(uint16_t (&row)[16], int first, int last) {
  for (auto& value : row) value = 0u;
  for (int i = first; i <= last; ++i) row[i] = 900u;
}

void testEdgeLoss(int side) {
  uint16_t row[16] = {};
  if (side > 0) fill(row, 14, 15);
  else fill(row, 0, 1);
  const RawLineSideObservation observed = observeRawLineSide(row);
  assert(observed.visibleSide == side && observed.edgeSide == side);
  LineRecoveryDirection direction;
  assert(updateLineRecoveryDirection(direction, observed, true, 1000u) == side);
  // A visible edge turns toward its measured position while black remains.
  assert(linePositionWithEdgeRecovery(
      true, true, false, 1.1f, true, observed.visibleSide,
      observed.visiblePosition, side, -side * 30.0f) ==
      observed.visiblePosition);
  assert(preventOppositeLineRecovery(
      -side * 10.0f, false, observed.visibleSide) == 0.0f);

  // The next frame is all-white. The last raw edge must win over an active
  // branch guard and any stale error on the opposite side.
  fill(row, 1, 0);
  assert(observeRawLineSide(row).edgeSide == 0);
  assert(updateLineRecoveryDirection(direction, observeRawLineSide(row),
                                     false, 2000u) == side);
  const float highPosition = linePositionWithEdgeRecovery(
      false, true, false, 1.1f, true, 0, 0.0f, direction.allWhiteSide,
      -side * 30.0f);
  assert(highPosition == side * 50.0f);
  const float highCorrection = 1.1f * highPosition;
  assert((side > 0 && highCorrection > 0.0f) ||
         (side < 0 && highCorrection < 0.0f));

  // At and below 0.95, all-white still uses the existing neutral recovery.
  for (float kp : {0.5f, 0.95f}) {
    assert(linePositionWithEdgeRecovery(
        false, true, false, kp, true, 0, 0.0f, direction.allWhiteSide,
        -side * 30.0f) == 0.0f);
    assert(preventOppositeLineRecovery(-side * 10.0f, false, side) == 0.0f);
    assert(preventOppositeLineRecovery(side * 10.0f, false, side) ==
           side * 10.0f);
  }
}

void testInnerSideLoss(int side) {
  uint16_t row[16] = {};
  if (side > 0) fill(row, 9, 14);
  else fill(row, 1, 6);
  const RawLineSideObservation observed = observeRawLineSide(row);
  assert(observed.visibleSide == side && observed.edgeSide == 0);
  assert(observed.visiblePosition * side >= 10.0f);
  LineRecoveryDirection direction;
  assert(updateLineRecoveryDirection(direction, observed, true, 1000u) == side);

  // This is the bend before f15/f0: it must steer now, not wait for the edge.
  const float visiblePosition = linePositionWithEdgeRecovery(
      true, true, false, 0.96f, true, observed.visibleSide,
      observed.visiblePosition, side, -side * 30.0f);
  assert(visiblePosition == observed.visiblePosition);
  const float correction = preventOppositeLineRecovery(
      0.96f * visiblePosition, false, side);
  assert(correction * side > 10.0f);
  // At base speed 40 the visible bend should turn, without reversing a wheel.
  assert(40.0f - fabsf(correction) > 0.0f);

  // If the next sample is all white before f15/f0, keep searching on that side.
  fill(row, 1, 0);
  assert(updateLineRecoveryDirection(direction, observeRawLineSide(row),
                                     false, 2000u) == side);
  assert(linePositionWithEdgeRecovery(
      false, true, false, 0.96f, true, 0, 0.0f, direction.allWhiteSide,
      -side * 30.0f) == side * 50.0f);
  assert(linePositionWithEdgeRecovery(
      false, true, false, 0.5f, true, 0, 0.0f, direction.allWhiteSide,
      -side * 30.0f) == 0.0f);
}

void testRightCrossingThroughAmbiguousFrame() {
  uint16_t row[16] = {};
  LineRecoveryDirection direction;
  fill(row, 1, 3);
  assert(updateLineRecoveryDirection(direction, observeRawLineSide(row),
                                     true, 1000u) == -1);
  fill(row, 9, 14);
  assert(updateLineRecoveryDirection(direction, observeRawLineSide(row),
                                     true, 2000u) == 1);
  // A diagonal crossing can briefly light sensors on both halves.
  fill(row, 7, 8);
  assert(observeRawLineSide(row).visibleSide == 0);
  assert(updateLineRecoveryDirection(direction, observeRawLineSide(row),
                                     true, 3000u) == 1);
  assert(linePositionWithEdgeRecovery(
      true, true, false, 0.96f, true, 0, 0.0f, 1, -30.0f) == 0.0f);
  fill(row, 1, 0);
  assert(updateLineRecoveryDirection(direction, observeRawLineSide(row),
                                     false, 4000u) == 1);
  const float position = linePositionWithEdgeRecovery(
      false, false, false, 0.96f, true, 0, 0.0f, direction.allWhiteSide,
      -30.0f);
  assert(position == 50.0f);
  assert(preventOppositeLineRecovery(0.96f * position, false,
                                     direction.allWhiteSide) > 0.0f);
  // The direction stays latched throughout this all-white interval.
  assert(updateLineRecoveryDirection(direction, observeRawLineSide(row),
                                     false, 500000u) == 1);
}

void testRightDominatesMixedFrame() {
  uint16_t row[16] = {};
  LineRecoveryDirection direction;
  fill(row, 1, 3);
  assert(updateLineRecoveryDirection(direction, observeRawLineSide(row),
                                     true, 1000u) == -1);
  fill(row, 9, 12);
  row[7] = 900u;
  const RawLineSideObservation observed = observeRawLineSide(row);
  assert(observed.visibleSide == 1);
  assert(observed.visiblePosition > 10.0f);
  assert(updateLineRecoveryDirection(direction, observed, true, 2000u) == 1);
  assert(linePositionWithEdgeRecovery(
      true, true, false, 0.96f, true, observed.visibleSide,
      observed.visiblePosition, 1, -30.0f) > 0.0f);
  fill(row, 1, 0);
  assert(updateLineRecoveryDirection(direction, observeRawLineSide(row),
                                     false, 3000u) == 1);
  assert(linePositionWithEdgeRecovery(
      false, false, false, 0.96f, true, 0, 0.0f, 1,
      -30.0f) == 50.0f);

  fill(row, 3, 6);
  row[8] = 900u;
  const RawLineSideObservation mirrored = observeRawLineSide(row);
  assert(mirrored.visibleSide == -1);
  assert(mirrored.visiblePosition < -10.0f);
}

void testOldDirectionExpires() {
  uint16_t row[16] = {};
  LineRecoveryDirection direction;
  fill(row, 9, 14);
  assert(updateLineRecoveryDirection(direction, observeRawLineSide(row),
                                     true, 1000u) == 1);
  fill(row, 7, 8);
  assert(updateLineRecoveryDirection(direction, observeRawLineSide(row),
                                     true, 300000u) == 0);
  fill(row, 1, 0);
  assert(updateLineRecoveryDirection(direction, observeRawLineSide(row),
                                     false, 301000u) == 0);
  assert(linePositionWithEdgeRecovery(
      false, false, false, 0.96f, true, 0, 0.0f, 0, -30.0f) == 0.0f);
}

void testTrackedErrorSnapsRight() {
  uint16_t row[16] = {};
  fill(row, 9, 11);
  const RawLineSideObservation observed = observeRawLineSide(row);
  float trackedError = -20.0f;
  assert(snapTrackedErrorToVisibleLine(trackedError,
                                      observed.visiblePosition, observed,
                                      true));
  assert(trackedError > 0.0f);
  assert(0.96f * trackedError > 0.0f);
  trackedError = -20.0f;
  assert(snapTrackedErrorToVisibleLine(trackedError, 3.0f,
                                      observed, true));
  assert(trackedError == 3.0f);
  trackedError = -20.0f;
  assert(!snapTrackedErrorToVisibleLine(trackedError, 3.0f,
                                       observed, false));
}

void testGuardKeepsOldOppositeGroup(int side) {
  uint16_t row[16] = {};
  uint16_t filtered[16] = {};
  LineBranchGuard guard;
  guard.stableFrames = 2u;
  guard.mainStart = side > 0 ? 5u : 9u;
  guard.mainEnd = side > 0 ? 6u : 10u;
  if (side > 0) {
    fill(row, 5, 6);
    for (int i = 9; i <= 14; ++i) row[i] = 900u;
  } else {
    fill(row, 9, 10);
    for (int i = 1; i <= 6; ++i) row[i] = 900u;
  }
  const RawLineSideObservation observed = observeRawLineSide(row);
  assert(observed.visibleSide == side);
  assert(observed.visibleCount == 6u);
  assert(observed.visiblePosition * side >= 15.0f);
  bool changed = false;
  assert(filterLineBranch(guard, row, filtered, changed));
  assert(changed && guard.blockedSide == side);
  float measured = -side * 13.5f;
  uint8_t outside = 0u;
  uint8_t start = 255u;
  uint8_t end = 255u;
  const bool found = selectTrackedGroup(filtered, measured, true,
      measured, outside, &start, &end);
  assert(found && measured * side < 0.0f);
  assert(preferDominantRawLine(0.96f, false, true, found,
                               measured, observed));
  assert(!preferDominantRawLine(0.95f, false, true, found,
                                measured, observed));
  assert(!preferDominantRawLine(0.96f, false, false, found,
                                measured, observed));
  const float position = linePositionWithEdgeRecovery(
      true, true, false, 0.96f, true, observed.visibleSide,
      observed.visiblePosition, side, -side * 13.5f);
  assert(position * side >= 15.0f);
  const float correction = 0.96f * position;
  const float left = 40.0f + correction;
  const float right = 40.0f - correction;
  assert((left - right) * side > 0.0f);
}

void testDerivativeCannotReverseVisibleTurn(int side) {
  uint16_t row[16] = {};
  if (side > 0) fill(row, 9, 14);
  else fill(row, 1, 6);
  const RawLineSideObservation observed = observeRawLineSide(row);
  const float opposingDerivativeCorrection = -side * 20.0f;
  const float proportional = side * 9.6f;
  assert(keepCorrectionTowardVisibleLine(opposingDerivativeCorrection,
      proportional, true, true, 0.96f, observed) == proportional);
  assert(keepCorrectionTowardVisibleLine(opposingDerivativeCorrection,
      proportional, true, true, 0.95f, observed) == opposingDerivativeCorrection);
  assert(keepCorrectionTowardVisibleLine(opposingDerivativeCorrection,
      proportional, false, true, 0.96f, observed) == opposingDerivativeCorrection);
}

void testGuardedEdgeHandoff(int side) {
  uint16_t row[16] = {};
  uint16_t filtered[16] = {};
  LineBranchGuard guard;
  guard.stableFrames = 2u;
  guard.mainStart = side > 0 ? 5u : 9u;
  guard.mainEnd = side > 0 ? 6u : 10u;
  const float oldError = side > 0 ? -13.5f : 13.5f;
  if (side > 0) fill(row, 9, 14);
  else fill(row, 1, 6);

  uint8_t frames = 0u;
  float previousCandidate = 0.0f;
  float recoveredError = 0.0f;
  for (int frame = 0; frame < 2; ++frame) {
    bool changed = false;
    assert(filterLineBranch(guard, row, filtered, changed));
    assert(observeRawLineSide(row).visibleSide == side);
    const bool adopted = recoverGuardedRawEdge(
        guard, row, filtered, oldError, side, frames,
        previousCandidate, recoveredError);
    assert(adopted == (frame == 1));
  }
  assert(recoveredError * side > 0.0f);
  assert(guard.blockedSide == 0 && guard.stableFrames == 0u);
  for (int i = 0; i < 16; ++i) assert(filtered[i] == row[i]);
}

int main() {
  testEdgeLoss(1);
  testEdgeLoss(-1);
  testInnerSideLoss(1);
  testInnerSideLoss(-1);
  testRightCrossingThroughAmbiguousFrame();
  testRightDominatesMixedFrame();
  testOldDirectionExpires();
  testTrackedErrorSnapsRight();
  testGuardKeepsOldOppositeGroup(1);
  testGuardKeepsOldOppositeGroup(-1);
  testDerivativeCannotReverseVisibleTurn(1);
  testDerivativeCannotReverseVisibleTurn(-1);
  testGuardedEdgeHandoff(1);
  testGuardedEdgeHandoff(-1);

  uint16_t ambiguous[16] = {};
  ambiguous[0] = ambiguous[15] = 900u;
  assert(observeRawLineSide(ambiguous).edgeSide == 0);
  assert(linePositionWithEdgeRecovery(
      false, true, false, 1.1f, true, 0, 0.0f, 0, -30.0f) == 0.0f);
}
'''
harness = harness.replace(
    "__TRACK_SELECTOR__",
    section("bool selectTrackedGroup(", "// If the line jumps beyond"),
).replace(
    "__RECOVERY_SELECTOR__",
    section("bool selectRecoveryGroup(", "float lineLostRecoveryError("),
).replace(
    "__SIDE_HELPERS__",
    section("float lineLostRecoveryError(", "// Filter only the PID input."),
).replace(
    "__BRANCH_HELPERS__",
    section("struct LineBranchGuard {", "// While cl/cr approaches"),
).replace(
    "__HANDOFF_HELPER__",
    section("bool recoverGuardedRawEdge(", "struct IntersectionGuard {"),
)

with tempfile.TemporaryDirectory(prefix="mymini-edge-recovery-") as directory:
    cpp = Path(directory) / "edge_recovery.cpp"
    exe = Path(directory) / "edge_recovery.exe"
    cpp.write_text(harness, encoding="utf-8")
    subprocess.run(
        ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", str(cpp), "-o", str(exe)],
        check=True,
    )
    subprocess.run([str(exe)], check=True)

print("PASS: right crossing, error sign, symmetric loss, handoff, and low-Kp behavior")
