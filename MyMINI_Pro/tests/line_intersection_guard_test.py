"""Compile the current line selector/PID/guard and replay synthetic sensor frames."""

from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "src" / "LineFollower.cpp").read_text(encoding="utf-8")
header = (ROOT / "src" / "LineFollower.h").read_text(encoding="utf-8")


def section(text: str, first: str, last: str) -> str:
    start = text.index(first)
    return text[start:text.index(last, start)]


assert source.index("if (centerBlack) {", source.index("void turn(TurnMode")) < source.index(
    "updateIntersectionGuard(intersectionGuard", source.index("void turn(TurnMode")
)
assert source.index("if (!distanceMode) {", source.index("void runLine")) < source.index(
    "updateIntersectionGuard(intersectionGuard", source.index("void runLine")
)

snippets = "\n".join([
    section(header, "constexpr float LINE_KI", "// f_line()/b_line() only.").replace(
        "void print_line_intersection_trace(Print& output = Serial);", ""),
    section(source, "constexpr int8_t LINE_WEIGHT", "struct TurnMotorRatio"),
    section(source, "bool selectTrackedGroup(", "// If the line jumps"),
    section(source, "struct IntersectionGuard", "// Keep the correction calculation"),
    section(source, "float calculateLinePidCorrection(", "float smoothstep("),
])

harness = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <type_traits>
using std::abs;
using std::max;
using std::min;
namespace MyMINIConfig { constexpr uint8_t SENSOR_COUNT = 16; }
template <class T, class L, class H> T constrain(T value, L low, H high) {
  return value < low ? static_cast<T>(low) :
         value > high ? static_cast<T>(high) : value;
}
__SNIPPETS__

struct Motion {
  float tracked = 0.0f, previous = 0.0f, integral = 0.0f, derivative = 0.0f;
  bool initialized = false, stopped = false;
  uint32_t whiteMs = 0;
  IntersectionGuard guard;
};
struct Step { float error; int left, right; bool hold; };

Step tick(Motion& motion, const uint16_t (&strength)[16], uint32_t nowUs,
          bool guarded, bool reverse, int speed, float kp) {
  constexpr float dt = 0.02f;
  float measured = motion.tracked;
  uint8_t outside = 0;
  bool found = selectTrackedGroup(strength, motion.tracked,
                                  motion.initialized, measured, outside);
  const auto decision = guarded ? updateIntersectionGuard(
      motion.guard, strength, motion.tracked, measured, found, outside,
      nowUs, intersectionHoldLimitUs(speed, speed))
      : IntersectionGuardResult::Follow;
  const bool hold = decision == IntersectionGuardResult::Hold;
  if (hold) { measured = motion.guard.heldError; found = true; }
  if (found) {
    if (hold) motion.tracked = motion.guard.heldError;
    else if (!motion.initialized) {
      motion.tracked = constrain(measured, -50.0f, 50.0f);
      motion.initialized = true;
    } else {
      const float maxChange = constrain(MAX_ERROR_RATE_PER_SECOND * dt,
                                        1.0f, 10.0f);
      motion.tracked = constrain(motion.tracked + constrain(
          measured - motion.tracked, -maxChange, maxChange), -50.0f, 50.0f);
    }
  }
  bool anyBlack = false;
  for (const uint16_t value : strength) anyBlack |= value > 0u;
  const float error = anyBlack ? motion.tracked : 0.0f;
  if (hold || (decision == IntersectionGuardResult::Released && found)) {
    motion.previous = error;
    motion.derivative = 0.0f;
  }
  motion.whiteMs = anyBlack ? 0u : motion.whiteMs + 20u;
  if (motion.whiteMs >= 3000u) {
    motion.stopped = true;
    return {error, 1, 1, hold};
  }
  const float correction = calculateLinePidCorrection(
      kp, error, dt, found && !hold, reverse ? -1 : 1,
      reverse ? 0.0f : -100.0f, speed, speed,
      motion.previous, motion.integral, motion.derivative,
      LINE_KI, true, LINE_KD);
  if (found) motion.previous = error;
  if (reverse) return {error,
      -static_cast<int>(constrain(lroundf(speed - correction), 0L, 100L)),
      -static_cast<int>(constrain(lroundf(speed + correction), 0L, 100L)), hold};
  return {error,
      static_cast<int>(constrain(lroundf(speed + correction), -100L, 100L)),
      static_cast<int>(constrain(lroundf(speed - correction), -100L, 100L)), hold};
}

void fill(uint16_t (&frame)[16], int first, int last) {
  for (int i = 0; i < 16; ++i) frame[i] = i >= first && i <= last ? 900u : 0u;
}
void report(const char* label, const Step& before, const Step& after) {
  std::cout << label << " before:error=" << before.error
            << " motor=" << before.left << ',' << before.right
            << " after:error=" << after.error << " motor="
            << after.left << ',' << after.right << '\n';
}
int main() {
  std::cout << std::unitbuf;
  uint16_t frame[16] = {};
  for (const bool reverse : {false, true}) {
    for (const bool right : {false, true}) {
      Motion before, after;
      fill(frame, 7, 8);
      tick(before, frame, 20000, false, reverse, 50, 0.4f);
      tick(after, frame, 20000, true, reverse, 50, 0.4f);
      tick(before, frame, 40000, false, reverse, 50, 0.4f);
      tick(after, frame, 40000, true, reverse, 50, 0.4f);
      fill(frame, right ? 7 : 0, right ? 15 : 8);
      Step oldStep{}, newStep{};
      for (uint32_t t : {60000u, 80000u, 100000u}) {
        oldStep = tick(before, frame, t, false, reverse, 50, 0.4f);
        newStep = tick(after, frame, t, true, reverse, 50, 0.4f);
      }
      report(reverse ? (right ? "b_line right" : "b_line left") :
             (right ? "f_line right" : "f_line left"), oldStep, newStep);
      assert(newStep.hold && newStep.error == 0.0f);
      assert(abs(newStep.left) == 50 && abs(newStep.right) == 50);
      assert(oldStep.error != 0.0f && oldStep.left != oldStep.right);
      fill(frame, 7, 8);
      tick(after, frame, 120000, true, reverse, 50, 0.4f);
      Step resumed = tick(after, frame, 140000, true, reverse, 50, 0.4f);
      assert(!resumed.hold && resumed.error == 0.0f);
      assert(abs(resumed.left) == 50 && abs(resumed.right) == 50);
    }
  }

  for (const bool right : {false, true}) {
    Motion before, after;
    fill(frame, 7, 8);
    for (uint32_t t : {20000u, 40000u}) {
      tick(before, frame, t, false, false, 10, 0.15f);
      tick(after, frame, t, true, false, 10, 0.15f);
    }
    fill(frame, right ? 7 : 0, right ? 15 : 8);
    Step oldStep{}, newStep{};
    for (uint32_t t : {60000u, 80000u}) {
      oldStep = tick(before, frame, t, false, false, 10, 0.15f);
      newStep = tick(after, frame, t, true, false, 10, 0.15f);
    }
    report(right ? "tcr approach right" : "tcl approach left",
           oldStep, newStep);
    assert(newStep.hold && newStep.left == 10 && newStep.right == 10);
    assert(oldStep.left != oldStep.right);
  }

  Motion offsetBranch;
  fill(frame, 6, 7);
  for (uint32_t t : {20000u, 40000u})
    tick(offsetBranch, frame, t, true, false, 50, 0.4f);
  const float entryError = offsetBranch.tracked;
  const float entryIntegral = offsetBranch.integral;
  fill(frame, 0, 7);
  for (uint32_t t : {60000u, 80000u}) {
    const Step step = tick(offsetBranch, frame, t,
                           true, false, 50, 0.4f);
    assert(step.hold && step.error == entryError);
    assert(offsetBranch.derivative == 0.0f);
    assert(offsetBranch.integral == entryIntegral);
  }
  fill(frame, 7, 8);
  assert(tick(offsetBranch, frame, 100000u,
              true, false, 50, 0.4f).hold);
  const Step rejoined = tick(offsetBranch, frame, 120000u,
                             true, false, 50, 0.4f);
  assert(!rejoined.hold && rejoined.error != entryError);
  assert(offsetBranch.derivative == 0.0f);
  std::cout << "off-center branch: error held; integral/D frozen; D-free rejoin\n";

  Motion straightBefore, straightAfter, curveBefore, curveAfter;
  for (uint32_t t : {20000u, 40000u, 60000u, 80000u}) {
    fill(frame, 7, 8);
    auto oldStep = tick(straightBefore, frame, t, false, false, 50, 0.4f);
    auto newStep = tick(straightAfter, frame, t, true, false, 50, 0.4f);
    assert(oldStep.left == newStep.left && oldStep.right == newStep.right);
  }
  const int curveFirst[] = {7, 7, 6, 5, 4};
  for (int i = 0; i < 5; ++i) {
    fill(frame, curveFirst[i], curveFirst[i] + 1);
    const uint32_t t = static_cast<uint32_t>(i + 1) * 20000u;
    auto oldStep = tick(curveBefore, frame, t, false, false, 50, 0.4f);
    auto newStep = tick(curveAfter, frame, t, true, false, 50, 0.4f);
    assert(!newStep.hold);
    assert(oldStep.left == newStep.left && oldStep.right == newStep.right);
    if (i == 4) report("real curve", oldStep, newStep);
  }
  IntersectionGuard ninetyDegree;
  fill(frame, 7, 8);
  for (uint32_t t : {20000u, 40000u})
    assert(updateIntersectionGuard(ninetyDegree, frame, 0.0f, 0.0f,
           true, 0, t, intersectionHoldLimitUs(50, 50)) ==
           IntersectionGuardResult::Follow);
  fill(frame, 0, 8);
  for (uint32_t t : {60000u, 80000u}) {
    float measured = 0.0f;
    uint8_t outside = 0;
    const bool found = selectTrackedGroup(frame, 0.0f, true,
                                          measured, outside);
    assert(updateIntersectionGuard(ninetyDegree, frame, 0.0f, measured,
           found, outside, t, intersectionHoldLimitUs(50, 50)) ==
           IntersectionGuardResult::Hold);
  }
  fill(frame, 0, 1);
  float measured = 0.0f;
  uint8_t outside = 0;
  const bool found = selectTrackedGroup(frame, 0.0f, true,
                                        measured, outside);
  assert(updateIntersectionGuard(ninetyDegree, frame, 0.0f, measured,
         found, outside, 100000u, intersectionHoldLimitUs(50, 50)) ==
         IntersectionGuardResult::Released);
  std::cout << "90-degree curve: hold releases when original anchor disappears\n";

  IntersectionGuard bounded;
  fill(frame, 7, 8);
  for (uint32_t t : {20000u, 40000u})
    updateIntersectionGuard(bounded, frame, 0.0f, 0.0f,
                            true, 0, t, intersectionHoldLimitUs(50, 50));
  fill(frame, 0, 8);
  selectTrackedGroup(frame, 0.0f, true, measured, outside);
  for (uint32_t t : {60000u, 80000u})
    updateIntersectionGuard(bounded, frame, 0.0f, measured,
                            true, outside, t, intersectionHoldLimitUs(50, 50));
  assert(updateIntersectionGuard(bounded, frame, 0.0f, measured,
         true, outside, 220000u, intersectionHoldLimitUs(50, 50)) ==
         IntersectionGuardResult::Released);
  std::cout << "crossing hold: bounded by speed-based time limit\n";
  Motion lostAfterBranch;
  fill(frame, 7, 8);
  for (uint32_t t : {20000u, 40000u})
    tick(lostAfterBranch, frame, t, true, false, 50, 0.4f);
  fill(frame, 0, 8);
  for (uint32_t t : {60000u, 80000u})
    assert(tick(lostAfterBranch, frame, t, true, false, 50, 0.4f).hold);
  fill(frame, -1, -1);
  const Step firstWhite = tick(lostAfterBranch, frame, 100000u,
                               true, false, 50, 0.4f);
  assert(!firstWhite.hold && !lostAfterBranch.stopped);
  for (int i = 1; i < 150; ++i)
    tick(lostAfterBranch, frame, 100000u + static_cast<uint32_t>(i) * 20000u,
         true, false, 50, 0.4f);
  assert(lostAfterBranch.stopped);
  std::cout << "branch then all white: guard releases; line-loss stop remains\n";
  fill(frame, -1, -1);
  Motion lostBefore, lostAfter;
  for (int i = 1; i <= 150; ++i) {
    tick(lostBefore, frame, static_cast<uint32_t>(i) * 20000u,
         false, false, 50, 0.4f);
    tick(lostAfter, frame, static_cast<uint32_t>(i) * 20000u,
         true, false, 50, 0.4f);
  }
  assert(lostBefore.stopped && lostAfter.stopped);
  std::cout << "all white: both stop at 3000 ms\n";

  // f0/f1 exit: arm after 100 ms and two white frames; confirm black 2/3.
  bool armed = false, crossing = false, cleared = false;
  bool history[3] = {false, false, false};
  bool clearHistory[3] = {false, false, false};
  int whiteFrames = 0, index = 0, blackCount = 0, clearIndex = 0;
  Motion guardedExit;
  for (int frameNo = 0; frameNo < 12; ++frameNo) {
    const uint32_t nowUs = static_cast<uint32_t>(frameNo + 1) * 20000u;
    const bool onBranch = frameNo >= 7 && frameNo <= 8;
    fill(frame, onBranch ? 0 : 7, 8);
    const bool exitWhite = frame[0] == 0u && frame[1] == 0u;
    const bool exitBlack = frame[0] > 0u || frame[1] > 0u;
    const Step step = tick(guardedExit, frame, nowUs,
                           true, false, 50, 0.4f);
    if (!armed && nowUs >= 100000u) {
      whiteFrames = exitWhite ? whiteFrames + 1 : 0;
      if (whiteFrames >= 2) armed = true;
    } else if (armed && !crossing) {
      history[index] = exitBlack;
      index = (index + 1) % 3;
      blackCount = history[0] + history[1] + history[2];
      if (blackCount >= 2) {
        crossing = true;
        assert(step.hold);
      }
    } else if (crossing && !cleared) {
      clearHistory[clearIndex] = exitWhite;
      clearIndex = (clearIndex + 1) % 3;
      cleared = clearHistory[0] + clearHistory[1] + clearHistory[2] >= 2;
    }
  }
  assert(armed && crossing && cleared && blackCount >= 2);
  std::cout << "f0 exit: black 2/3 while guard holds, then white 2/3 completes\n";
  bool backArmed = false, backStopped = false;
  bool backHistory[3] = {false, false, false};
  int backWhiteFrames = 0, backHistoryIndex = 0;
  Motion guardedBackExit;
  for (int frameNo = 0; frameNo <= 8; ++frameNo) {
    const uint32_t nowUs = static_cast<uint32_t>(frameNo + 1) * 20000u;
    fill(frame, frameNo >= 7 ? 0 : 7, 8);
    const bool exitWhite = frame[0] == 0u && frame[1] == 0u;
    const bool exitBlack = frame[0] > 0u || frame[1] > 0u;
    const Step step = tick(guardedBackExit, frame, nowUs,
                           true, true, 50, 0.4f);
    if (!backArmed && nowUs >= 100000u) {
      backWhiteFrames = exitWhite ? backWhiteFrames + 1 : 0;
      backArmed = backWhiteFrames >= 2;
    } else if (backArmed) {
      backHistory[backHistoryIndex] = exitBlack;
      backHistoryIndex = (backHistoryIndex + 1) % 3;
      if (backHistory[0] + backHistory[1] + backHistory[2] >= 2) {
        assert(step.hold);
        backStopped = true;
        break;
      }
    }
  }
  assert(backArmed && backStopped);
  std::cout << "b0 exit: black 2/3 while guard holds; stop condition remains\n";
  std::cout << "cl/cr: source checks centerBlack before guard and enters Rotate\n";
  lineIntersectionTraceEnabled = true;
  uint16_t normalizedTrace[16] = {}, blackTrace[16] = {}, guardTrace[16] = {};
  normalizedTrace[0] = 123u;
  blackTrace[0] = 877u;
  guardTrace[0] = 0u;
  for (uint32_t i = 0; i < 60u; ++i) {
    auto* trace = nextLineIntersectionTraceFrame('F', i + 1u,
        i * 20000u, 20000u, 20000u, normalizedTrace, blackTrace,
        guardTrace);
    if (!trace) break;
    trace->error = i < 4u ? -3.0f : 8.0f;
    finishLineIntersectionTraceFrame(*trace);
  }
  assert(lineIntersectionTraceTriggered && lineIntersectionTraceFrozen);
  assert(lineIntersectionTraceCount == 53u);
  assert(lineIntersectionTrace[0].normalized[0] == 123u);
  assert(lineIntersectionTrace[0].blackStrength[0] == 877u);
  assert(lineIntersectionTrace[0].guardStrength[0] == 0u);
  std::cout << "passive trace: 16-channel copies; 48 post-jump frames retained\n";

  // An arm can first enter the tracking window without reaching two outside
  // sensors. That first widening must not erase the stable narrow-line anchor.
  for (const bool reverse : {false, true}) {
    for (const bool right : {false, true}) {
      Motion unguarded, guarded;
      fill(frame, 7, 8);
      for (uint32_t t : {20000u, 40000u}) {
        tick(unguarded, frame, t, false, reverse, 50, 0.4f);
        tick(guarded, frame, t, true, reverse, 50, 0.4f);
      }
      fill(frame, right ? 7 : 4, right ? 11 : 8);
      Step oldStep = tick(unguarded, frame, 60000u, false, reverse, 50, 0.4f);
      Step newStep = tick(guarded, frame, 60000u, true, reverse, 50, 0.4f);
      report(reverse ? (right ? "b_line early right" : "b_line early left") :
             (right ? "f_line early right" : "f_line early left"), oldStep, newStep);
      assert(newStep.hold && newStep.error == 0.0f);
      assert(abs(newStep.left) == 50 && abs(newStep.right) == 50);
      fill(frame, right ? 7 : 2, right ? 13 : 8);
      newStep = tick(guarded, frame, 80000u, true, reverse, 50, 0.4f);
      assert(newStep.hold && newStep.error == 0.0f);
    }
  }

  // A side arm first adds just two adjacent black sensors. It has not yet
  // reached the far-outside count used by the original guard.
  for (const bool reverse : {false, true}) {
    for (const bool right : {false, true}) {
      Motion unguarded, guarded;
      fill(frame, 7, 8);
      for (uint32_t t : {20000u, 40000u}) {
        tick(unguarded, frame, t, false, reverse, 50, 0.4f);
        tick(guarded, frame, t, true, reverse, 50, 0.4f);
      }
      fill(frame, right ? 7 : 5, right ? 10 : 8);
      const Step oldFirst = tick(unguarded, frame, 60000u,
                                 false, reverse, 50, 0.4f);
      const Step newFirst = tick(guarded, frame, 60000u,
                                 true, reverse, 50, 0.4f);
      report(reverse ? (right ? "b_line first right wing" :
                                "b_line first left wing") :
                       (right ? "f_line first right wing" :
                                "f_line first left wing"), oldFirst, newFirst);
      assert(oldFirst.error != 0.0f && oldFirst.left != oldFirst.right);
      assert(newFirst.hold && newFirst.error == 0.0f);
      assert(abs(newFirst.left) == 50 && abs(newFirst.right) == 50);
      const Step newSecond = tick(guarded, frame, 80000u,
                                  true, reverse, 50, 0.4f);
      assert(newSecond.hold && guarded.guard.confirmed);
      fill(frame, 7, 8);
      assert(tick(guarded, frame, 100000u,
                  true, reverse, 50, 0.4f).hold);
      const Step rejoined = tick(guarded, frame, 120000u,
                                true, reverse, 50, 0.4f);
      assert(!rejoined.hold && guarded.derivative == 0.0f);
    }
  }

  for (const bool right : {false, true}) {
    Motion unguarded, guarded;
    fill(frame, 7, 8);
    for (uint32_t t : {20000u, 40000u}) {
      tick(unguarded, frame, t, false, false, 10, 0.15f);
      tick(guarded, frame, t, true, false, 10, 0.15f);
    }
    fill(frame, right ? 7 : 5, right ? 10 : 8);
    const Step oldStep = tick(unguarded, frame, 60000u,
                              false, false, 10, 0.15f);
    const Step newStep = tick(guarded, frame, 60000u,
                              true, false, 10, 0.15f);
    report(right ? "tcr first right wing" : "tcl first left wing",
           oldStep, newStep);
    assert(oldStep.left != oldStep.right);
    assert(newStep.hold && newStep.left == 10 && newStep.right == 10);
    assert(tick(guarded, frame, 80000u,
                true, false, 10, 0.15f).hold);
    assert(guarded.guard.confirmed);
  }

  Motion wideningCurveBefore, wideningCurveAfter;
  const int wideningFirst[] = {7, 7, 6, 5, 4};
  const int wideningLast[] = {8, 8, 8, 8, 8};
  for (int i = 0; i < 5; ++i) {
    fill(frame, wideningFirst[i], wideningLast[i]);
    const uint32_t t = static_cast<uint32_t>(i + 1) * 20000u;
    const Step oldStep = tick(wideningCurveBefore, frame, t,
                              false, false, 50, 0.4f);
    const Step newStep = tick(wideningCurveAfter, frame, t,
                              true, false, 50, 0.4f);
    assert(!newStep.hold && oldStep.left == newStep.left &&
           oldStep.right == newStep.right);
  }
  std::cout << "gradual wide curve: PID follows all frames\n";

  // A full cross has black on both sides of the prior line, yet the main
  // anchor is still visible and must remain the tracking target.
  Motion cross;
  fill(frame, 7, 8);
  tick(cross, frame, 20000u, true, false, 50, 0.4f);
  tick(cross, frame, 40000u, true, false, 50, 0.4f);
  fill(frame, 2, 13);
  assert(tick(cross, frame, 60000u, true, false, 50, 0.4f).hold);
  assert(tick(cross, frame, 80000u, true, false, 50, 0.4f).hold);
  std::cout << "PASS\n";
}
'''.replace("__SNIPPETS__", snippets)

with tempfile.TemporaryDirectory(prefix="mymini-intersection-") as directory:
    directory = Path(directory)
    cpp = directory / "frames.cpp"
    exe = directory / "frames.exe"
    cpp.write_text(harness, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-O2", str(cpp), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
