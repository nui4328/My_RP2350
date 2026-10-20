"""Replay side branches and ordinary curves against the current line filter."""

from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "src" / "LineFollower.cpp").read_text(encoding="utf-8")
selector = source[source.index("bool selectTrackedGroup("):source.index("// If the line jumps")]
branch = source[source.index("struct LineBranchGuard {"):source.index("// While cl/cr approaches")]

harness = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <initializer_list>
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
__SELECTOR__
__BRANCH__

void frame(uint16_t (&raw)[16], std::initializer_list<int> black) {
  for (auto& value : raw) value = 0u;
  for (int index : black) raw[index] = 900u;
}
struct Result {
  bool active;
  bool changed;
  bool found;
  uint8_t start;
  uint8_t end;
  uint16_t filtered[16];
};
Result step(LineBranchGuard& guard, const uint16_t (&raw)[16], float reference) {
  Result result{};
  result.active = filterLineBranch(guard, raw, result.filtered, result.changed);
  float measured = reference;
  uint8_t outside = 0u;
  result.start = 255u;
  result.end = 255u;
  result.found = selectTrackedGroup(result.filtered, reference, true,
                                    measured, outside, &result.start,
                                    &result.end);
  if (result.active && result.found &&
      !lineGroupOverlapsTrustedFootprint(guard, result.start, result.end))
    result.found = false;
  rememberLineBranchFootprint(guard, result.filtered, result.found,
                              result.start, result.end);
  return result;
}
int main() {
  uint16_t raw[16] = {};
  LineBranchGuard guard;
  frame(raw, {7, 8});
  assert(!step(guard, raw, 0.0f).active);
  assert(!step(guard, raw, 0.0f).active);
  assert(guard.stableFrames == 2u);

  // Separate right arm: retain the main group and leave raw exit data intact.
  frame(raw, {7, 8, 10, 11});
  Result result = step(guard, raw, 0.0f);
  assert(result.active && result.changed && result.found);
  assert(result.start == 7u && result.end == 8u);
  assert(result.filtered[10] == 0u && result.filtered[11] == 0u);
  assert(raw[10] == 900u && raw[11] == 900u);
  frame(raw, {10, 11});
  result = step(guard, raw, 0.0f);
  assert(result.active && !result.found);
  frame(raw, {7, 8});
  assert(step(guard, raw, 0.0f).active);
  result = step(guard, raw, 0.0f);
  assert(!result.active && result.changed && result.found);

  // A connected diagonal wing must not drag the tracked centroid outward.
  guard = LineBranchGuard{};
  frame(raw, {7, 8});
  step(guard, raw, 0.0f);
  step(guard, raw, 0.0f);
  frame(raw, {7, 8, 9, 10, 11});
  result = step(guard, raw, 0.0f);
  assert(result.active && result.found);
  assert(result.filtered[7] && result.filtered[8] && result.filtered[9]);
  assert(!result.filtered[10] && !result.filtered[11]);

  // A detached arm can arrive in the first crossing frame.
  guard = LineBranchGuard{};
  frame(raw, {7, 8});
  step(guard, raw, 0.0f);
  step(guard, raw, 0.0f);
  frame(raw, {10, 11});
  result = step(guard, raw, 0.0f);
  assert(result.active && !result.found);

  // One missed white frame must not let the following branch become the line.
  guard = LineBranchGuard{};
  frame(raw, {7, 8});
  step(guard, raw, 0.0f);
  step(guard, raw, 0.0f);
  frame(raw, {});
  assert(!step(guard, raw, 0.0f).active);
  assert(guard.stableFrames == 2u && guard.missingFrames == 1u);
  frame(raw, {10, 11});
  result = step(guard, raw, 0.0f);
  assert(result.active && !result.found);

  // Symmetric left arm.
  guard = LineBranchGuard{};
  frame(raw, {7, 8});
  step(guard, raw, 0.0f);
  step(guard, raw, 0.0f);
  frame(raw, {4, 5, 7, 8});
  result = step(guard, raw, 0.0f);
  assert(result.active && result.found);
  assert(!result.filtered[4] && !result.filtered[5]);
  assert(result.filtered[7] && result.filtered[8]);

  // A single narrow line moving across the array is an ordinary curve.
  guard = LineBranchGuard{};
  frame(raw, {7, 8});
  step(guard, raw, 0.0f);
  step(guard, raw, 0.0f);
  frame(raw, {8, 9});
  assert(!step(guard, raw, 0.0f).active);
  frame(raw, {9, 10});
  assert(!step(guard, raw, 5.0f).active);
}
'''.replace("__SELECTOR__", selector).replace("__BRANCH__", branch)

with tempfile.TemporaryDirectory() as directory:
    cpp = Path(directory) / "line_branch_filter.cpp"
    exe = Path(directory) / "line_branch_filter.exe"
    cpp.write_text(harness, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    str(cpp), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print("PASS: side branch filter and ordinary curve")
