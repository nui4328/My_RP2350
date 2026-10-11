"""Compile the current distance deceleration function and check turn handoff speeds."""

from pathlib import Path
import re
import subprocess
import tempfile


root = Path(__file__).resolve().parents[1]
source = (root / "src" / "LineFollower.cpp").read_text(encoding="utf-8")
header = (root / "src" / "LineFollower.h").read_text(encoding="utf-8")


def section(text: str, first: str, last: str) -> str:
    start = text.index(first)
    return text[start:text.index(last, start)]


assert "decelFactor = distanceLineDecelFactor(" in source
assert "bool set_line_decel_ramp_cm(int distanceCmTimes100);" in header
assert "recordLineNormalExit(" in source
assert "pendingLineHandoff = stopPull == 0u;" in source
assert re.search(r"previousExitSource\s*==\s*LineExitSource::distance\s*"
                 r"\?\s*turnDistanceExitSpeed", source)
backward_wrappers = section(source, "// Public b_line API", "// Public fw_gyro API")
assert "if (!continuousEntry) motor(1, 1);" not in backward_wrappers
assert "runLine(sl, sr, kp, true, distanceCm * 10.0f, f0, stopPull, true," in backward_wrappers

constants = section(header, "constexpr float LINE_DECEL_DISTANCE_MM", "constexpr int16_t TRACK_WINDOW_RADIUS")
functions = section(source, "float smoothstep(", "float wrapAngle180(")
setter = section(source, "bool set_line_decel_ramp_cm(", "bool set_line_pid_tuning(")
declarations = section(header, "bool set_line_decel_ramp_cm(int", "bool set_line_pid_tuning(")
harness = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
using std::abs;
using std::isfinite;
using std::min;
template <class T, class L, class H>
T constrain(T value, L low, H high) {
  return value < low ? static_cast<T>(low) :
         value > high ? static_cast<T>(high) : value;
}
__CONSTANTS__
uint8_t turnDistanceExitSpeed = 30;
float lineDecelRampMm = -1.0f;
__FUNCTIONS__
__SETTER__

void expect(const char* label, float remaining, int baseSpeed,
            uint8_t pull, bool reverse, float expectedSpeed) {
  const float factor = distanceLineDecelFactor(
      remaining, 100.0f, baseSpeed, baseSpeed, pull);
  const float result = (reverse ? -baseSpeed : baseSpeed) * factor;
  assert(fabsf(result - (reverse ? -expectedSpeed : expectedSpeed)) < 0.02f);
  std::cout << label << " remaining_mm=" << remaining
            << " motor=" << result << ',' << result << '\n';
}

int main() {
  expect("f_line30 handoff", 40.0f, 30, 0, false, 30.0f);
  expect("f_line30 handoff", 20.0f, 30, 0, false, 30.0f);
  expect("f_line30 handoff", 0.0f, 30, 0, false, 30.0f);
  expect("f_line50 handoff", 40.0f, 50, 0, false, 50.0f);
  expect("f_line50 handoff", 20.0f, 50, 0, false, 40.0f);
  expect("f_line50 handoff", 0.0f, 50, 0, false, 30.0f);
  expect("f_line30 brake pulse", 0.0f, 30, 10, false, 7.5f);
  expect("b_line30 handoff", 40.0f, 30, 0, true, 30.0f);
  expect("b_line30 handoff", 0.0f, 30, 0, true, 30.0f);
  expect("b_line50 handoff", 20.0f, 50, 0, true, 40.0f);
  expect("b_line50 handoff", 0.0f, 50, 0, true, 30.0f);
  turnDistanceExitSpeed = 20;
  expect("f_line30 changed setting", 0.0f, 30, 0, false, 20.0f);
  expect("f_line30 changed setting", 20.0f, 30, 0, false, 25.0f);
  expect("b_line30 changed setting", 0.0f, 30, 0, true, 20.0f);
  turnDistanceExitSpeed = 30;
  assert(set_line_decel_ramp_cm(200));
  expect("2cm range", 30.0f, 50, 0, false, 50.0f);
  expect("2cm range", 10.0f, 50, 0, false, 40.0f);
  expect("2cm range", 0.0f, 50, 0, false, 30.0f);
  expect("2cm brake pulse", 0.0f, 50, 10, false, 12.5f);
  expect("b_line 2cm range", 30.0f, 50, 0, true, 50.0f);
  expect("b_line 2cm range", 10.0f, 50, 0, true, 40.0f);
  expect("b_line 2cm range", 0.0f, 50, 0, true, 30.0f);
  expect("b_line brake pulse", 0.0f, 50, 10, true, 12.5f);
  assert(set_line_decel_ramp_cm(150));
  expect("1.5cm range", 20.0f, 50, 0, false, 50.0f);
  expect("1.5cm range", 7.5f, 50, 0, false, 40.0f);
  expect("1.5cm range", 0.0f, 50, 0, false, 30.0f);
  assert(set_line_decel_ramp_cm(1200));
  expect("range capped at target", 50.0f, 50, 0, false, 40.0f);
  assert(!set_line_decel_ramp_cm(-1));
  expect("invalid keeps setting", 50.0f, 50, 0, false, 40.0f);
  assert(set_line_decel_ramp_cm(0));
  expect("zero disables ramp", 0.0f, 50, 0, false, 50.0f);
  std::cout << "PASS\n";
}
'''.replace("__CONSTANTS__", constants).replace("__FUNCTIONS__", functions).replace("__SETTER__", setter)

with tempfile.TemporaryDirectory(prefix="mymini-distance-handoff-") as directory:
    cpp = Path(directory) / "handoff.cpp"
    exe = Path(directory) / "handoff.exe"
    cpp.write_text(harness, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    str(cpp), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)

    old_call = Path(directory) / "old_float_call.cpp"
    old_call.write_text(declarations + "\nint main() { return set_line_decel_ramp_cm(2.0f); }\n",
                        encoding="utf-8")
    rejected = subprocess.run(["g++", "-std=c++17", "-fsyntax-only", str(old_call)],
                              capture_output=True, text=True)
    assert rejected.returncode != 0 and "deleted" in rejected.stderr
