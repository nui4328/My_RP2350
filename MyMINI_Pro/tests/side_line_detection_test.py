"""Compile the side-line detector used by both tracking arrays and replay frames."""

from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "src" / "LineFollower.cpp").read_text(encoding="utf-8")
start = source.index("struct SideLineCrossing {")
end = source.index("// While cl/cr approaches", start)
detector = source[start:end]

harness = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
using std::max;
namespace MyMINIConfig { constexpr uint8_t SENSOR_COUNT = 16; }
constexpr int8_t LINE_WEIGHT[16] = {
  -50, -43, -37, -30, -23, -17, -10, -3,
     3,  10,  17,  23,  30,  37,  43, 50
};
constexpr int16_t INTERSECTION_ANCHOR_RADIUS = 10;
constexpr int16_t TRACK_WINDOW_RADIUS = 22;
constexpr uint16_t LINE_EXIT_WHITE_RELEASE = 600;
constexpr uint8_t SIDE_LINE_CONFIRM_FRAMES = 2;
__DETECTOR__

void frame(uint16_t (&strength)[16], std::initializer_list<int> black) {
  for (auto& value : strength) value = 0;
  for (int index : black) strength[index] = 900;
}
int main() {
  uint16_t strength[16] = {};
  SideLineCrossing crossing;
  frame(strength, {7, 8});
  auto evidence = detectForwardSideLine(strength, 0.0f, true);
  assert(evidence.side == 0);
  frame(strength, {5, 6, 7});  // An ordinary curved main line stays one group.
  evidence = detectForwardSideLine(strength, 0.0f, true);
  assert(evidence.side == 0);

  // A separated outer pair plus a tracked main group is sufficient evidence
  // immediately; the branch need not move another sensor between frames.
  frame(strength, {7, 8, 14, 15});
  evidence = detectForwardSideLine(strength, 0.0f, true);
  assert(evidence.side == 1 && evidence.separateEdgePair);
  assert(confirmForwardSideLine(crossing, evidence, 0.0f, 1000));
  assert(sideCrossingMainVisible(strength, crossing));
  frame(strength, {5, 14, 15});  // A curved main line remains in its window.
  assert(sideCrossingMainVisible(strength, crossing));
  frame(strength, {14, 15});
  assert(!sideCrossingMainVisible(strength, crossing));
  evidence = detectForwardSideLine(strength, 0.0f, true);
  assert(evidence.side == 0);  // An outer pair alone proves no main line.

  crossing = SideLineCrossing{};
  frame(strength, {11, 12, 14, 15});  // Main track is right of center.
  evidence = detectForwardSideLine(strength, 30.0f, true);
  assert(evidence.side == 1 && evidence.separateEdgePair);
  assert(confirmForwardSideLine(crossing, evidence, 30.0f, 1500));
  assert(sideCrossingMainVisible(strength, crossing));
  frame(strength, {14, 15});
  assert(!sideCrossingMainVisible(strength, crossing));
  frame(strength, {13, 14, 15});
  assert(!sideCrossingMainVisible(strength, crossing));
  frame(strength, {13, 14});
  assert(!sideCrossingMainVisible(strength, crossing));
  frame(strength, {13});
  assert(sideCrossingMainVisible(strength, crossing));

  crossing = SideLineCrossing{};
  frame(strength, {0, 1, 7, 8});
  evidence = detectForwardSideLine(strength, 0.0f, true);
  assert(evidence.side == -1 && evidence.separateEdgePair);
  assert(confirmForwardSideLine(crossing, evidence, 0.0f, 2000));
  assert(sideCrossingMainVisible(strength, crossing));
  frame(strength, {0, 1});
  assert(!sideCrossingMainVisible(strength, crossing));
  frame(strength, {0, 1, 2});
  assert(!sideCrossingMainVisible(strength, crossing));
  frame(strength, {1, 2});
  assert(!sideCrossingMainVisible(strength, crossing));

  crossing = SideLineCrossing{};
  frame(strength, {0, 1, 7, 8, 14, 15});
  evidence = detectForwardSideLine(strength, 0.0f, true);
  assert(evidence.side == 0);  // Both outer sides require separate handling.
  frame(strength, {11, 12, 13, 14, 15});
  evidence = detectForwardSideLine(strength, 30.0f, true);
  assert(evidence.side == 0);  // One continuous off-center curve is not a branch.

  crossing = SideLineCrossing{};
  frame(strength, {2, 3, 4, 5, 6, 7, 8});  // Broad 90-degree left branch.
  evidence = detectForwardSideLine(strength, 0.0f, true);
  assert(evidence.side == -1 && evidence.wide);
  assert(confirmForwardSideLine(crossing, evidence, 0.0f, 1000));
  assert(crossing.activeSide == -1 && crossing.mainError == 0.0f);

  crossing = SideLineCrossing{};
  frame(strength, {7, 8, 9, 10, 11, 12, 13});
  evidence = detectForwardSideLine(strength, 0.0f, true);
  assert(evidence.side == 1 && evidence.wide);
  assert(confirmForwardSideLine(crossing, evidence, 0.0f, 1000));

  crossing = SideLineCrossing{};
  frame(strength, {3, 4, 7, 8});  // Separate 45-degree branch appears.
  evidence = detectForwardSideLine(strength, 0.0f, true);
  assert(evidence.side == -1 && !evidence.wide);
  assert(!confirmForwardSideLine(crossing, evidence, 0.0f, 1000));
  assert(!confirmForwardSideLine(crossing, evidence, 0.0f, 2000));
  frame(strength, {2, 3, 7, 8});  // Branch advances outward, main stays.
  evidence = detectForwardSideLine(strength, 0.0f, true);
  assert(confirmForwardSideLine(crossing, evidence, 0.0f, 3000));
  assert(crossing.activeSide == -1 && crossing.mainError == 0.0f);

  crossing = SideLineCrossing{};
  frame(strength, {7, 8, 10, 11});
  evidence = detectForwardSideLine(strength, 0.0f, true);
  assert(evidence.side == 1 && !evidence.wide);
  assert(!confirmForwardSideLine(crossing, evidence, 0.0f, 1000));
  frame(strength, {7, 8, 11, 12});
  evidence = detectForwardSideLine(strength, 0.0f, true);
  assert(confirmForwardSideLine(crossing, evidence, 0.0f, 2000));

  crossing = SideLineCrossing{};
  for (int i = 0; i < 16; ++i) strength[i] = 900; // Symmetric cross.
  evidence = detectForwardSideLine(strength, 0.0f, true);
  assert(evidence.side == 0);

  crossing = SideLineCrossing{};
  frame(strength, {4, 5, 9, 10});
  evidence = detectForwardSideLine(strength, -20.0f, true);
  assert(!confirmForwardSideLine(crossing, evidence, -20.0f, 1000));
  frame(strength, {4, 5, 10, 11});
  evidence = detectForwardSideLine(strength, -20.0f, true);
  assert(confirmForwardSideLine(crossing, evidence, -19.0f, 2000));
  assert(crossing.mainError == -20.0f);
}
'''.replace("__DETECTOR__", detector)

with tempfile.TemporaryDirectory() as directory:
    source_file = Path(directory) / "side_line_test.cpp"
    executable = Path(directory) / "side_line_test.exe"
    source_file.write_text(harness, encoding="utf-8")
    subprocess.run(
        ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
         str(source_file), "-o", str(executable)], check=True
    )
    subprocess.run([str(executable)], check=True)

print("PASS: side-line detection and main-line visibility")
