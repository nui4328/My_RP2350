"""Replay ordinary distance recovery and final-10-cm side exits in actual helpers."""
from pathlib import Path
import subprocess
import tempfile

source = (Path(__file__).resolve().parents[1] / "src/LineFollower.cpp").read_text(encoding="utf-8")


def section(start, end):
    first = source.index(start)
    return source[first:source.index(end, first)]


# Verify the new policy is confined to ordinary forward distance commands.
policy = section("  const bool directDistanceTracking =", "  // Keep the selected target")
assert "distanceMode && !reverseDirection && !crossingMode" in policy
assert "!directDistanceTracking || kp > 0.5f" in policy
assert "!crossingMode && !directDistanceTracking && filterLineBranch(" in source
assert "if (crossingMode || directDistanceTracking)" in source
assert "if (forwardLineTracking && !directDistanceTracking) {\n      lineFound = acceptDistanceCaptureGroup(" in source
assert "if (!directDistanceTracking)\n      rememberLineBranchFootprint(" in source
assert "!crossingMode && !directDistanceTracking && kp > 0.95f" in source
assert source.index("distanceSideExit.detected(") < source.index("const bool selectedLineFound =")
assert source.index("distanceSideExit.detected(") < source.index("distanceSideExit.observe(")

helpers = section("struct DistanceSideExit {", "struct RawLineSideObservation {")
capture = section("    if (forwardLineTracking && edgeRecoveryActive && lineFound) {",
                  "    const bool snappedToVisibleLine")
white_update = section("    if (forwardLineTracking && edgeRecoveryEnabled && anyConfirmedBlack",
                       "    const RawLineSideObservation rawSide")
edge_selector = section("bool selectRememberedEdgeGroup(", "// While cl/cr approaches")
recovery_selector = section("bool selectRecoveryGroup(", "struct CrossingMainTrack {")

harness = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <iostream>
namespace MyMINIConfig { constexpr uint8_t SENSOR_COUNT = 16; }
constexpr int8_t LINE_WEIGHT[16] = {
  -50,-43,-37,-30,-23,-17,-10,-3,3,10,17,23,30,37,43,50
};
constexpr float TRACK_WINDOW_RADIUS = 22.0f;
constexpr uint32_t DISTANCE_EDGE_TURN_DELAY_US = 30000UL;
constexpr uint8_t DISTANCE_PID_REJOIN_FRAMES = 3u;
constexpr uint8_t DISTANCE_PID_REJOIN_MAX_WIDTH = 4u;
constexpr float DISTANCE_PID_REJOIN_CENTER_ERROR = 10.0f;
constexpr float DISTANCE_PID_REJOIN_BASE_FACTOR = 0.25f;
constexpr uint32_t DISTANCE_PID_REJOIN_RAMP_US = 80000UL;
__HELPERS__
__EDGE_SELECTOR__
__RECOVERY_SELECTOR__

void row(uint16_t (&raw)[16], std::initializer_list<int> black) {
  for (auto& v : raw) v = 0u;
  for (int i : black) raw[i] = 900u;
}

void testSideExit() {
  uint16_t raw[16] = {};
  DistanceSideExit exit;
  row(raw, {6,7,8,14,15});
  assert(!exit.detected(raw, 50)); // No previous main line.
  exit.observe(true,6,8,-3);
  assert(!exit.detected(raw, 100.01f));
  assert(exit.detected(raw, 100.0f));
  assert(exit.detected(raw, 0.0f));
  assert(exit.detected(raw, 50.0f)); // Also works for a target under 10 cm.
  row(raw, {0,1,6,7,8});
  assert(exit.detected(raw, 99));
  row(raw, {0,1,6,7,8,14,15});
  assert(exit.detected(raw, 99));
  row(raw, {6,7,8,15});
  assert(!exit.detected(raw, 99));
  row(raw, {0,6,7,8});
  assert(!exit.detected(raw, 99));
  row(raw, {14,15});
  assert(!exit.detected(raw, 99)); // Main vanished; edge alone is not a branch.
  row(raw, {0,1});
  assert(!exit.detected(raw, 99));
  row(raw, {0,1,14,15});
  assert(!exit.detected(raw, 99)); // Neither is near the previous center group.
  row(raw, {});
  assert(!exit.detected(raw, 99));
  row(raw, {6,7,8,9,10,11,12,13,14,15});
  assert(!exit.detected(raw, 99)); // Connected line: no white separator.
  row(raw, {0,1,2,3,4,5,6,7,8});
  assert(!exit.detected(raw, 99));
  row(raw, {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15});
  assert(!exit.detected(raw, 99));
  // Curves migrate to either edge but remain a single group.
  for (int direction : {-1,1}) {
    exit.observe(true,7,8,0);
    for (int n=1; n<=7; ++n) {
      int first=7+direction*n, last=first+1;
      row(raw,{first,last});
      assert(!exit.detected(raw, 50));
      exit.observe(true,first,last,(LINE_WEIGHT[first]+LINE_WEIGHT[last])*0.5f);
    }
  }
  // Main itself is at the right edge; a separate left edge is still a marker.
  exit.observe(true,14,15,46.5f);
  row(raw,{0,1,14,15});
  assert(exit.detected(raw, 99));
  exit.observe(false,255,255,0);
  assert(!exit.detected(raw, 99));
  // A recovered group outside the old tracking window retains its bounds.
  row(raw,{10,11});
  float measured=0;
  uint8_t size=0,first=255,last=255;
  assert(selectRecoveryGroup(raw,-30,measured,size,&first,&last));
  assert(first==10 && last==11 && size==2);
  exit.observe(true,first,last,measured);
  row(raw,{0,1,10,11});
  assert(exit.detected(raw,99));
}

struct RecoveryRun {
  bool distanceMode=true, reverseDirection=false, crossingMode=false;
  bool forwardLineTracking=true;
  LastBlackSensor lastBlackSensor;
  DistanceEdgeTurnDelay distanceEdgeTurnDelay;
  CrossingRecoveryBrake crossingRecoveryBrake;
  DistancePidRejoinGate distancePidRejoinGate;
  bool edgeRecoveryActive=false;
  float integral=10, filteredDerivative=1000, lastPidError=-50;
  bool sample(float kp, const uint16_t* blackStrength, uint32_t nowUs) {
    __POLICY__
    bool anyConfirmedBlack=false;
    for (int i=0;i<16;++i) anyConfirmedBlack |= blackStrength[i]!=0u;
    __WHITE_UPDATE__
    (void)crossingBrakeActive;
    return waitingForEdgeTurn;
  }
  void capture(bool lineFound, float measuredError, float targetPosition=0) {
    const float kp=0.85f;
    __POLICY__
    (void)edgeRecoveryEnabled;
    __CAPTURE__
  }
};

void testRecovery() {
  uint16_t raw[16] = {};
  for (int side : {-1,1}) {
    for (float kp : {0.35f,0.5f,0.5001f,0.85f,1.5f}) {
      RecoveryRun r;
      row(raw, side>0 ? std::initializer_list<int>{14,15}
                      : std::initializer_list<int>{0,1});
      assert(!r.sample(kp,raw,1000));
      row(raw,{});
      assert(!r.sample(kp,raw,2000)); // No 30-ms forward delay.
      assert(r.edgeRecoveryActive == (kp>0.5f));
      float error=distanceLinePosition(false,false,false,0,r.lastBlackSensor);
      assert(error == (kp>0.5f ? side*50.0f : 0.0f));
      if (kp<=0.5f) continue;
      assert(side*(kp*error)>0);
      r.distanceEdgeTurnDelay.markTurnCommanded();
      // The new group can be anywhere and need not be centered or <=4 wide.
      row(raw,{5,6,7,8,9});
      float measured=0;
      uint8_t first=255,last=255;
      assert(selectRememberedEdgeGroup(raw,r.lastBlackSensor.position(),measured,first,last));
      assert(first==5 && last==9);
      r.capture(true,measured);
      assert(!r.edgeRecoveryActive && r.lastBlackSensor.position()==0);
      assert(!r.distancePidRejoinGate.active);
      assert(r.distancePidRejoinGate.baseFactor(3000)==1.0f);
      assert(r.integral==0 && r.filteredDerivative==0 && r.lastPidError==measured);
      // A later opposite corner must replace the old direction.
      row(raw, side>0 ? std::initializer_list<int>{0}
                      : std::initializer_list<int>{15});
      r.sample(kp,raw,4000);
      row(raw,{});
      assert(!r.sample(kp,raw,5000));
      assert(r.edgeRecoveryActive && r.lastBlackSensor.position()==-side*50.0f);
    }
  }
  // No edge was ever seen: do not invent a turn.
  RecoveryRun empty;
  row(raw,{7,8});
  empty.sample(0.85f,raw,0);
  row(raw,{});
  empty.sample(0.85f,raw,1000);
  assert(!empty.edgeRecoveryActive && empty.lastBlackSensor.position()==0);
  // Sensor-ended calls retain their approach delay and three-frame capture.
  RecoveryRun sensor;
  sensor.distanceMode=false;
  row(raw,{15});
  sensor.sample(0.35f,raw,0);
  row(raw,{});
  assert(sensor.sample(0.35f,raw,1000));
  assert(!sensor.sample(0.35f,raw,31000));
  sensor.capture(true,43);
  assert(sensor.distancePidRejoinGate.active);
  // Crossing mode retains its brake and capture gate.
  RecoveryRun crossing;
  crossing.crossingMode=true;
  row(raw,{15});
  crossing.sample(0.35f,raw,0);
  row(raw,{});
  assert(!crossing.sample(0.35f,raw,1000));
  assert(crossing.crossingRecoveryBrake.active);
  crossing.capture(true,43);
  assert(crossing.distancePidRejoinGate.active);
}

int main() {
  testSideExit();
  testRecovery();
  std::cout << "PASS: side-marker/curve separation, 10-cm boundary, Kp threshold, "
               "immediate recovery, fresh PID, opposite corner and preserved modes\n";
}
'''
for key, value in {
    "__HELPERS__": helpers, "__EDGE_SELECTOR__": edge_selector,
    "__RECOVERY_SELECTOR__": recovery_selector,
    "__POLICY__": policy, "__WHITE_UPDATE__": white_update,
    "__CAPTURE__": capture,
}.items():
    harness = harness.replace(key, value)
with tempfile.TemporaryDirectory(prefix="line-distance-direct-") as tmp:
    cpp = Path(tmp) / "test.cpp"
    exe = Path(tmp) / "test.exe"
    cpp.write_text(harness, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", str(cpp), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
