"""Regression checks for gyro rotation stop decisions; no hardware required."""

from pathlib import Path


source = (Path(__file__).resolve().parents[1] / "src" / "LineFollower.cpp").read_text(
    encoding="utf-8"
)
spin = source.split("bool rotateWithGyro(", 1)[1].split(
    "bool rotatePivotToHeadingWithGyro(", 1
)[0]
pivot = source.split("bool rotatePivotToHeadingWithGyro(", 1)[1].split(
    "bool rotateWithFallback(", 1
)[0]

# Both paths must decide on the unfiltered BNO055 heading. The filtered yaw
# trails the moving robot by several samples at the library's 0.75 LPF alpha.
assert spin.count("imu.yawRaw()") == 2
assert pivot.count("imu.yawRaw()") == 2
assert "imu.yaw()" not in spin + pivot

# Signed travel rejects movement in the wrong direction. In particular, a
# pivot must finish after crossing the target, rather than drive the other
# wheel back toward an already-crossed heading.
assert "turnedDegrees += turnRight ? deltaYaw : -deltaYaw;" in spin
assert "turnedDegrees += turnRight ? deltaYaw : -deltaYaw;" in pivot
assert "const bool turnRight = initialError > 0.0f;" in pivot
assert "const bool turnRight = error > 0.0f;" not in pivot
assert "remainingDegrees <= ROTATE_TOLERANCE_DEG" in pivot
assert "completePivotRotation(backwardPivot, turnRight," in pivot


def first_stop_heading(use_filter: bool, direction: int) -> int:
    filtered = 0.0
    for travel in range(0, 121, 2):
        raw_yaw = direction * travel
        filtered = 0.75 * filtered + 0.25 * raw_yaw
        observed = filtered if use_filter else raw_yaw
        if direction * observed >= 90.0 - 1.5:
            return raw_yaw
    raise AssertionError("rotation never stopped")


for direction in (1, -1):
    old_stop = first_stop_heading(True, direction)
    new_stop = first_stop_heading(False, direction)
    assert abs(old_stop) > abs(new_stop)
    assert abs(new_stop) == 90
    print(f"{'right' if direction > 0 else 'left'}: filtered stop {old_stop}°, raw stop {new_stop}°")

# A sampled pivot that jumps from 87° to 93° has crossed 90°. The old
# error-sign controller commands reverse; signed progress completes it.
samples = (0.0, 80.0, 87.0, 93.0)
target = 90.0
old_command_after_crossing = 1 if target - samples[-1] > 0 else -1
new_remaining = target - sum(
    samples[i] - samples[i - 1] for i in range(1, len(samples))
)
assert old_command_after_crossing == -1
assert new_remaining <= 1.5
print("pivot crossing: old command reverses; new controller brakes")
print("PASS")
