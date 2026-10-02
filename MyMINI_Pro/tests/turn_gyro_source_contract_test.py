"""Check integration paths that cannot run without the robot hardware."""

from pathlib import Path

root = Path(__file__).resolve().parents[1]
line = (root / "src/LineFollower.cpp").read_text(encoding="utf-8")
robot = (root / "src/MyMINI_Pro.cpp").read_text(encoding="utf-8")

gyro = line.split("void runTurnGyro(", 1)[1].split("} // namespace", 1)[0]
failure = line.split("void failTurnGyro(", 1)[1].split("}", 1)[0]
rotate = gyro.index("if (phase == TurnPhase::Rotate)")
sensor_gate = gyro.index("if (!sensorFrame) continue;")
assert rotate < sensor_gate, "white/no sensor frame must not block gyro rotation"
assert "readTurnExit(" not in gyro, "gyro rotation must not use stopSensor"
assert "failTurnGyro(F(\"GYRO_OR_START_REFERENCE_NOT_READY\"))" in gyro
assert "TurnGyroTarget::timedOut(" in gyro
assert "failTurnGyro(F(\"TIMEOUT\"))" in gyro
assert "beginWithBrakeLead(" in gyro
assert "TURN_GYRO_BRAKE_LEAD_DEG = 20.0f" in line
assert "gyroPid.command(" not in gyro
assert "turnLineSearchSpeed" not in gyro[rotate:sensor_gate]
assert "turnFastTimeMs" not in gyro[rotate:sensor_gate]
assert "stopAfterTurnComplete(" not in gyro
assert "settleTurnGyro(" not in line
rotate_body = gyro[rotate:sensor_gate]
reached = rotate_body.index("if (result == TurnGyroResult::Reached)")
drive = rotate_body.index("motorMotion(initialLeftCommand, initialRightCommand)")
assert reached < drive
assert "motor(1, 1);" in rotate_body[reached:drive]
assert "motor(1, 1);" in failure
assert "motor(0, 0)" not in gyro

wait = robot.split("void wait_button()", 1)[1].split("void robot_begin()", 1)[0]
press = wait.split("if (calibrationManager.consumeShortPressEvent())", 1)[1]
assert "headingReferenceReady = false;" in wait
assert press.count("imu.resetAngles();") == 1
assert "imu.update()" in press
assert "headingReferenceReady = true;" in press
assert "imu.resetAngles();" not in gyro

print("turn_gyro source contract: PASS")
