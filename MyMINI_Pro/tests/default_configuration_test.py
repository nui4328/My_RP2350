"""Check the MyMINI_Pro defaults used when a sketch only calls robot_begin()."""

from pathlib import Path
import re


src = Path(__file__).resolve().parents[1] / "src"


def text(name: str) -> str:
    return (src / name).read_text(encoding="utf-8")


def value(content: str, name: str) -> str:
    match = re.search(rf"\b{name}\s*=\s*([^;]+);", content)
    assert match, name
    return match.group(1).strip()


config = text("RobotConfig.h")
sensors = text("DualMuxSensors.h")
sensor_members = sensors.split("private:", 1)[1]
motors = text("MotorDriver.cpp")
line = text("LineFollower.cpp")
robot = text("MyMINI_Pro.cpp")

for name, expected in {
    "MUX_SETTLE_US": "40",
    "MUX_DISCARD_READS": "1",
    "MUX_AVERAGE_SAMPLES": "2",
    "MUX_SMOOTHING_DIVISOR": "4",
    "SERIAL_BAUD": "115200",
}.items():
    assert value(config, name) == expected, name

for name, expected in {
    "settleUs_": "MyMINIConfig::MUX_SETTLE_US",
    "discardReads_": "MyMINIConfig::MUX_DISCARD_READS",
    "averageReads_": "MyMINIConfig::MUX_AVERAGE_SAMPLES",
    "smoothingDivisor_": "MyMINIConfig::MUX_SMOOTHING_DIVISOR",
}.items():
    assert value(sensor_members, name) == expected, name

assert value(motors, "selectedDriver") == "MotorDriverType::DRV8874"
assert value(motors, "driverStatus") == "MotorDriverStatus::DRV8874_NOT_CONFIGURED"
assert "selectedDriver = type;" in motors
assert "driverStatus = MotorDriverStatus::DRV8874_READY;" in motors
begin = robot.split("void robot_begin() {", 1)[1].split("void robot_update()", 1)[0]
assert begin.index("Serial.begin(MyMINIConfig::SERIAL_BAUD);") < begin.index(
    "motorDriver.begin();") < begin.index("sensorArrays.begin();")

ratios = re.search(r"TurnMotorRatio turnMotorRatios\[\]\s*=\s*\{(.*?)\};", line, re.S)
assert ratios
assert re.findall(r"\{\s*(-?\d+)\s*,\s*(-?\d+)\s*\}", ratios.group(1)) == [
    ("-10", "100"), ("100", "-10"), ("-100", "100"),
    ("100", "-100"), ("-100", "100"), ("100", "-100"),
]
for name, expected in {
    "turnOvershootMs": "20",
    "turnTouchBrakeMs": "30",
    "turnTimeoutMs": "3000",
    "turnApproachKp": "0.150f",
    "turnSensorExitSpeed": "25",
    "turnDistanceExitSpeed": "25",
    "turnTouchBrake": "15",
    "turnLineSearchSpeed": "100",
    "turnFastTimeMs": "60",
    "lineRampMs": "LINE_FOLLOW_DEFAULT_RAMP_MS",
    "lineDecelRampMm": "20.0f",
    "lineKd": "LINE_KD",
}.items():
    assert value(line, name) == expected, name
assert value(line, "LINE_FOLLOW_DEFAULT_RAMP_MS") == "200"
assert "LINE_KD = 0.012f;" in text("LineFollower.h")

print("PASS: sensor, motor driver, line and turn defaults")
