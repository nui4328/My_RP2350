#include <Wire.h>
#include <my_BNO085.h>

my_BNO085 imu(0x4B, Wire); // Pico 2: I2C0 on GP4/GP5.

void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT_PULLUP);
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
  if (!imu.begin()) { // Game Rotation Vector: stable near robot motors.
    Serial.println("BNO085 not found");
    while (true) delay(500);
  }
  // Keep the robot still while collecting the optional gyro-rate offset.
  imu.calibrate();
  imu.resetAngles();
}

void loop() {
  static bool wasPressed = false;
  const bool pressed = digitalRead(2) == LOW;
  if (pressed && !wasPressed) imu.resetAngles();
  wasPressed = pressed;

  imu.update();
  if (!imu.angleReady()) {
    Serial.println("BNO085 heading unavailable");
    delay(10);
    return;
  }
  const float angle = imu.gyro('z');       // Continuous degrees, beyond 360.
  const float rate = imu.getAngle('z');    // Degrees/second.
  Serial.print("Z angle: ");
  Serial.print(angle, 2);
  Serial.print(" deg | Z rate: ");
  Serial.print(rate, 2);
  Serial.println(" deg/s");
  delay(50);
}
