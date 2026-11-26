#include <PiperPico2.h>
PiperPico2 robot(PiperPico2::MotorDriver::DRV8874);
void setup() {
  robot.setMotorDriver('B', PiperPico2::MotorDriver::LEGACY);
  robot.setMotorDriver('C', PiperPico2::MotorDriver::DRV8874, true);
  robot.setServoPulseRangeUs(0, 1000, 2000);
  robot.setServoPulseRange(1, 120, 600);
  if (!robot.begin()) return;
  robot.setservo(0, 90);
  robot.setI2CClock(100000);
  robot.motor('A', 30); robot.motor('A', 40); robot.motor('A', -30);
  robot.stopMotors(); robot.resetEncoders();
  robot.updateBattery(); robot.getBatteryVoltage(); robot.bat_control();
  robot.playToneAsync(1000, 10);
  robot.startCalibration(100); robot.updateCalibration(); robot.cancelCalibration();
}
void loop() { robot.enc1Left(); robot.enc1Right(); robot.enc2Left(); robot.enc2Right(); }
