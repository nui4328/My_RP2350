#include <Wire.h>
#include <my_BNO085.h>

// Pico 2: BNO085 SDA=GP4, SCL=GP5, I2C address 0x4B.
// This example does not drive motors or require Adafruit_BNO08x.
my_BNO085 imu(0x4B, Wire);
uint32_t lastPrintMs = 0;

void setup() {
  Serial.begin(115200);
  delay(1500);
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setTimeout(25, true);

  if (!imu.begin()) {
    Serial.println(F("BNO085 begin failed"));
    imu.printDiagnostics(Serial);
    return;
  }
  Serial.println(F("BNO085 ready"));
}

void loop() {
  imu.update();
  if (millis() - lastPrintMs >= 1000) {
    lastPrintMs = millis();
    Serial.print(F("ready=")); Serial.print(imu.angleReady() ? 1 : 0);
    Serial.print(F(" yaw=")); Serial.print(imu.yaw(), 2);
    Serial.print(F(" rateZ=")); Serial.print(imu.rate('z'), 2);
    Serial.print(' ');
    imu.printDiagnostics(Serial);
  }
  delay(10);
}
