#include <Wire.h>
#include "my_TCS.h"

#define TCS34725_ADDRESS 0x29

my_TCS tcs0(0x0000);
my_TCS tcs1(0x0040);

void calibrateSensor(my_TCS &sensor, const char* name) {
  Serial.print("\n=== Calibrate ");
  Serial.print(name);
  Serial.println(" ===");

  auto waitForEnter = []() -> bool {
    Serial.println("   กด Enter เมื่อพร้อม... (หรือ 's' เพื่อข้าม)");
    while (Serial.available() == 0) delay(100);
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.equalsIgnoreCase("s")) {
      Serial.println("   → ข้าม");
      return false;
    }
    return true;
  };

  if (waitForEnter()) { sensor.calibrateRed();    Serial.println("   → RED");    sensor.printRaw(); }
  if (waitForEnter()) { sensor.calibrateGreen();  Serial.println("   → GREEN");  sensor.printRaw(); }
  if (waitForEnter()) { sensor.calibrateBlue();   Serial.println("   → BLUE");   sensor.printRaw(); }
  if (waitForEnter()) { sensor.calibrateYellow(); Serial.println("   → YELLOW"); sensor.printRaw(); }
  if (waitForEnter()) { sensor.calibrateWhite();  Serial.println("   → WHITE");  sensor.printRaw(); }
  if (waitForEnter()) { sensor.calibrateBlack();  Serial.println("   → BLACK");  sensor.printRaw(); }

  sensor.calibrateDone();
  sensor.printCalibration();
  Serial.print(name);
  Serial.println(" เสร็จสิ้น");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(500);

  Serial.println("=== TCS34725 Dual Calibration Tool ===");
  Serial.println("EEPROM ใช้ตัวเดียว (I2C0), แยก offset");

  Wire.setSDA(4);   Wire.setSCL(5);   Wire.begin();
  Wire1.setSDA(26); Wire1.setSCL(27); Wire1.begin();

  Serial.print("TCS0 (I2C0): ");
  if (tcs0.begin(TCS34725_ADDRESS, &Wire)) {
    tcs0.setConfig(TCS_IT_101MS, TCS_GAIN_16X);
    Serial.println("OK");
  } else {
    Serial.println("ไม่พบ"); while(1);
  }

  Serial.print("TCS1 (I2C1): ");
  if (tcs1.begin(TCS34725_ADDRESS, &Wire1)) {
    tcs1.setConfig(TCS_IT_101MS, TCS_GAIN_16X);
    Serial.println("OK");
  } else {
    Serial.println("ไม่พบ"); while(1);
  }
  
  Serial.println("\n=== สถานะหลังโหลด calibration ===");
  Serial.print("TCS0 calibrated: "); Serial.println(tcs0.isCalibrated() ? "YES" : "NO");
  Serial.print("TCS1 calibrated: "); Serial.println(tcs1.isCalibrated() ? "YES" : "NO");

  Serial.println("\nTCS0 Calibration Details:");
  tcs0.printCalibration();

  Serial.println("\nTCS1 Calibration Details:");
  tcs1.printCalibration();

  Serial.println("\nต้องการ calibrate ใหม่? (y/n)");
  while (!Serial.available()) delay(50);
  char ch = Serial.read();
  if (ch == 'y' || ch == 'Y') {
    calibrateSensor(tcs0, "TCS0");
    calibrateSensor(tcs1, "TCS1");
  }

  Serial.println("\nพร้อมใช้งาน");
}

void loop() {
  if (tcs0.readFast()) {
    Serial.print("TCS0: "); Serial.print(tcs0.getColor());
    Serial.printf("  (C:%u R:%u G:%u B:%u)\n", tcs0.c, tcs0.r, tcs0.g, tcs0.b);
  }
  if (tcs1.readFast()) {
    Serial.print("TCS1: "); Serial.print(tcs1.getColor());
    Serial.printf("  (C:%u R:%u G:%u B:%u)\n", tcs1.c, tcs1.r, tcs1.g, tcs1.b);
  }
  delay(400);
}