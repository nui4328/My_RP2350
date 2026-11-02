#include <Wire.h>
#include <SoftwareWire.h>
#include <my_TCS.h>

// This standalone sketch uses GPIOs assigned to line sensors and a servo in
// MyMINI_16Pro_color. Do not run the two pin assignments simultaneously.
SoftwareWire SoftWire(28, 1);
my_TCS tcs0(0x0000), tcs1(0x0040), tcs2(0x0080);
bool ready0 = false, ready1 = false, ready2 = false;

void waitForEnter() {
  while (Serial.available()) Serial.read();
  Serial.println("กด Enter เมื่อพร้อม...");
  while (!Serial.available()) delay(10);
  while (Serial.available()) Serial.read();
}

void calibrateSensor(my_TCS &sensor, const char *name, bool ready) {
  if (!ready) { Serial.print(name); Serial.println(" ไม่พบเซนเซอร์ ข้าม calibration"); return; }
  Serial.print("=== Calibrate "); Serial.print(name); Serial.println(" ===");
  sensor.startColorCalibration();
  const char *names[] = {"RED", "GREEN", "BLUE", "YELLOW", "WHITE", "BLACK"};
  for (unsigned i=0; i<6; ++i) {
    Serial.print("วางเซนเซอร์บนสี "); Serial.println(names[i]);
    waitForEnter();
    switch (i) {
      case 0: sensor.calibrateRed(); break;
      case 1: sensor.calibrateGreen(); break;
      case 2: sensor.calibrateBlue(); break;
      case 3: sensor.calibrateYellow(); break;
      case 4: sensor.calibrateWhite(); break;
      case 5: sensor.calibrateBlack(); break;
    }
  }
  if (!sensor.allColorsCalibrated()) {
    Serial.println("อ่านบางสีไม่สำเร็จ ไม่บันทึก EEPROM"); return;
  }
  sensor.calibrateDone();
  Serial.println(sensor.lastCalibrationSaveSucceeded()
      ? "บันทึก calibration สำเร็จ" : "บันทึกไม่ได้ ตรวจ EEPROM หรือข้อมูล Legacy");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Wire.setSDA(4); Wire.setSCL(5);
  Wire1.setSDA(26); Wire1.setSCL(27);
  Wire.setTimeout(25, true); Wire1.setTimeout(25, true);
  SoftWire.setTimeout(25);
  // begin() on each sensor initializes its bus once; calibration EEPROM stays on Wire.
  ready0=tcs0.begin(TCS34725_ADDRESS, &Wire);
  ready1=tcs1.begin(TCS34725_ADDRESS, &Wire1);
  ready2=tcs2.begin(TCS34725_ADDRESS, &SoftWire);
  Serial.print("TCS0: "); Serial.println(ready0 ? "OK" : "ไม่พบ");
  Serial.print("TCS1: "); Serial.println(ready1 ? "OK" : "ไม่พบ");
  Serial.print("TCS2: "); Serial.println(ready2 ? "OK" : "ไม่พบ");
  Serial.println("เริ่ม calibration ทั้งสามตัว (จะไม่บันทึกจนกด Enter ครบทุกสี)");
  calibrateSensor(tcs0,"TCS0",ready0);
  calibrateSensor(tcs1,"TCS1",ready1);
  calibrateSensor(tcs2,"TCS2",ready2);
}

void printColor(const char *name, my_TCS &sensor, bool ready) {
  Serial.print(name); Serial.print(": ");
  if (!ready) Serial.print("ไม่พบ");
  else if (!sensor.readFast()) Serial.print("อ่านไม่สำเร็จ/ข้อมูลยังไม่พร้อม");
  else Serial.print(sensor.getColor());
}
void loop() {
  printColor("TCS0",tcs0,ready0); Serial.print(" | ");
  printColor("TCS1",tcs1,ready1); Serial.print(" | ");
  printColor("TCS2",tcs2,ready2); Serial.println();
  delay(500);
}
