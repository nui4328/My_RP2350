// sketch_calibrate_tcs_dual.ino
// ตัวอย่าง calibrate สีสำหรับ TCS34725 สองตัว แยก I2C bus
// วันที่ปรับ: 16 ม.ค. 2026

#include <Wire.h>
#include <my_TCS.h>

#define TCS34725_ADDRESS 0x29   // ปกติคือ 0x29 (ถ้าเป็น 0x39 ให้เปลี่ยนตรงนี้)

// สร้าง instance พร้อม offset แยก
my_TCS tcs0(0x0000);   // เซ็นเซอร์ตัวแรก บน I2C0 (GP4 SDA, GP5 SCL)
my_TCS tcs1(0x0040);   // เซ็นเซอร์ตัวที่สอง บน I2C1 (GP26 SDA, GP27 SCL)

void calibrateSensor(my_TCS &sensor, const char* name) {
  Serial.print("\n=== Calibrate ");
  Serial.print(name);
  Serial.println(" ===");
  Serial.println("เตรียมแสงและวัตถุสีมาตรฐานให้คงที่ก่อนเริ่ม");

  // ฟังก์ชันย่อยสำหรับรอ Enter อย่างปลอดภัย
  auto waitForEnter = []() {
    Serial.println("   กด Enter เมื่อพร้อม...");
    
    // ล้าง buffer ก่อนรอ
    while (Serial.available() > 0) {
      Serial.read();  // อ่านและทิ้งทุกอย่างที่ค้างอยู่
    }

    // รอจนกว่าจะมีข้อมูลใหม่
    while (Serial.available() == 0) {
      delay(100);  // รอ 100ms แล้วเช็คใหม่ (ประหยัด CPU)
    }

    // อ่านและล้าง buffer ให้หมด (ป้องกัน \r\n หรือตัวอักษรอื่น)
    while (Serial.available() > 0) {
      Serial.read();
    }
  };

  waitForEnter();
  sensor.calibrateRed();
  Serial.println("   → Calibrate RED เสร็จ");

  waitForEnter();
  sensor.calibrateGreen();
  Serial.println("   → Calibrate GREEN เสร็จ");

  waitForEnter();
  sensor.calibrateBlue();
  Serial.println("   → Calibrate BLUE เสร็จ");

  waitForEnter();
  sensor.calibrateYellow();
  Serial.println("   → Calibrate YELLOW เสร็จ");

  waitForEnter();
  sensor.calibrateWhite();
  Serial.println("   → Calibrate WHITE เสร็จ");

  waitForEnter();
  sensor.calibrateBlack();
  Serial.println("   → Calibrate BLACK เสร็จ");

  sensor.calibrateDone();
  Serial.print(name);
  Serial.println(" Calibrate เสร็จสิ้น! ค่าเก็บใน EEPROM แล้ว");
}
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(500);

  Serial.println("=== TCS34725 Dual Calibration Tool ===");
  Serial.println("I2C0: GP4 SDA, GP5 SCL");
  Serial.println("I2C1: GP26 SDA, GP27 SCL");
  Serial.println("-------------------------------------");

  // ตั้งค่า I2C0
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();

  // ตั้งค่า I2C1
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();

  // เริ่มเซ็นเซอร์ตัวแรก
  Serial.print("เริ่มต้น TCS0 (I2C0): ");
  if (tcs0.begin(TCS34725_ADDRESS, &Wire)) {
    Serial.println("OK");
  } else {
    Serial.println("ไม่พบ! ตรวจสอบสาย I2C0");
    while(1);
  }

  // เริ่มเซ็นเซอร์ตัวที่สอง
  Serial.print("เริ่มต้น TCS1 (I2C1): ");
  if (tcs1.begin(TCS34725_ADDRESS, &Wire1)) {
    Serial.println("OK");
  } else {
    Serial.println("ไม่พบ! ตรวจสอบสาย I2C1");
    while(1);
  }

  // Calibrate เซ็นเซอร์ตัวแรกก่อน
 calibrateSensor(tcs0, "TCS0 (I2C0)");

  // Calibrate เซ็นเซอร์ตัวที่สอง
 calibrateSensor(tcs1, "TCS1 (I2C1)");

  Serial.println("\nการ calibrate ทั้งสองเซ็นเซอร์เสร็จสิ้น!");
  Serial.println("ครั้งต่อไปเปิดเครื่องใหม่จะโหลดค่า calibration อัตโนมัติ");
  Serial.println("พร้อมใช้งานแล้ว!");
}

void loop() {
  if (tcs0.readFast()) {
    Serial.print("TCS0: "); Serial.println(tcs0.getColor());
  }
  if (tcs1.readFast()) {
    Serial.print("TCS1: "); Serial.println(tcs1.getColor());
  }
  delay(500);
}