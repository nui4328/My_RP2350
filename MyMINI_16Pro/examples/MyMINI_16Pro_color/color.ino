
void setup_color()
  {
    
      Serial.println("=== TCS34725 Dual Calibration Tool ===");
  Serial.println("I2C0: GP4 SDA, GP5 SCL");
  Serial.println("-------------------------------------");

  // ตั้งค่า I2C0
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();

  // เริ่มเซ็นเซอร์ตัวแรก
  Serial.print("เริ่มต้น TCS0 (I2C0): ");
  if (tcs0.begin(TCS34725_ADDRESS, &Wire)) {
    Serial.println("OK");
  } else {
    Serial.println("ไม่พบ! ตรวจสอบสาย I2C0");
    //while(1);
  }
  }
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
  Serial.println("   → Calibrate RED ");
  waitForEnter();
  sensor.calibrateRed();
  Serial.println("   → Calibrate RED เสร็จ");
  Serial.println("   → Calibrate GREEN ");
  waitForEnter();
  sensor.calibrateGreen();
  Serial.println("   → Calibrate GREEN เสร็จ");
  Serial.println("   → Calibrate BLUE ");
  waitForEnter();
  sensor.calibrateBlue();
  Serial.println("   → Calibrate BLUE เสร็จ");
  Serial.println("   → Calibrate YELLOW ");
  waitForEnter();
  sensor.calibrateYellow();
  Serial.println("   → Calibrate YELLOW เสร็จ");
  Serial.println("   → Calibrate WHITE ");
  waitForEnter();
  sensor.calibrateWhite();
  Serial.println("   → Calibrate WHITE เสร็จ");
   Serial.println("   → Calibrate BLACK  ");
  waitForEnter();
  sensor.calibrateBlack();
  Serial.println("   → Calibrate BLACK เสร็จ");

  sensor.calibrateDone();
  Serial.print(name);
  Serial.println(" Calibrate เสร็จสิ้น! ค่าเก็บใน EEPROM แล้ว");
}

void read_color()
  {
           if (tcs0.readFast()) {
    //Serial.print("TCS0: "); Serial.println(tcs0.getColor());
    //tcs0.printRaw();
    Serial.print("color: ");Serial.print(tcs0.getColor());
    Serial.print("   C: ");Serial.println(tcs0._Raw());
  }
  }