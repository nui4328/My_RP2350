bool scanColorSensorReady = false;

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
    if (tcs0.setConfig(TCS_IT_50MS, TCS_GAIN_4X)) {
      scanColorSensorReady = true;
      Serial.println("OK; integration 50 ms, gain 4x");
      tcs0.printCalibration(); // Must be calibrated at these same settings.
    } else {
      Serial.println("ตั้งเวลา/เกน TCS ไม่สำเร็จ");
    }
  } else {
    Serial.println("ไม่พบ! ตรวจสอบสาย I2C0");
    //while(1);
  }
  }
void calibrateSensor(my_TCS &sensor, const char* name) {
  if (!scanColorSensorReady) {
    Serial.println("TCS ยังไม่พร้อม: ยกเลิกการคาริเบรท");
    return;
  }
  Serial.print("\n=== Calibrate ");
  Serial.print(name);
  Serial.println(" ===");
  Serial.println("เตรียมแสงและวัตถุสีมาตรฐานให้คงที่ก่อนเริ่ม");
  sensor.startColorCalibration();
  bool saturatedDuringCalibration = false;
  auto checkSaturation = [&](const char* colorName) {
    if (sensor.c < 21504u) return;
    saturatedDuringCalibration = true;
    Serial.print("TCS อิ่มตัวขณะคาริเบรท ");
    Serial.print(colorName);
    Serial.println("; ลดแสงหรือ gain แล้วคาริเบรทใหม่");
  };
  Serial.println("  เริ่มคาริเบรท สีแดง --->>");

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
  checkSaturation("RED");
  Serial.println("   → Calibrate RED เสร็จ");
  Serial.println("   → Calibrate GREEN ");
  waitForEnter();
  sensor.calibrateGreen();
  checkSaturation("GREEN");
  Serial.println("   → Calibrate GREEN เสร็จ");
  Serial.println("   → Calibrate BLUE ");
  waitForEnter();
  sensor.calibrateBlue();
  checkSaturation("BLUE");
  Serial.println("   → Calibrate BLUE เสร็จ");
  Serial.println("   → Calibrate YELLOW ");
  waitForEnter();
  sensor.calibrateYellow();
  checkSaturation("YELLOW");
  Serial.println("   → Calibrate YELLOW เสร็จ");
  Serial.println("   → Calibrate WHITE ");
  waitForEnter();
  sensor.calibrateWhite();
  checkSaturation("WHITE");
  Serial.println("   → Calibrate WHITE เสร็จ");
   Serial.println("   → Calibrate BLACK  ");
  waitForEnter();
  sensor.calibrateBlack();
  checkSaturation("BLACK");
  Serial.println("   → Calibrate BLACK เสร็จ");

  if (!sensor.allColorsCalibrated() || saturatedDuringCalibration) {
    Serial.println("อ่านบางสีไม่สำเร็จหรือ TCS อิ่มตัว: ไม่บันทึก EEPROM");
    sensor.loadColorCalibration();
    return;
  }

  sensor.calibrateDone();
  if (!sensor.lastCalibrationSaveSucceeded()) {
    Serial.println("บันทึก EEPROM ไม่สำเร็จ: ตรวจสายและ EEPROM address 0x50");
    sensor.loadColorCalibration();
    return;
  }

  sensor.loadColorCalibration();
  if (!sensor.allColorsCalibrated()) {
    Serial.println("อ่านค่าจาก EEPROM กลับมาไม่ครบ: ตรวจ EEPROM");
    return;
  }
  Serial.print(name);
  Serial.println(" บันทึกและโหลดค่าคาลิเบรทจาก EEPROM สำเร็จ");
  sensor.printCalibration();
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
