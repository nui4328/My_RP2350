void calibrateSensor(my_TCS &sensor, const char* name) {
  Serial.print("\n=== Calibrate ");
  Serial.print(name);
  Serial.println(" ===");
  Serial.println("เตรียมแสงและวัตถุสีมาตรฐานให้คงที่ก่อนเริ่ม");
  Serial.println("   → Calibrate RED ");
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
  Serial.println("   → Calibrate GREEN ต่อ");

  waitForEnter();
  sensor.calibrateGreen();
  Serial.println("   → Calibrate GREEN เสร็จ");
  Serial.println("   → Calibrate BLUE ต่อ");

  waitForEnter();
  sensor.calibrateBlue();
  Serial.println("   → Calibrate BLUE เสร็จ");
  Serial.println("   → Calibrate YELLOW ต่อ");

  waitForEnter();
  sensor.calibrateYellow();
  Serial.println("   → Calibrate YELLOW เสร็จ");
  Serial.println("   → Calibrate WHITE ต่อ");

  waitForEnter();
  sensor.calibrateWhite();
  Serial.println("   → Calibrate WHITE เสร็จ");
  Serial.println("   → Calibrate BLACK ต่อ");

  waitForEnter();
  sensor.calibrateBlack();
  Serial.println("   → Calibrate BLACK เสร็จ");




  sensor.calibrateDone();
  Serial.print(name);
  Serial.println(" Calibrate เสร็จสิ้น! ค่าเก็บใน EEPROM แล้ว");

  while(1)
    {
      if (tcs0.readFast()) {
      Serial.print("TCS0: "); Serial.println(tcs0.getColor());
     }
    }
}