void setup_robot() {
 
  robot.setMotorPWMFrequency(20000);  // ลอง 12 kHz ถ้าต้องการ เปลี่ยนความถี่ PWM สำหรับมอเตอร์ได้ที่นี่
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();
  gyro.setWire(Wire1);

  // หรือเลือก Wire
  // Wire.begin();
  // gyro.setWire(Wire);

  if (!gyro.begin()) {
    Serial.println("GYRO init failed!");
    //while (true) delay(10);
  }

  gyro.resetAngles(); 
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
    while(1);
  }
  
  encoder.setupEncoder();    //-------------------->> เรียกฟังก์ชัน setupEncoder
  encoder.resetEncoders();  //--------------------->> ฟังก์ชันรอก
  robot.run();    // แสดง Welcome + เมนู + รอคำสั่งrobot.ServoAttach(16);   // ติดตั้ง Servo บนพิน 16
}