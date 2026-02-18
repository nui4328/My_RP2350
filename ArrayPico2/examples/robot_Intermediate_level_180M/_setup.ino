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

  encoder.setupEncoder();    //-------------------->> เรียกฟังก์ชัน setupEncoder
  encoder.resetEncoders();  //--------------------->> ฟังก์ชันรอก
  robot.run();    // แสดง Welcome + เมนู + รอคำสั่งrobot.ServoAttach(16);   // ติดตั้ง Servo บนพิน 16
}