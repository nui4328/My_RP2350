void servo()
  {
    //robot.update();  // เมื่อกด RUN จะทำงานที่นี่
  // ตัวอย่างควบคุม Servo
  robot.ServoWrite(16, 60);   // ครั้งแรก: attach + ไป 60 องศา
  delay(1000);
  robot.ServoWrite(16, 120);  // ครั้งต่อไป: ไป 120 องศา
  delay(1000);
  robot.ServoWrite(16, 0);    // ไป 0 องศา
  // ถ้าต้องการถอด
  // robot.ServoDetach(16);
  }
// ฟังก์ชันแสดงแรงดันไฟบน OLED

