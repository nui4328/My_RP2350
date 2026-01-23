

void release_box()
  {
  robot.ServoWrite(18, 150);   // ครั้งแรก: attach + ไป 60 องศา
  delay(100);
  robot.ServoWrite(18, 10);  // ครั้งต่อไป: ไป 120 องศา
  delay(1000);
  robot.ServoWrite(18, 150);    // ไป 0 องศา
  delay(100);

  }

///------------------------------------------------------------------------>>ตัวอย่าง

void servo()
  {
    //robot.robot.Motorupdate();  // เมื่อกด RUN จะทำงานที่นี่
  // ตัวอย่างควบคุม Servo
  robot.ServoWrite(16, 60);   // ครั้งแรก: attach + ไป 60 องศา
  delay(1000);
  robot.ServoWrite(16, 120);  // ครั้งต่อไป: ไป 120 องศา
  delay(1000);
  robot.ServoWrite(16, 0);    // ไป 0 องศา
  // ถ้าต้องการถอด
  // robot.robot.MotorServoDetach(16);
  }