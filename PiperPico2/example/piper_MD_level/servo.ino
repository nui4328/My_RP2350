void open_servo()
  {
    robot.setServo(0, 160); delay(200);
    robot.setServo(0, 60); delay(500);
    robot.setServo(0, 160); delay(200);
  }