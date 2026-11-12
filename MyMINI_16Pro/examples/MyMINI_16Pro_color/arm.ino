void arm_openDown()
  {
    servoL(servo_L);
    servoR(servo_R);
    delay(300);
    servoUpDown(servo_down);
    delay(100);
  }
void arm_openUp()
  {
    servoL(servo_L);
    servoR(servo_R);
    delay(300);
    servoUpDown(servo_down+90);
    delay(100);
  }
void arm_closeDown()
  {
    servoL(servo_L-130);
    servoR(servo_R-130);
    delay(100);
    servoUpDown(servo_down);
    delay(300);
  }

void arm_closeUp()
  {
    servoL(servo_L-130);
    servoR(servo_R-130);
    delay(300);
    servoUpDown(servo_down+90);
    delay(100);
  }

void arm_minOpenUp()
  {
    servoL(servo_L-60);
    servoR(servo_R-60);
    delay(300);
    servoUpDown(servo_down+90);
    delay(100);
  }

void arm_upClose()
  {
    servoUpDown(servo_down+90);
    delay(300);
    servoL(servo_L-130);
    servoR(servo_R-130);
    delay(100);
    
  }

void servoL(int sl)
  {
    servo(18, sl);    // GP18: มุมซ้าย = 180° - sl
  }
void servoR(int sr)
  {
    servo(22, 180-sr);    // GP18: ตั้งมุมตามค่า sr
  }
void servoUpDown(int updown)
  {
    servo(28, 180-updown);   // GP28: มุมยก = 180° - updown
  }