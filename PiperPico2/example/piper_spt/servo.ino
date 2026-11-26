void slow2_hand() {
  for (int i = 80; i >= 20; i--) {
    arm_hand('L', i);
    arm_hand('R', i);
    delay(5);
  }
}

void servo_begin() {
    arm_shoulder('L', arm_shoulderL_begin);  // ค่าเดิมคือ65
    arm_shoulder('R', arm_shoulderR_begin);

  arm_hand('L', 40);
  arm_hand('R', 40);

  // arm_can('L', 90);  // 140
  // arm_can('R', 90);
}

void move_mision_5cm() {
  arm_updown(0);
  arm_hand('L', 30);  // ฝ่ามือซ้ายหุบเข้า
  arm_hand('R', 30);
  arm_shoulder('L', 70);
  arm_shoulder('R', 70);
  delay(200);
  move(30, 30, 2.4, 30, 5);
}
void trim_servo_0(int trim) {
  trim_0 = trim;
}
void trim_servo_1(int trim) {
  trim_1 = trim;
}
void trim_servo_2(int trim) {
  trim_2 = trim;
}
void trim_servo_3(int trim) {
  trim_3 = trim;
}
void trim_servo_4(int trim) {
  trim_4 = trim;
}
void trim_servo_5(int trim) {
  trim_5 = trim;
}

void arm_can(char LR, int deg) {
  if (LR == 'L') {
    robot.setservo(4, deg + trim_4);
  } else if (LR == 'R') {
    robot.setservo(5, (180 - deg) + trim_5);
  }
}
void arm_hand(char LR, int deg) {
  if (LR == 'L') {
    robot.setservo(0, (180 - deg) + trim_0);
  } else if (LR == 'R') {
    robot.setservo(3, (180 - deg) + trim_3);
  }
}

void arm_shoulder(char LR, int deg) {
  if (LR == 'L' || LR == 'l') {
    robot.setservo(1, (180 - deg) - trim_1);
  } else if (LR == 'R' || LR == 'r') {
    robot.setservo(2, deg + trim_2);
  }
}


void slow_2hand() {
  for (int i = 80; i >= 20; i--) {
    arm_hand('L', i);
    arm_hand('R', i);
    delay(5);
  }
}

void arm_bridge()  ////////////////////// มือและแขนหุบเข้าเพื่อข้ามสะพาน
{
  arm_hand('L', 40);
  arm_hand('R', 40);
  arm_shoulder('L', 10);
  arm_shoulder('R', 10);

}
