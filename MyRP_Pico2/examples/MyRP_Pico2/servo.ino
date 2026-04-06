
void arm_ready()  //--------->> แขนลง กางฝ่ามือออกเตรียมเข้าไปคีบ
  {
    robot.servo(1, servo_down);
    robot.servo(10, servoL_open - 30);
    robot.servo(0, servoR_open - 30);
  }

void arm_open_down()   //--------->>  กางฝ่ามือออก  และเอาแขนลง
  {    
    robot.servo(10, servoL_open);
    robot.servo(0, servoR_open);
    delay(300);
    robot.servo(1, servo_down);    
  }
void arm_down_open()   //--------->>  เอาแขนลง  และ กางฝ่ามือออก  
  {
    robot.servo(1, servo_down);
    delay(300);
    robot.servo(10, servoL_open);
    robot.servo(0, servoR_open);
  }
void arm_open_up()    //--------->>  กางฝ่ามือออก  และยกแขนขึ้น
  {    
    robot.servo(10, servoL_open);
    robot.servo(0, servoR_open); 
    delay(300);
    robot.servo(1, servo_down+95);   
  }
void arm_up_open()   //--------->>  เอาแขนขึ้น  และ กางฝ่ามือออก  
  {
    robot.servo(1, servo_down+95);
    delay(300);
    robot.servo(10, servoL_open);
    robot.servo(0, servoR_open);
  }
void arm_down_close()  //--------->>  เอาแขนลง  และ หุบมือเข้า  
  {
    robot.servo(1, servo_down);
    delay(100);
    robot.servo(10, servoL_open - 95);
    robot.servo(0, servoR_open - 91);
  }

void arm_up_close()   //--------->>  ยกแขนขึ้น  และ หุบมือเข้า  
  {
    robot.servo(1, servo_down+95);
    delay(100);
    robot.servo(10, servoL_open - 95);
    robot.servo(0, servoR_open - 91);
  }
void arm_close_up()   //--------->>  ยกแขนขึ้น  และ หุบมือเข้า  
  {
    robot.servo(10, servoL_open - 95);
    robot.servo(0, servoR_open - 91);
    delay(100);
    robot.servo(1, servo_down+95); 
  }
void arm_big_box()   //--------->>  คีบกล่องใหญ่
  {
    robot.servo(1, servo_down);
    robot.servo(10, servoL_open - 30);
    robot.servo(0, servoR_open - 30);
  }

