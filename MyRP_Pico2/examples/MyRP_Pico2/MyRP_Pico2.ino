#include <MyRP_Pico2.h>
MyRP_Pico2 robot;

int last_servo_28 = 90;
//----------------------------------------------------------------------------->> ตั้งค่ามือจับ
int servo_down = 125;        //-------------------->> ตั้งค่า มื่อจับลงขนานกับพื้น
int servoL_open = 117;      //-------------------->> ตั้งค่า กางฝ่ามือด้านซ้าย
int servoR_open = 120;      //-------------------->> ตั้งค่า กางฝ่ามือด้านขวา
//----------------------------------------------------------------------------->> ตั้งค่ามือจับ
//----------------------------------------------------------------------------->> ตั้งค่ามือจับ

void setup() 
  {
    robot.begin();
    set_motor();
    arm_down_open();
    robot.servo(28, 90);
    robot.sw();


    // robot.fline(40, 40, 0.45, "fl", 'f', 's', 60, "a1", 30);
    // arm_rotrate(125);
    //  arm_down_open();
    //  delay(200);
    //   robot.bline(40, 40, 0.45, "30", 'n', 's', 60, "a1", 30);
    //   arm_rotrate(90);
    //   arm_down_close();

    
     
 

    
  }

void loop() 
  {
    robot.Motor(50, 50);
    robot.servo(1, 20);
    robot.servo(10, 20);
    robot.servo(0, 20);
    delay(1000);
    robot.Motor(-50, -50);
    robot.servo(1, 120);
    robot.servo(10, 120);
    robot.servo(0, 120);
    delay(1000);
  //  arm_down_close(); delay(1000);
  //  arm_up_open();delay(1000);
   Serial.print( " my.gyro('z')  " );   Serial.print( robot.gyro('z')); 
    Serial.print( analogRead(26) );   Serial.print( "  26 " );   
    Serial.println( analogRead(27) ); Serial.print( "  27 " );   
    //Serial.println( analogRead(28) ); 
    delay(10);

  }
