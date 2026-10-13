#include <MyMINI_Pro.h>
int servo_down = 45;
int servo_L = 160;
int servo_R = 160;
void setup() 
  {
      robot_setup();
      set_line_error_monitor(true);   
      arm_openDown();   
      wait_button();
      if (!gyro_set()) return;

      ////----------------------------------------------->>>
      ////----------------------------------------------->>>
      
      positoin_error = 80;  
      f_line(70, 70, 1.25f, gyro_angle(-135.0f), 10);
      if (!f_line_gyro_succeeded()) { motor(1, 1); return; }
      delay(1000);
      turn(tr,  60, f11, 5);
      positoin_error = 20;
      f_line(70, 70, 1.25f, gyro_angle(210.0f), 10);
      if (!f_line_gyro_succeeded()) { motor(1, 1); return; }
      
        





      ////----------------------------------------------->>>
      ////----------------------------------------------->>>
  
////----------------------------------------------->>>
////----------------------------------------------->>>
  }   ///ห้ามลบ

void loop() {


  //Serial.println(error, 2);
  delay(20);
  // motor(30, 30);    // Forward.
  // delay(700);
  // stopFor(400);

  // motor(-30, -30);  // Reverse.
  // delay(700);
  // stopFor(400);

  // motor(-30, 30);   // Turn left using the existing motor(left, right) contract.
  // delay(550);
  // stopFor(400);

  // motor(30, -30);   // Turn right.
  // delay(550);
  // stopFor(800);
}

