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
      gyro_set();

      ////----------------------------------------------->>>
      ////----------------------------------------------->>>
      
      
      arm_closeDown();
       delay(1000);
      arm_closeUp();
      
        





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

