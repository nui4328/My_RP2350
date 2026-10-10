#include <MyMINI_Pro.h>

void setup() 
  {
      robot_setup();
      set_line_error_monitor(true);
      arm_Upclose();
      wait_button();
      set_gyro();

      ////----------------------------------------------->>>
      ////----------------------------------------------->>>
      
      miss_01();





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

