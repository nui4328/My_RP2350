#include <MyMINI_Pro.h>
int servo_down = 40;
int servo_L = 160;
int servo_R = 160;
void setup() 
  {
      robot_setup();
      set_line_error_monitor(true);   
     // arm_openDown();  
      //arm_closeDown(); 
      wait_button();
      // wait_button() has already selected real gyro or estimated heading.

      ////----------------------------------------------->>>
      ////----------------------------------------------->>>

        servo(18, 130);
        delay(1000);
        servo(18, 60);

        
      




      ////----------------------------------------------->>>
      ////----------------------------------------------->>>
  
////----------------------------------------------->>>
////----------------------------------------------->>>
  }   ///ห้ามลบ

void loop()
  {    
    
     delay(30);
  }

