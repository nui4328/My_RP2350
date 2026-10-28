#include <MyMINI_Pro.h>
int servo_down = 40;
int servo_L = 160;
int servo_R = 160;
void setup() 
  {
      robot_setup();
      set_line_error_monitor(true);   
      //arm_openDown();  // สำหรับประกอบหุ่นยนต์
      arm_closeDown(); // สำหรับแข่งขัน
      wait_button();
      // wait_button() has already selected real gyro or estimated heading.

      ////----------------------------------------------->>>
      ////----------------------------------------------->>>

        
      
        
      




      ////----------------------------------------------->>>
      ////----------------------------------------------->>>
  
////----------------------------------------------->>>
////----------------------------------------------->>>
  }   ///ห้ามลบ

void loop()
  {    
    // servo(18, 30); delay(1000);
    // servo(18, 120); delay(1000);
    //  delay(30);
  }

