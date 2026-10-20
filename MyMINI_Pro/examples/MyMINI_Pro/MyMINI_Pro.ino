#include <MyMINI_Pro.h>
int servo_down = 40;
int servo_L = 160;
int servo_R = 160;
void setup() 
  {
      robot_setup();
      set_line_error_monitor(true);   
     // arm_openDown();  
      arm_closeDown(); 
      wait_button();
      // wait_button() has already selected real gyro or estimated heading.

      ////----------------------------------------------->>>
      ////----------------------------------------------->>>

for(int i=0; i<10; i++)
  {
      f_line(65, 65, 0.95f, 100.0f, 0, crossing_mode);
      f_line(30, 30, 0.35f, f15,    1);
      turn(tl,  60, f4,  5); 
      f_line(60, 60, 0.975f, 100.0f, 0, crossing_mode);
      f_line(30, 30, 0.35f, f0,    1);
       b_line(30, 30, 0.35f, 5,    1);
       turn(tl,  60, f4,  5); 
  } 
        

        
      




      ////----------------------------------------------->>>
      ////----------------------------------------------->>>
  
////----------------------------------------------->>>
////----------------------------------------------->>>
  }   ///ห้ามลบ

void loop() {
  
        

   float yaw = 0.0f, rateX = 0.0f, rateY = 0.0f, rateZ = 0.0f;
   if (gyro_read(yaw, rateX, rateY, rateZ)) {
     Serial.print(F("Yaw (deg): "));
     Serial.print(yaw, 2);
     Serial.print(F(" | rate z (deg/s): "));
     Serial.println(rateZ, 2);
   } else {
     Serial.println(F("Gyro read failed"));
   }
  //Serial.println(error, 2);
  delay(100);
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

