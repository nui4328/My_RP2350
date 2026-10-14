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
      if (!gyro_set()) {
        Serial.println(F("Gyro setup failed. Check BNO085 power, address and selected I2C pins."));
        for (;;) delay(1000);
      }

      ////----------------------------------------------->>>
      ////----------------------------------------------->>>
       arm_closeDown();
        

        

        





      ////----------------------------------------------->>>
      ////----------------------------------------------->>>
  
////----------------------------------------------->>>
////----------------------------------------------->>>
  }   ///ห้ามลบ

void loop() {
  
        f_line(50, 50, 1.75f, f0&15, 15);
        point_3_ring();
        f_line(50, 50, 1.75f, f0&15, 15);
        b_line(50, 50, 1.75f, 10, 15);
        turn(tl, 40, f6,  20);

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

