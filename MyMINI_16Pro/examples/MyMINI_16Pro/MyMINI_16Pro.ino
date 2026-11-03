#include <MyMINI_16Pro.h>
#include <Wire.h>

int servo_down = 50;
int servo_L = 160;
int servo_R = 158;
void setup() 
  {
      robot_setup();
      set_line_error_monitor(true);
      // คาลิเบรต BNO055 ให้วางหุ่นยนต์นิ่ง 
      if (!gyro_calibrate(false)) Serial.println("Gyro calibration failed");   
      
      
      arm_openDown();  // สำหรับประกอบหุ่นยนต์
      //arm_closeUp(); // สำหรับแข่งขัน
      wait_button();
      // เรียนกใช้งานฟังก์ชัน คาริเบรท  gyro
      // wait_button() has already selected real gyro or estimated heading.

      ////----------------------------------------------->>>
      ////----------------------------------------------->>>
       
       
        f_line(40, 40, 0.55f, f15, 1);
        turn(tfl, 90, f4,  5); 
        f_line(40, 40, 0.55f, f15, 1);
        b_line(40, 40, 0.55f, b15, 1);
        turn(tcl, 50, f4,  5); 

      
        
      




      ////----------------------------------------------->>>
      ////----------------------------------------------->>>
  
////----------------------------------------------->>>
////----------------------------------------------->>>
  }   ///ห้ามลบ

void loop()
  {    
  //         if (tcs0.readFast()) {
  //   //Serial.print("TCS0: "); Serial.println(tcs0.getColor());
  //   //tcs0.printRaw();
  //   //Serial.println(tcs0.getColor());
  //   Serial.println(tcs0._Raw());
  // }
    read_gyro();
    //Serial.println(readADC_F());
     delay(30);

    // servo(18, 30); delay(1000);
    // servo(18, 120); delay(1000);
    //  delay(30);
  }

