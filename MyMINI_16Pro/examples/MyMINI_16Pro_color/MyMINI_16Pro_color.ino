#include <MyMINI_16Pro.h>

#include <Wire.h>
#include <my_TCS.h>

#define TCS34725_ADDRESS 0x29   // ปกติคือ 0x29 (ถ้าเป็น 0x39 ให้เปลี่ยนตรงนี้)

// สร้าง instance พร้อม offset แยก
my_TCS tcs0(0x0000);   // เซ็นเซอร์ตัวแรก บน I2C0 (GP4 SDA, GP5 SCL)

int servo_down = 40;
int servo_L = 160;
int servo_R = 160;


void setup() 
  {
      robot_setup();
      set_line_error_monitor(true);   
      setup_color();
      //arm_openDown();  // สำหรับประกอบหุ่นยนต์
      arm_closeDown(); // สำหรับแข่งขัน
      wait_button();
      // wait_button() has already selected real gyro or estimated heading.

      ////----------------------------------------------->>>
      ////----------------------------------------------->>>
       
       f_line(30, 30, 0.5f, cl,    1);
       wait_button();
       f_line(30, 30, 0.5f, cr,    1);
       wait_button();
       f_line(30, 30, 0.5f, cl&cr,    1);
        
      
        
      




      ////----------------------------------------------->>>
      ////----------------------------------------------->>>
  
////----------------------------------------------->>>
////----------------------------------------------->>>
  }   ///ห้ามลบ

void loop()
  {    
   read_color();
     //read_gyro();
     delay(30);

    // servo(18, 30); delay(1000);
    // servo(18, 120); delay(1000);
    //  delay(30);
  }

