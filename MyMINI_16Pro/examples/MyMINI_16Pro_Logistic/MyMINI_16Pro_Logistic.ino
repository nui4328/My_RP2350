#include <MyMINI_16Pro.h>

#include <Wire.h>
#include <my_TCS.h>
#include "color_i2c_send.h"

#define TCS34725_ADDRESS 0x29   // ปกติคือ 0x29 (ถ้าเป็น 0x39 ให้เปลี่ยนตรงนี้)

// สร้าง instance พร้อม offset แยก
my_TCS tcs0(0x0128);   // EEPROM 0x01A8-0x01CC: new 50 ms / 4x color calibration
bool scanSixColorBands(int leftSpeed, int rightSpeed, float kp, float kd,
                       float maxDistanceCm);
bool colorScanComplete = false;
// Band 1..6 from left to right. Filled only after a complete scan.
String order_color[6] = {};
String can_color[6] = {};
String _can_color;
uint32_t lastColorSendMs = 0;

int servo_down = 40;
int servo_L = 160;
int servo_R = 160;


int empty = 0;
int Empty = 0;

int C1 = 0;
int C2 = 0;
int C3 = 0;
int C4 = 0;
int C5 = 0;
int C6 = 0;

int lastCan = 0;



void setup() 
  {
      robot_setup();
      set_line_error_monitor(true);   
      setup_color();
      // A fixed test color does not depend on a fresh TCS conversion.
      if (!sendColorBands(String("GREEN")))
        Serial.println(F("COLOR I2C: GREEN test failed; check ESP32 and I2C wiring"));
      lastColorSendMs = millis();
      

      pinMode(1, INPUT_PULLUP);
      set_line_error_monitor(true);  
      servo(18, 50);
      griper_set();
      // Retry after the gripper setup if the ESP32 was still booting.
      sendColorBands(String("GREEN"));
      
      //calibrateSensor(tcs0, "TCS0 (I2C0)");
      wait_button();
      // wait_button() has already selected real gyro or estimated heading.
      // sendColorBands(String("GREEN"));
      // lastColorSendMs = millis();

       
      ////////////////////////////////////////////////////////////////////////////////////////---------------->>>>>>>>>>>>> 
     
      servo(28, 10);
      delay(200);
      f_line(20, 20, 0.0f, 10.0f, 0); 
      positoin_error = 70; 
      f_line(20, 20, 0.25f, cl, 0);
      positoin_error = 70; 
      f_line(20, 20, 0.25f, b0, 5);
      delay(200);
      start_robot();
      delay(1000);
      positoin_error = 70; 
      f_line(40, 40, 0.5f, f0, 1);
      turn(tfl, 90, f5,  5);
      run();
      
      // emptyOrder();

      getCan();
      //  GC1();
      // drop();
      // Order1C1();
      // getCan();
      // emptyOrder();

      // Order1C1();

       //run();

      // C1_T_C2();
      
        
      




      ////----------------------------------------------->>>
      ////----------------------------------------------->>>
  
////----------------------------------------------->>>
////----------------------------------------------->>>
  }   ///ห้ามลบ

void loop()
  {    
    //Repeat a confirmed order so a restarted ESP32 display can recover.
    if (millis() - lastColorSendMs >= 2000u) {
      resendColorBandsIfEnabled();
      lastColorSendMs = millis();
    }
  //read_color();
  //    //read_gyro();
  //    delay(30);

    // servo(18, 30); delay(1000);
    // servo(18, 120); delay(1000);
    //  delay(30);

      //   if (tcs0.readFast()) {
      //    String color = tcs0.getColor();
      //   //  String color = "GREEN";
      //    sendColorBands(color);
      //      Serial.println(color);
      //  }
      //  turnOffTft();  // Blank the TFT and stop repeated color packets.
      //  turnOnTft();   // Wake it and show the last sent order.


  }


void start_robot()
  {
    colorScanComplete = scanSixColorBands(10, 10, 0.150f, 0.01f, 55.0f);
       //                  L   R      Kp       Kd      max cm from band 1
       if (colorScanComplete) {
         publishScannedColors();
         lastColorSendMs = millis();
       } else {
         Serial.println(F("COLOR I2C: scan incomplete; no order sent"));
       }

      for(int i=0; i<6; i++)
        {
          Serial.print(order_color[i]);
          Serial.print("  ");
        }
  }
