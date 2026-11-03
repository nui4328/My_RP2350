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
uint32_t lastColorSendMs = 0;

int servo_down = 40;
int servo_L = 160;
int servo_R = 160;


void setup() 
  {
      robot_setup();
      set_line_error_monitor(true);   
      setup_color();
      // Test the TFT independently of whether a fresh TCS sample is ready.
      const bool greenSent = sendColorBands(String("GREEN"));
      Serial.println(greenSent ? F("COLOR I2C: GREEN test sent") :
                                 F("COLOR I2C: GREEN test failed; check ESP32 and I2C wiring"));
      lastColorSendMs = millis();
      //calibrateSensor(tcs0, "TCS0 (I2C0)");
      wait_button();
      // wait_button() has already selected real gyro or estimated heading.
      // Retry after START in case the ESP32 finished booting while we waited.
      sendColorBands(String("GREEN"));
      lastColorSendMs = millis();

       colorScanComplete = scanSixColorBands(10, 10, 0.250f, 0.0125f, 50.0f);
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
      //  }
      //  turnOffTft();  // Blank the TFT and stop repeated color packets.
      //  turnOnTft();   // Wake it and show the last sent order.


  }

