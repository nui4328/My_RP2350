#include <MyMINI_16Pro.h>

#include <Wire.h>
#include <my_TCS.h>

#define TCS34725_ADDRESS 0x29   // ปกติคือ 0x29 (ถ้าเป็น 0x39 ให้เปลี่ยนตรงนี้)

// สร้าง instance พร้อม offset แยก
my_TCS tcs0(0x0000);   // เซ็นเซอร์ตัวแรก บน I2C0 (GP4 SDA, GP5 SCL)

int servo_down = 40;
int servo_L = 160;
int servo_R = 160;
int ch_red, ch_green, ch_blue, ch_yellow, ch_white, ch_black;   // ค่าเริ่มต้นเป็น 0
int ch_1, ch_2, ch_3, ch_4, ch_5, ch_6;   // ค่าเริ่มต้นเป็น 0
int ch_1p, ch_2p, ch_3p, ch_4p, ch_5p, ch_6p;   // ค่าเริ่มต้นเป็น 0
String _end;


void calibrateSensor(my_TCS &sensor, const char* name) {
  Serial.print("\n=== Calibrate ");
  Serial.print(name);
  Serial.println(" ===");
  Serial.println("เตรียมแสงและวัตถุสีมาตรฐานให้คงที่ก่อนเริ่ม");

  // ฟังก์ชันย่อยสำหรับรอ Enter อย่างปลอดภัย
  auto waitForEnter = []() {
    Serial.println("   กด Enter เมื่อพร้อม...");
    
    // ล้าง buffer ก่อนรอ
    while (Serial.available() > 0) {
      Serial.read();  // อ่านและทิ้งทุกอย่างที่ค้างอยู่
    }

    // รอจนกว่าจะมีข้อมูลใหม่
    while (Serial.available() == 0) {
      delay(100);  // รอ 100ms แล้วเช็คใหม่ (ประหยัด CPU)
    }

    // อ่านและล้าง buffer ให้หมด (ป้องกัน \r\n หรือตัวอักษรอื่น)
    while (Serial.available() > 0) {
      Serial.read();
    }
  };
  Serial.println("   → Calibrate RED ");
  waitForEnter();
  sensor.calibrateRed();
  Serial.println("   → Calibrate RED เสร็จ");
  Serial.println("   → Calibrate GREEN ");
  waitForEnter();
  sensor.calibrateGreen();
  Serial.println("   → Calibrate GREEN เสร็จ");
  Serial.println("   → Calibrate BLUE ");
  waitForEnter();
  sensor.calibrateBlue();
  Serial.println("   → Calibrate BLUE เสร็จ");
  Serial.println("   → Calibrate YELLOW ");
  waitForEnter();
  sensor.calibrateYellow();
  Serial.println("   → Calibrate YELLOW เสร็จ");
  Serial.println("   → Calibrate WHITE ");
  waitForEnter();
  sensor.calibrateWhite();
  Serial.println("   → Calibrate WHITE เสร็จ");
   Serial.println("   → Calibrate BLACK  ");
  waitForEnter();
  sensor.calibrateBlack();
  Serial.println("   → Calibrate BLACK เสร็จ");

  sensor.calibrateDone();
  Serial.print(name);
  Serial.println(" Calibrate เสร็จสิ้น! ค่าเก็บใน EEPROM แล้ว");
}

int servo_down = 40;
int servo_L = 160;
int servo_R = 160;
void setup() 
  {
      robot_setup();
      set_line_error_monitor(true);   

      Serial.println("=== TCS34725 Dual Calibration Tool ===");
  Serial.println("I2C0: GP4 SDA, GP5 SCL");
  Serial.println("-------------------------------------");

  // ตั้งค่า I2C0
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();

  // เริ่มเซ็นเซอร์ตัวแรก
  Serial.print("เริ่มต้น TCS0 (I2C0): ");
  if (tcs0.begin(TCS34725_ADDRESS, &Wire)) {
    Serial.println("OK");
  } else {
    Serial.println("ไม่พบ! ตรวจสอบสาย I2C0");
    //while(1);
  }
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
          if (tcs0.readFast()) {
    //Serial.print("TCS0: "); Serial.println(tcs0.getColor());
    //tcs0.printRaw();
    //Serial.println(tcs0.getColor());
    Serial.println(tcs0._Raw());
  }
     read_gyro();
     delay(30);

    // servo(18, 30); delay(1000);
    // servo(18, 120); delay(1000);
    //  delay(30);
  }

