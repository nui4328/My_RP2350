// UI.ino - จัดการเมนูและแสดงผล OLED 128x32 ทั้งหมด (ปรับ UI ให้สม่ำเสมอด้วย invert selected item)

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C   // เปลี่ยนเป็น 0x3D ถ้าไม่ขึ้น
#define TCS34725_ADDRESS 0x29 // กำหนดที่นี่ ถ้า my_TCS.h ไม่มี

#include <my_TCS.h>

extern my_TCS tcs0;
extern my_TCS tcs1;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ตัวแปรสำหรับเมนู (global)
int currentMenu = 0;
int currentSubMode = 0;       // 1 = อยู่ใน ADC sub-menu
int selectedItem = 0;
unsigned long buttonPressTime = 0;
bool buttonWasPressed = false;
unsigned long lastBeepTime = 0;

// ข้อมูลเมนู
const char* mainMenu[] = {"RUN", "READSENSOR", "CALIBRATESENSOR"};
const int mainMenuCount = 3;

const char* readSensorMenu[] = {"ADC", "COLOR", "GYRO"};
const int readSensorMenuCount = 3;

const char* calibrateMenu[] = {"ADC-Cal", "COLOR-Cal"};
const int calibrateMenuCount = 2;

const char* adcSubMenu[] = {"adcRead", "adcMD", "adcMax", "adcMin"};
const int adcSubMenuCount = 4;



void opt_begin()
  {
      Serial.begin(115200);
      robot.begin();
      Wire.begin();
      Wire.setClock(400000);
      gyro.setWire(Wire);
      if (imu.begin(0x29, Wire)) 
        {
          Connected_gyro = true;
        }
      // //gyro.begin();
      // Wire.begin();
      // Wire.setClock(400000);

      // if (!imu.begin(0x29, Wire)) 
      //   {
      //     Serial.println("BNO055 not found");
      //     while (1);
      //   }
        
      Serial.println("BNO055 Ready");
      pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
      pinMode(echoPin, INPUT);   // Sets the echoPin as an Input
      initOLED();  // แสดง splash 1 วินาที + init จอ

      // หรือเลือก Wire
      //Wire1.begin();
      // gyro.setWire(Wire);
      encoder1.setupEncoder();    //-------------------->> เรียกฟังก์ชัน setupEncoder
      encoder1.resetEncoders();  //--------------------->> ฟังก์ชันรอก
      encoder2.setupEncoder();    //-------------------->> เรียกฟังก์ชัน setupEncoder
      encoder2.resetEncoders();  //--------------------->> ฟังก์ชันรอก
      //robot.showVoltageUntilButton();  // หรืออะไรก็ได้
      pinMode(2, INPUT_PULLUP);

     // ตั้งค่าก่อนเรียก turnGyro
        set_move_pid(2.35, 0.015, 0.042);  // PID สำหรับแข่ง
        

        // ตั้งค่าแรงมอเตอร์ แยกซ้าย-ขวา
        set_turnL_motor_bias(1.0, 1.0);  // หมุนซ้าย: มอเตอร์ A แรงขึ้น, มอเตอร์ C อ่อนลง//set_turnL_motor_bias(0.6, 1.2);   // หมุนซ้าย: มอเตอร์ A อ่อนลง, มอเตอร์ C แรงขึ้น
        set_turnR_motor_bias(1.0, 1.0);  // หมุนขวา: มอเตอร์ A อ่อนลง, มอเตอร์ C แรงขึ้น
  }
// ฟังก์ชันช่วยแสดงเมนูแบบ invert (ใช้ซ้ำทุกเมนู)
void drawMenuHeader(const char* title) {
  display.setCursor(0, 0);
  display.println(title);
  //display.drawFastHLine(0, 7, SCREEN_WIDTH, SSD1306_WHITE);
}

void drawMenuItem(int y, const char* text, bool isSelected) {   // กรอบสีขาว
  if (isSelected) {
    display.fillRect(0, y - 1, SCREEN_WIDTH, 8, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  }
  display.setCursor(4, y);  // เว้นขอบซ้ายเล็กน้อย
  display.print(text);
  if (isSelected) {
    display.setTextColor(SSD1306_WHITE);
  }
}

// เริ่มต้น OLED + Splash + calibrate gyro + เซ็นเซอร์สีทั้งหมด
void initOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED failed - check wiring"));
    return;
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  String text = "PIPER";
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  int x = (SCREEN_WIDTH - w) / 2;
  display.setCursor(x, 4);
  display.println(text);

  text = "ROBOTECH";
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  x = (SCREEN_WIDTH - w) / 2;
  display.setCursor(x, 18);
  display.println(text);

  display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.display();

  delay(1000);

  robot.playTone(3000, 70); delay(70);
  robot.playTone(3000, 70); delay(70);

  
  if(Connected_gyro == true)
    {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(20, 10);
      display.println("calibrateGYRO");
      display.display();
      imu.setLPF(0.75f);
      imu.calibrate(11, false);
      imu.resetAngles();
      //gyro.calibrate();  // calibrate gyro
    }
  else{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(20, 10);
    display.println("NONE GYRO");
    display.display();
  }

  delay(1000);

  display.clearDisplay();
  display.display();

  // เริ่มต้นเซ็นเซอร์สี (TCS34725 x2)
  Serial.println("\n=== Initializing TCS34725 Sensors ===");

  Wire.setSDA(4);   Wire.setSCL(5);   Wire.begin();
  Wire1.setSDA(26); Wire1.setSCL(27); Wire1.begin();

  Serial.print("TCS0 (I2C0): ");
  if (tcs0.begin(TCS34725_ADDRESS, &Wire)) {
    tcs0.setConfig(TCS_IT_101MS, TCS_GAIN_16X);
    Serial.println("OK");
  } else {
    Serial.println("ไม่พบ TCS0");
  }

  Serial.print("TCS1 (I2C1): ");
  if (tcs1.begin(TCS34725_ADDRESS, &Wire1)) {
    tcs1.setConfig(TCS_IT_101MS, TCS_GAIN_16X);
    Serial.println("OK");
  } else {
    Serial.println("ไม่พบ TCS1");
  }

  Serial.println("\n=== สถานะหลังโหลด calibration ===");
  Serial.print("TCS0 calibrated: "); Serial.println(tcs0.isCalibrated() ? "YES" : "NO");
  Serial.print("TCS1 calibrated: "); Serial.println(tcs1.isCalibrated() ? "YES" : "NO");

  Serial.println("\nTCS0 Calibration Details:");
  tcs0.printCalibration();

  Serial.println("\nTCS1 Calibration Details:");
  tcs1.printCalibration();
}

// Map ค่า knob → index
int readKnobSelection(int itemCount) {
  int raw = robot.knopRead();
  int mapped = map(raw, 0, 1023, -5, itemCount * 10 + 5);
  int index = mapped / 10;
  index = constrain(index, 0, itemCount - 1);
  return index;
}

void showMainMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  drawMenuHeader("Select Mode");

  int y = 8;           // เริ่มสูงขึ้นหน่อย (จาก 12 → 10)
  int lineHeight = 8;   // ลดเหลือ 8 พิกเซลต่อบรรทัด (text size 1 สูงจริง ๆ ~7–8 px)

  for (int i = 0; i < mainMenuCount; i++) {
    drawMenuItem(y, mainMenu[i], (i == selectedItem));
    y += lineHeight;
  }

  // แสดงแบตฯ (ปรับตำแหน่งให้ไม่ทับ ถ้าทับให้เลื่อนลง/ซ้าย)
  float voltage = robot.getVoltage();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(70, 0);   // อยู่บรรทัดหัวข้อ
  display.print(voltage, 2);
  display.print(F("V"));

  display.setCursor(104,0);   // เลื่อนลงมาหน่อย
  if (voltage < 2.0f)       
    {display.print(F("No Bat!"));}
  else if (voltage < 10.0f) 
    {display.print(F("Low!")); robot.playTone(2000, 200); delay(100);robot.playTone(3000, 100); delay(100);}    
      
  else                      
    {display.print(F("Full"));}

  display.display();
}

void showReadSensorMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  int y = 8;
  int lineHeight = 8;

  if (currentSubMode == 1) {
    drawMenuHeader("ADC");

    for (int i = 0; i < adcSubMenuCount; i++) {
      drawMenuItem(y, adcSubMenu[i], (i == selectedItem));
      y += lineHeight;
    }
  } else {
    drawMenuHeader("Read Sensor");

    for (int i = 0; i < readSensorMenuCount; i++) {
      drawMenuItem(y, readSensorMenu[i], (i == selectedItem));
      y += lineHeight;
    }
  }

  display.display();
}

void showCalibrateMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  drawMenuHeader("Calibrate Sensor");

  int y = 8;
  int lineHeight = 8;

  for (int i = 0; i < calibrateMenuCount; i++) {
    drawMenuItem(y, calibrateMenu[i], (i == selectedItem));
    y += lineHeight;
  }

  display.display();
}

void handleMenuCore() {
  int itemCnt = (currentSubMode == 1) ? adcSubMenuCount :
                (currentMenu == 0)     ? mainMenuCount :
                (currentMenu == 1)     ? readSensorMenuCount :
                                         calibrateMenuCount;

  selectedItem = readKnobSelection(itemCnt);

  if (currentSubMode == 1)      showReadSensorMenu();
  else if (currentMenu == 0)    showMainMenu();
  else if (currentMenu == 1)    showReadSensorMenu();
  else if (currentMenu == 2)    showCalibrateMenu();
}

// ฟังก์ชันแสดงค่า ADC (เหมือนเดิม แต่ปรับขนาดตัวอักษรให้พอดีจอ)
void showADCValues(uint16_t (*getValueFunc)(int)) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  int y = 0, count = 0;
  for (int ch = 0; ch < 10; ch++) {
    uint16_t val = getValueFunc(ch);

    if (count % 4 == 0 && count > 0) y += 10;

    display.setCursor((count % 4) * 32, y);
    display.printf("%4u", val);

    count++;
  }

  display.setCursor(60, 24);
  display.println("Hold 3s back");
  display.display();
}

// while loop หลักสำหรับเลือกโหมด (ส่วนนี้ logic เดิม แต่เรียกเมนูใหม่)
void selectAndRunMode() {
  currentMenu = 0;
  currentSubMode = 0;
  selectedItem = 0;
  buttonWasPressed = false;
  lastBeepTime = 0;

  while (true) {
    handleMenuCore();

    bool buttonNow = (digitalRead(2) == LOW);

    if (buttonNow && !buttonWasPressed) {
      buttonPressTime = millis();
      buttonWasPressed = true;
      lastBeepTime = 0;
    }

    if (!buttonNow && buttonWasPressed) {
      unsigned long duration = millis() - buttonPressTime;
      buttonWasPressed = false;

      if (duration < 3000) {
        if (currentMenu == 0 && selectedItem == 0) {
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(30, 8);
          display.println("RUN");
          display.display();          
          return;
        } else {
          enterSelectedMode();
        }
      }
    }

    // กดค้าง 3 วินาที → ย้อนกลับ
    if (buttonNow && buttonWasPressed) {
      unsigned long holdDuration = millis() - buttonPressTime;

      if (holdDuration >= 1500 && holdDuration < 3000) {
        if (millis() - lastBeepTime >= 500) {
          robot.playTone(1800, 60);
          lastBeepTime = millis();
        }

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(10, 10);
        display.print("Hold ");
        display.print((3000 - holdDuration)/1000);
        display.println("s more");
        display.setCursor(10, 20);
        display.println("to go back");
        display.display();
      }

      if (holdDuration >= 3000) {
        display.clearDisplay();
        display.display();

        for (int i = 0; i < 3; i++) {
          robot.playTone(800, 300);
          delay(150);
        }

        if (currentSubMode == 1) {
          currentSubMode = 0;
          selectedItem = 0;
        } else if (currentMenu == 1 || currentMenu == 2) {
          currentMenu = 0;
          currentSubMode = 0;
          selectedItem = 0;
        }

        buttonWasPressed = false;
        buttonPressTime = 0;
        lastBeepTime = 0;

        while (digitalRead(2) == LOW) delay(10);
        
      }
    }

    delay(20);
  }
}


// เข้าโหมดหรือแสดงค่าจริง
void enterSelectedMode() {
  robot.playTone(2500, 80);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  if (currentMenu == 0) {
    if (selectedItem == 0) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(30, 8);
      display.println("RUN");
      display.display();
      delay(600);
      return;
    } 
    else if (selectedItem == 1) {
      currentMenu = 1;
      selectedItem = 0;
      display.println("ReadSensor");
      display.setCursor(0, 12);
      display.println("Selected");
    } 
    else if (selectedItem == 2) {
      currentMenu = 2;
      selectedItem = 0;
      display.println("calibrateSensor");
      display.setCursor(0, 12);
      display.println("Selected");
    }
  } 
  else if (currentMenu == 1 && currentSubMode == 0) {
    if (selectedItem == 0) {
      currentSubMode = 1;
      selectedItem = 0;
      display.println("ADC");
      display.setCursor(0, 12);
      display.println("Selected");
    } 
    else if (selectedItem == 1) {   // color mode (ตรวจสอบสีปกติ)
      robot.playTone(1800, 100);

      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Color Mode");
      display.setCursor(0, 12);
      display.println("TCS0 & TCS1");
      display.display();
      delay(1000);

      unsigned long lastUpdate = 0;
      while (true) {
        if (millis() - lastUpdate >= 250) {
          display.clearDisplay();
          display.setTextSize(1);

          bool ok0 = tcs0.readFast();
          bool ok1 = tcs1.readFast();

          display.setCursor(0, 4);
          if (ok0) {
            display.print("T0: ");
            display.print(tcs0.getColor());
          } else {
            display.print("T0: Error");
          }

          display.setCursor(0, 16);
          if (ok1) {
            display.print("T1: ");
            display.print(tcs1.getColor());
          } else {
            display.print("T1: Error");
          }

          display.setCursor(60, 25);
          display.print("Hold 3s exit");

          display.display();
          lastUpdate = millis();
        }

        bool buttonNow = (digitalRead(2) == LOW);
        if (buttonNow) {
          if (!buttonWasPressed) {
            buttonPressTime = millis();
            buttonWasPressed = true;
          }

          if (millis() - buttonPressTime >= 3000) {
            robot.playTone(1200, 200);
            robot.playTone(800, 300);
            buttonWasPressed = false;
            return;
          }
        } else {
          buttonWasPressed = false;
        }

        delay(10);
      }
    } 
    else if (selectedItem == 2) {
      // gyro mode (คงเดิม)
      unsigned long lastUpdate = 0;
      while (true) {
        if (millis() - lastUpdate >= 20) {
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(10, 2);
          float z = gyro.gyro('z');
          display.print(z, 1);

          display.setTextSize(1);
          display.setCursor(10, 20);
          display.println("deg/s");

          display.setCursor(60, 25);
          display.println("Hold 3s back");
          display.display();

          lastUpdate = millis();
        }

        bool buttonNow = (digitalRead(2) == LOW);
        if (buttonNow) {
          if (!buttonWasPressed) {
            buttonPressTime = millis();
            buttonWasPressed = true;
          }
          if (millis() - buttonPressTime >= 3000) {
            buttonWasPressed = false;
            return;
          }
        } else {
          buttonWasPressed = false;
        }
      }
    }
  } 
  else if (currentSubMode == 1) {
    // ADC sub-menu (คงเดิม)
    uint16_t (*func)(int);
    switch (selectedItem) {
      case 0: func = [](int ch) -> uint16_t { return robot.adcRead(ch); }; break;
      case 1: func = [](int ch) -> uint16_t { return robot.adcMD(ch); }; break;
      case 2: func = [](int ch) -> uint16_t { return robot.adcMax(ch); }; break;
      case 3: func = [](int ch) -> uint16_t { return robot.adcMin(ch); }; break;
      default: func = [](int ch) -> uint16_t { return 0; }; break;
    }

    unsigned long lastUpdate = 0;
    while (true) {
      if (millis() - lastUpdate >= 20) {
        showADCValues(func);
        lastUpdate = millis();
      }

      bool buttonNow = (digitalRead(2) == LOW);
      if (buttonNow) {
        if (!buttonWasPressed) {
          buttonPressTime = millis();
          buttonWasPressed = true;
        }
        if (millis() - buttonPressTime >= 3000) {
          buttonWasPressed = false;
          return;
        }
      } else {
        buttonWasPressed = false;
      }
    }
  } 
  else if (currentMenu == 2) {
    // calibrateSensor
    if (selectedItem == 0) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 10);
      display.println("Calibrating ADC...");
      display.display();

      robot.calibrate();

      display.clearDisplay();
      display.setCursor(20, 8);
      display.println("Calibrated!");
      display.display();

      delay(2000);
    } 
    else if (selectedItem == 1) {   // colorCal - Calibrate แล้วตรวจสอบสีเลย
      robot.playTone(2000, 150);

      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Color Calibration");
      display.setCursor(0, 12);
      display.println("Press to start");
      display.display();
      delay(1500);

      const char* colors[] = {"RED", "GREEN", "BLUE", "YELLOW", "WHITE", "BLACK"};
      const int numColors = 6;

      // Calibrate TCS0
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Cal>> T0 Start...");
      display.display();
      delay(1000);

      for (int i = 0; i < numColors; i++) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("Cal>> -T0-");
        display.setCursor(0, 15);
        display.print("Cal>> ");
        display.println(colors[i]);
        display.setTextSize(1);
        display.setCursor(0, 25);
        display.println(">> Press start button");
        display.display();

        bool pressed = false;
        while (!pressed) {
          if (digitalRead(2) == LOW) {
            delay(50);
            if (digitalRead(2) == LOW) {
              robot.playTone(1500, 80);

              switch (i) {
                case 0: tcs0.calibrateRed();    break;
                case 1: tcs0.calibrateGreen();  break;
                case 2: tcs0.calibrateBlue();   break;
                case 3: tcs0.calibrateYellow(); break;
                case 4: tcs0.calibrateWhite();  break;
                case 5: tcs0.calibrateBlack();  break;
              }

              display.clearDisplay();
              display.setCursor(20, 10);
              robot.playTone(1000, 250);delay(100);
              robot.playTone(3000, 100);delay(100);
              display.println("Cal OK!");
              display.display();
              delay(800);
              pressed = true;
            }
          }
          delay(10);
        }
      }

      // Calibrate TCS1
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Cal>> T1 Start...");
      display.display();
      delay(1000);

      for (int i = 0; i < numColors; i++) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("Cal>> -T1-");
        display.setCursor(0, 15);
        display.print("Cal>> ");
        display.println(colors[i]);
        display.setTextSize(1);
        display.setCursor(0, 25);
        display.println(">> Press start button");
        display.display();

        bool pressed = false;
        while (!pressed) {
          if (digitalRead(2) == LOW) {
            delay(50);
            if (digitalRead(2) == LOW) {
              robot.playTone(1500, 80);

              switch (i) {
                case 0: tcs1.calibrateRed();    break;
                case 1: tcs1.calibrateGreen();  break;
                case 2: tcs1.calibrateBlue();   break;
                case 3: tcs1.calibrateYellow(); break;
                case 4: tcs1.calibrateWhite();  break;
                case 5: tcs1.calibrateBlack();  break;
              }

              display.clearDisplay();
              display.setCursor(20, 10);
              display.println("Cal OK!");
              robot.playTone(1000, 250);delay(100);
              robot.playTone(3000, 100);delay(100);
              display.display();
              delay(800);
              pressed = true;
            }
          }
          delay(10);
        }
      }

      // จบ calibration
      tcs0.calibrateDone();
      tcs1.calibrateDone();
      robot.playTone(3000, 100);delay(100);
      robot.playTone(3000, 100);delay(200);
      robot.playTone(3000, 100);delay(100);
      // แสดงสรุปแล้วเข้าสู่โหมดตรวจสอบสีทันที
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Cal Done!");
      display.setCursor(0, 12);
      display.println("Checking colors...");
      display.display();
      delay(1500);

      // โหมดตรวจสอบสีเรียลไทม์ (ชื่อสีเต็ม)
      unsigned long lastUpdate = 0;
      while (true) {
        if (millis() - lastUpdate >= 250) {
          display.clearDisplay();
          display.setTextSize(1);

          bool ok0 = tcs0.readFast();
          bool ok1 = tcs1.readFast();

          display.setCursor(0, 4);
          if (ok0) {
            display.print("T0: ");
            display.print(tcs0.getColor());
          } else {
            display.print("T0: Error");
          }

          display.setCursor(0, 16);
          if (ok1) {
            display.print("T1: ");
            display.print(tcs1.getColor());
          } else {
            display.print("T1: Error");
          }

          display.setCursor(60, 25);
          display.print("Hold 3s exit");

          display.display();
          lastUpdate = millis();
        }

        bool buttonNow = (digitalRead(2) == LOW);
        if (buttonNow) {
          if (!buttonWasPressed) {
            buttonPressTime = millis();
            buttonWasPressed = true;
          }

          if (millis() - buttonPressTime >= 3000) {
            robot.playTone(1200, 200);
            robot.playTone(800, 300);
            buttonWasPressed = false;
            return;
          }
        } else {
          buttonWasPressed = false;
        }

        delay(10);
      }
    }
  }

  display.display();
  delay(1200);
}