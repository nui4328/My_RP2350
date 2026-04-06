#ifndef _myrp_pico2_
#define _myrp_pico2_

#include <Wire.h>
#include <BatteryMonitor.h>
BatteryMonitor bat;
#include  <my_BMI160.h>

#include  <my_MCP3008s.h>
my_MCP3008s adc;
#include  <EncoderLibrarys.h>

//EncoderLibrarys encoder(22, 18, 11, 12);
EncoderLibrarys encoder(18, 22, 12, 11);
// กำหนดพินควบคุมมอเตอร์
my_BMI160 my; // สร้างอ็อบเจ็กต์ด้วยที่อยู่เริ่มต้น (0x69)


#define PWMA 19     // PWM ซ้าย
#define AIN1 20
#define AIN2 21

#define PWMB 6     // PWM ขวา
#define BIN1 8
#define BIN2 7

bool DC_Motors = true;
//___--------------------------------------------->>
#define EEPROM_ADDRESS 0x50
const int numSensors = 8;
const int numSamples = 1000;

int sensorValuesA[numSensors][numSamples];
int sensorMaxA[numSensors];
int sensorMinA[numSensors];
int sensorValuesB[numSensors][numSamples];
int sensorMaxB[numSensors];
int sensorMinB[numSensors];
int sensorValuesC[2][numSamples];
int sensorMaxC[2];
int sensorMinC[2];

float P, I, D, previous_I, previous_error,errors, PID_output, present_position, previous_integral; 
int numSensor = 6; 
int state_on_Line = 0;
int setpoint;
int _lastPosition;
int sensor_pin_A[] = {1,2,3,4,5,6}; 
int sensor_pin_B[] = {1,2,3,4,5,6}; 
const float I_MAX = 1000.0; // ขีดจำกัดบนของ integral
const float I_MIN = -1000.0; // ขีดจำกัดล่างของ integral

int numSensor4 = 4; 
int sensor_pin_4[] = {2,3,4,5}; 
//-------------------------------------------------------->>fline

bool pid_error = true;
const int ramp_delay = 6; // ms
int slmotor = 20, srmotor = 20; // PWM
int clml = -90, clmr = 90; // เลี้ยวซ้าย center
int crml = 90, crmr = -90; // เลี้ยวขวา center
int flml = -15, flmr = 100; // เลี้ยวซ้าย front
int frml = 100, frmr = -15; // เลี้ยวขวา front
int llmotor = 100, lrmotor = 50, ldelaymotor = 50; // เลี้ยวซ้าย speed
int rlmotor = 50, rrmotor = 100, rdelaymotor = 50; // เลี้ยวขวา speed
int break_ff = 5, break_fc = 30, break_bf = 10, break_bc = 20; // การหน่วง
int delay_f = 15; // การหน่วงก่อนเลี้ยว
float kd_f = 0.55, kd_b = 0.025; // Kd PID
float kp_slow = 0.1, ki_slow = 0.0001; // PID ช้า
float redius_wheel = 3.0; // รัศมีล้อ (cm)
int ch_p = 0;
bool _fw = true;
float new_encoder = 0;
float speed_scale_fw = 1.0; // สมมติ 1 PWM = 0.1 cm/s (ต้องปรับตามจริง)
float speed_scale_bw = 1.05; // สมมติ 1 PWM = 0.1 cm/s (ต้องปรับตามจริง)
String Freq_motor ;
// ค่าคงที่สำหรับ encoder (ใส่ส่วนบนของไฟล์ .ino หรือใน header file)
// ============================================================================
const float WHEEL_CIRCUMFERENCE_CM = 2.0 * 3.141592653589793 * 23.0;  // ≈ 263.8937829 cm
const int   TICKS_PER_REVOLUTION    = 25;
const float CM_PER_TICK             = (WHEEL_CIRCUMFERENCE_CM / TICKS_PER_REVOLUTION)/10.0;  // ≈ 2.000 cm/tick

// พารามิเตอร์ ramping & ชะลอ (หน่วย cm/s²)
float ACCELERATION_CM_S2   = 90.0;     // เร่งความเร็ว (ปรับให้ช้าลง = ค่าน้อยลง)
float DECELERATION_CM_S2   = 90.0;    // ชะลอ (ปรับให้ช้าลง = ค่าน้อยลง)
float SLOW_DOWN_START_CM   = 15.0;     // เริ่มชะลอเมื่อเหลือระยะนี้ (cm)
float MIN_SPEED            = slmotor;     // ความเร็วต่ำสุด (ป้องกันมอเตอร์หยุดนิ่ง)

// ≈ 0.19992 cm/tick   (ปรับจากผลทดสอบว่าต้องหาร 10 เพื่อให้ตรงกับความเป็นจริง)


bool setpoint_fw = false;
float _positions = 50;

//___--------------------------------------------->>
void get_EEP_Program(void);
void read_sensorA_program(void);
int md_sensorC(int sensor) ;

void myTone(int pin, int freq, int duration) {
     tone(pin, freq, duration);
    delay(duration + 50);  
}

void resetAngles()
  {
    my.resetAngles();
  }
void set_Freq(String fr_motor)
  {
    if(fr_motor == "DC_Motors" )
      {
        DC_Motors = true;
      }
    else  
      {
         DC_Motors = false;
      }
  }  
void distance_scale_fw(float scale)
  {
    speed_scale_fw = scale;
  }
void distance_scale_bw(float scale)
  {
    speed_scale_bw = scale;
  }
void set_slow_motor(int sl, int sr)
     {       
        slmotor = sl;  
        srmotor = sr;       
     }
void set_turn_center_l(int ml,int mr)
     {       
        clml = ml;
        clmr = mr;
     }
void set_turn_center_r(int ml,int mr)
     {       
        crml = ml;
        crmr = mr;
     }
void set_turn_front_l(int ml,int mr)
     {       
        flml = ml;
        flmr = mr;
     }
void set_turn_front_r(int ml,int mr)
     {       
        frml = ml;
        frmr = mr;
     }
void set_brake_fc(int ff, int fc)
     {       
        break_ff = ff;  
        break_fc = fc;       
     }
void set_brake_bc(int ff, int fc)
     {       
        break_bf = ff;  
        break_bc = fc;       
     }
void set_delay_f(int ff)
     {       
        delay_f = ff;       
     }
void set_speed_turn_fl(int inM, int outM, int delayM )
     {
        llmotor = inM;
        lrmotor = outM;
        ldelaymotor = delayM;
     }
void set_speed_turn_fr(int inM, int outM, int delayM )
     {
        rlmotor = inM;
        rrmotor = outM;
        rdelaymotor = delayM;
     }
#define I2C_SDA     4
#define I2C_SCL     5
void setup_robot() 
  {
    Serial.begin(115200); 
    pinMode(I2C_SDA, OUTPUT);
    pinMode(I2C_SCL, OUTPUT);
    digitalWrite(I2C_SDA, HIGH);
    digitalWrite(I2C_SCL, HIGH);
    delay(100);
    for (int i = 0; i < 9; i++) {
      digitalWrite(I2C_SCL, LOW); delayMicroseconds(5);
      digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
    }

  Wire.end();
  delay(800);
  Wire.begin();
    bat.begin();
    analogReadResolution(12);  
    analogWriteResolution(12);
       
   // resetAngles();
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(3, INPUT_PULLUP);
    pinMode(2, INPUT_PULLUP);
    pinMode(9, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    if(DC_Motors==true)
      {
        analogWriteFreq(1000);
      }    
    else
      {
        analogWriteFreq(20000);
      }
    
    //Serial.print( analogRead(26) );   Serial.print( "  " );   Serial.print( analogRead(27) ); Serial.print( "     " );
    //Serial.print(  md_sensorC(0));   Serial.print( "  " );   Serial.println(  md_sensorC(1) ); 
    for(int i = 0; i<2; i++)
      {
        for(int i = 0; i<3; i++)
          {
            digitalWrite(LED_BUILTIN,1);
            delay(50);
              digitalWrite(LED_BUILTIN,0);
              delay(50);
            }
        }   
      get_EEP_Program();
      read_sensorA_program();

      if (!my.begin()) {
    //Serial.println("Failed to initialize GYRO160!");
    //while (1);
   }
      resetAngles();
   
  }


/*
   get_maxMinA();
   get_maxMinB();
   get_maxMinC(); 
   read_eepA();
   read_sensorA_program();
   read_eepB();
   read_sensorB_program();
   read_eepC();
   read_sensorC_program();
*/


uint16_t read_sensorA(int sensor) 
  {       
     adc.begin(14, 15, 16, 13 );
     adc.begin(14, 15, 16, 17 );  //adc.begin(5, 4, 12, 20 );    // 5=clk, 4=IN, 12=out
     return adc.readADC(sensor);  
  }

uint16_t read_sensorB(int sensor) 
  {       
     adc.begin(14, 15, 16, 17 ); 
     adc.begin(14, 15, 16, 13 );  //adc.begin(5, 4, 12, 20 );    // 5=clk, 4=IN, 12=out
     return adc.readADC(sensor);  
  }
int md_sensorA(int sensor) 
  {      
     return (sensorMaxA[sensor]+sensorMinA[sensor])/2;
  }

int md_sensorB(int sensor) 
  {      
     return (sensorMaxB[sensor]+sensorMinB[sensor])/2;
  }
int md_sensorC(int sensor) 
  {      
     return (sensorMaxC[sensor]+sensorMinC[sensor])/2;
  }

// EEPROM
void writeEEPROM(int deviceAddress, unsigned int eeAddress, byte *data, int dataLength) {
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8));
  Wire.write((int)(eeAddress & 0xFF));
  for (int i = 0; i < dataLength; i++) Wire.write(data[i]);
  Wire.endTransmission();
  delay(5);
}

void readEEPROM(int deviceAddress, unsigned int eeAddress, byte *buffer, int dataLength) {
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8));
  Wire.write((int)(eeAddress & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, dataLength);
  for (int i = 0; i < dataLength; i++) {
    if (Wire.available()) buffer[i] = Wire.read();
  }
}

// ==================== get_maxMinA (เดิมของคุณ ใช้ได้อยู่แล้ว) ====================
void get_maxMinA() {
  for (int sample = 0; sample < numSamples; sample++) {
    for (int sensor = 0; sensor < numSensors; sensor++) {
      sensorValuesA[sensor][sample] = read_sensorA(sensor);
      delay(1);
    }
  }
  for (int sensor = 0; sensor < numSensors; sensor++) {
    sensorMaxA[sensor] = sensorValuesA[sensor][0];
    sensorMinA[sensor] = sensorValuesA[sensor][0];
    for (int sample = 1; sample < numSamples; sample++) {
      int value = sensorValuesA[sensor][sample];
      if (value > sensorMaxA[sensor]) sensorMaxA[sensor] = value;
      if (value < sensorMinA[sensor]) sensorMinA[sensor] = value;
    }
  }
  byte buffer[16];
  for (int i = 0; i < numSensors; i++) {
    buffer[i * 2] = highByte(sensorMaxA[i]);
    buffer[i * 2 + 1] = lowByte(sensorMaxA[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 0, buffer, 16);
  for (int i = 0; i < numSensors; i++) {
    buffer[i * 2] = highByte(sensorMinA[i]);
    buffer[i * 2 + 1] = lowByte(sensorMinA[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 16, buffer, 16);

  tone(9, 3000, 100); delay(200);
  tone(9, 3000, 200); delay(200);
}

// ==================== get_maxMinB (เหมือน A) ====================
void get_maxMinB() {
  // เหมือน get_maxMinA แต่ใช้ sensorMaxB/sensorMinB และ address 32, 48
  for (int sample = 0; sample < numSamples; sample++) {
    for (int sensor = 0; sensor < numSensors; sensor++) {
      sensorValuesB[sensor][sample] = read_sensorB(sensor);
      delay(1);
    }
  }
  for (int sensor = 0; sensor < numSensors; sensor++) {
    sensorMaxB[sensor] = sensorValuesB[sensor][0];
    sensorMinB[sensor] = sensorValuesB[sensor][0];
    for (int sample = 1; sample < numSamples; sample++) {
      int value = sensorValuesB[sensor][sample];
      if (value > sensorMaxB[sensor]) sensorMaxB[sensor] = value;
      if (value < sensorMinB[sensor]) sensorMinB[sensor] = value;
    }
  }
  byte buffer[16];
  for (int i = 0; i < numSensors; i++) {
    buffer[i * 2] = highByte(sensorMaxB[i]);
    buffer[i * 2 + 1] = lowByte(sensorMaxB[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 32, buffer, 16);
  for (int i = 0; i < numSensors; i++) {
    buffer[i * 2] = highByte(sensorMinB[i]);
    buffer[i * 2 + 1] = lowByte(sensorMinB[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 48, buffer, 16);

  tone(9, 3000, 100); delay(200);
  tone(9, 3000, 200); delay(200);
}

// ==================== get_maxMinC (ของคุณเดิม + constrain) ====================
void get_maxMinC() {
  const int PIN_C0 = 26;
  const int PIN_C1 = 27;

  for (int sample = 0; sample < numSamples; sample++) {
    sensorValuesC[0][sample] = analogRead(PIN_C0);
    sensorValuesC[1][sample] = analogRead(PIN_C1);
    delay(5);
  }

  for (int sensor = 0; sensor < 2; sensor++) {
    sensorMaxC[sensor] = 0;
    sensorMinC[sensor] = 4095;
    for (int sample = 0; sample < numSamples; sample++) {
      uint16_t value = sensorValuesC[sensor][sample];
      if (value > sensorMaxC[sensor]) sensorMaxC[sensor] = value;
      if (value < sensorMinC[sensor]) sensorMinC[sensor] = value;
    }
    sensorMaxC[sensor] = constrain(sensorMaxC[sensor], 0, 4000);
    sensorMinC[sensor] = constrain(sensorMinC[sensor], 0, 4000);
  }

  uint8_t buffer[4];
  for (int i = 0; i < 2; i++) {
    buffer[i*2]     = highByte(sensorMaxC[i]);
    buffer[i*2 + 1] = lowByte(sensorMaxC[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 64, buffer, 4);

  for (int i = 0; i < 2; i++) {
    buffer[i*2]     = highByte(sensorMinC[i]);
    buffer[i*2 + 1] = lowByte(sensorMinC[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 68, buffer, 4);

  tone(9, 3000, 150); delay(200);
  tone(9, 3000, 200); delay(250);
}

// ==================== read_eepA (แก้ให้โหลดเข้า sensorMaxA/sensorMinA จริง) ====================
void read_eepA() {
  byte readBuffer[16];
  int readMaxA[numSensors], readMinA[numSensors];
  readEEPROM(EEPROM_ADDRESS, 0, readBuffer, 16);
  for (int i = 0; i < numSensors; i++) {
    readMaxA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 16, readBuffer, 16);
  for (int i = 0; i < numSensors; i++) {
    readMinA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }

  Serial.println("Sensor A Values read from EEPROM:");
  for (int sensor = 0; sensor < numSensors; sensor++) {
    Serial.print("Sensor "); Serial.print(sensor); Serial.print(" => Max: "); Serial.print(readMaxA[sensor]); Serial.print(", Min: "); Serial.println(readMinA[sensor]);
    sensorMaxA[sensor] = readMaxA[sensor];   // โหลดเข้า sensorMaxA จริง
    sensorMinA[sensor] = readMinA[sensor];   // โหลดเข้า sensorMinA จริง
  }   
}

// ==================== read_eepB (เหมือนกัน) ====================
void read_eepB() {
  byte readBuffer[16];
  int readMaxB[numSensors], readMinB[numSensors];
  readEEPROM(EEPROM_ADDRESS, 32, readBuffer, 16);
  for (int i = 0; i < numSensors; i++) {
    readMaxB[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 48, readBuffer, 16);
  for (int i = 0; i < numSensors; i++) {
    readMinB[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }

  Serial.println("Sensor B Values read from EEPROM:");
  for (int sensor = 0; sensor < numSensors; sensor++) {
    Serial.print("Sensor "); Serial.print(sensor); Serial.print(" => Max: "); Serial.print(readMaxB[sensor]); Serial.print(", Min: "); Serial.println(readMinB[sensor]);
    sensorMaxB[sensor] = readMaxB[sensor];   // โหลดเข้า sensorMaxB จริง
    sensorMinB[sensor] = readMinB[sensor];   // โหลดเข้า sensorMinB จริง
  }
}

// ==================== read_eepC (แก้แล้ว) ====================
void read_eepC() {
  byte readBuffer[4];
  int readMaxC[2], readMinC[2];
  readEEPROM(EEPROM_ADDRESS, 64, readBuffer, 4);
  for (int i = 0; i < 2; i++) {
    readMaxC[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 68, readBuffer, 4);
  for (int i = 0; i < 2; i++) {
    readMinC[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }

  Serial.println("Sensor C Values read from EEPROM:");
  Serial.print("Sensor C0 (Pin 26) => Max: "); Serial.print(readMaxC[0]); Serial.print(", Min: "); Serial.println(readMinC[0]);
  Serial.print("Sensor C1 (Pin 27) => Max: "); Serial.print(readMaxC[1]); Serial.print(", Min: "); Serial.println(readMinC[1]);

  sensorMaxC[0] = readMaxC[0]; sensorMinC[0] = readMinC[0];   // โหลดเข้า sensorMaxC/sensorMinC จริง
  sensorMaxC[1] = readMaxC[1]; sensorMinC[1] = readMinC[1];
}

// ==================== แสดงผล (ใช้ตัวแปรเดียวกัน) ====================
void read_sensorA_program() {
  Serial.println("Sensor MAX A Values read from program:");
  for (int sensor = 0; sensor < numSensors; sensor++) {
    Serial.print("Sensor "); Serial.print(sensor); Serial.print(" => Max: "); Serial.print(sensorMaxA[sensor]); Serial.print(", Min: "); Serial.println(sensorMinA[sensor]);
  }   
}

void read_sensorB_program() {
  Serial.println("Sensor MAX B Values read from program:");
  for (int sensor = 0; sensor < numSensors; sensor++) {
    Serial.print("Sensor "); Serial.print(sensor); Serial.print(" => Max: "); Serial.print(sensorMaxB[sensor]); Serial.print(", Min: "); Serial.println(sensorMinB[sensor]);
  }   
}

void read_sensorC_program() {
  Serial.println("Sensor C Values read from program:");
  Serial.print("Sensor C0 (Pin 26) => Max: "); Serial.print(sensorMaxC[0]); Serial.print(", Min: "); Serial.println(sensorMinC[0]);
  Serial.print("Sensor C1 (Pin 27) => Max: "); Serial.print(sensorMaxC[1]); Serial.print(", Min: "); Serial.println(sensorMinC[1]);
}

void get_EEP_Program() {
  read_eepA();
  read_eepB();
  read_eepC();
  read_sensorA_program();
  read_sensorB_program();
  read_sensorC_program();
}
//-------------------------------------------------------------------------------------->>ควบคุมมอเตอร์


//-------------------------------------------------------------------------------------->>ควบคุมมอเตอร์
void tone(int t, int d) {
  tone(9, t, d);
  delay(d + 50);
}

void blinkLED(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_BUILTIN, 1);
    delay(100);
    digitalWrite(LED_BUILTIN, 0);
    delay(100);
  }
}

void waitButton() {
  while (digitalRead(3) == 1);  // รอจนกดปุ่ม
  delay(200); // debounce
  while (digitalRead(3) == 0);  // รอจนปล่อยปุ่ม
  delay(200);
}
#define MCP3421_ADDR 0x68  // I2C Address when A0 = GND  
int ADC_i2c()
  {
    long ADC01 = 0;
    int adc_01;

    // อ่านจาก Wire (I2C0)
    Wire.requestFrom(MCP3421_ADDR, 4);
    if (Wire.available() == 4) 
      {
        byte b1 = Wire.read();
        byte b2 = Wire.read();
        byte b3 = Wire.read();
        byte cfg = Wire.read();

        ADC01 = ((long)b1 << 16) | ((long)b2 << 8) | b3;
        if (b1 & 0x80) ADC01 |= 0xFF000000; // sign-extend
      }
    adc_01 = map(ADC01, 524048, 282, 4000, 0);
    //Serial.print("ADC01: "); Serial.print(adc_01);
    //delay(10);
    return adc_01;
  }

void sw()
  {
   
    tone(9, 2000, 100);   // โด
    delay(100);
    tone(9, 2400, 100);   // เร
    delay(100);
    tone(9, 3000, 160);  // มี (ยาวนิดนึง)
    delay(500);
    tone(9, 3000, 60);  // มี (ยาวนิดนึง)
    delay(80);
    tone(9, 3000, 60);  // มี (ยาวนิดนึง)
    delay(80);
    int buttonState;
    unsigned long pressStartTime = 0;
    bool isPressed = false;   

    if(ADC_i2c() < 4500 && ADC_i2c() > 3500)
      {
        while(1)
          { 
            bat.update_led();      
            if(digitalRead(3) == 0)
                {
                  digitalWrite(LED_BUILTIN, 1);

                  tone(3000, 200); 
                  get_maxMinA();
                  blinkLED(5);

                }
            if(ADC_i2c() < 2500)
                {
                  tone(3000, 100);
                  tone(3000, 200);
                  tone(3000, 200);

                  get_maxMinB();
                  blinkLED(5);
                  tone(9, 3000, 100);
                  delay(200); // รอ 1 วินาที
                  tone(9, 3000, 200);
                  delay(200); // รอ 1 วินาที
                }
              Serial.print("From A ");
              for (int i = 0; i < 8; i++) {
                Serial.print(read_sensorA(i));  // ใช้ read_sensorA
                Serial.print(" ");
              }
              Serial.print("   ");
              
              // แสดงค่าจาก Nano 0x09
              Serial.print("From B ");
              for (int i = 0; i < 8; i++) {
                Serial.print(read_sensorB(i));  // ใช้ read_sensorB
                Serial.print(" ");
              }
              Serial.println("  ");
            // Serial.println(my_tcs('r'));
              
              delay(10);  // อัปเดตทุก 100ms
            Serial.println(" "); 
            
            if (digitalRead(2) == LOW) 
              {  // ปุ่มถูกกด (LOW เพราะใช้ PULLUP)
                
                if (!isPressed) \
                  {
                    pressStartTime = millis();  // บันทึกเวลาที่กดปุ่มครั้งแรก
                    isPressed = true;
                  } 
                else 
                  {
                    unsigned long pressDuration = millis() - pressStartTime;   
                     
                    if (pressDuration >= 3000) 
                      {  // กดค้าง 3 วินาที
                        blinkLED(5);
                        tone(3000, 100);
                        tone(3000, 200);

                        get_maxMinC();
                        blinkLED(5);
                        tone(9, 3000, 100);
                        delay(200); // รอ 1 วินาที
                        tone(9, 3000, 200);
                        delay(200); // รอ 1 วินาที
                          delay(50);
                                while (digitalRead(2) == LOW);  // รอให้ปล่อยปุ่ม
                                delay(200);  // ป้องกันการเด้งของปุ่ม
                              }
                  }
              } 
            else 
              {
                if (isPressed) 
                  {
                    unsigned long pressDuration = millis() - pressStartTime;
                    
                    if (pressDuration >= 50 && pressDuration < 3000) 
                      {  
                        Serial.println("Entering Mode B");
                        break;
                      }
                    isPressed = false;
                  }
              }
          }
      }
    else  
      {
        while(1)
          {
            bat.update();
            if(digitalRead(3) == 0)
              {
                digitalWrite(LED_BUILTIN, 1);

                tone(3000, 200); 
                get_maxMinA();
                blinkLED(5);
                while(1)
                  {
                    waitButton();
                    tone(3000, 100);
                    tone(3000, 200);

                    get_maxMinC();
                    blinkLED(5);
                    tone(9, 3000, 100);
                    delay(200); // รอ 1 วินาที
                    tone(9, 3000, 200);
                    delay(200); // รอ 1 วินาที

                    waitButton();
                    tone(3000, 100);
                    tone(3000, 200);
                    tone(3000, 200);

                    get_maxMinB();
                    blinkLED(5);
                    tone(9, 3000, 100);
                    delay(200); // รอ 1 วินาที
                    tone(9, 3000, 200);
                    delay(200); // รอ 1 วินาที

                    break;
                  }
              }

      
            if(digitalRead(2) == 0)
              {
                break;
              }
            Serial.print("From A ");
              for (int i = 0; i < 8; i++) {
                  Serial.print(read_sensorA(i));  // ใช้ read_sensorA
                  Serial.print(" ");
                }
                Serial.print("   ");
                
                // แสดงค่าจาก Nano 0x09
                Serial.print("From B ");
                for (int i = 0; i < 8; i++) {
                  Serial.print(read_sensorB(i));  // ใช้ read_sensorB
                  Serial.print(" ");
                }
                Serial.println("  ");
            }
          }
    
    get_EEP_Program();
    tone(9, 3000, 400);
    delay(500);
  }

///----------------------------------------------------------------------------------->>>>
#define VMAX 12.6
#define VMIN 7.4
#define VNOM 11.55

#define BATTERY_INTERVAL 60

float Vbat = -1.0;
unsigned long lastBatteryRead = 0;

//--------------------------------
// อ่านค่าแบต (optional)
//--------------------------------
void updateBattery()
{
    bat.update();
}

float getBatteryVoltage()
{
    return bat.getVoltage();
}
//--------------------------------
// Motor Control
//--------------------------------
float scale = 1.0;
bool batteryUsed = false;
void bat_control()
  {     
    if(getBatteryVoltage() >= VMIN && getBatteryVoltage() <= VMAX)
    {
      scale = pow(VNOM / getBatteryVoltage(), 0.95);
      batteryUsed = true;
    }
  }

void Motor(int pwmL, int pwmR)
{
  
  // ตรวจสอบว่ามีค่าแบตหรือไม่
   bat_control();
  
  int pwmValueL = map(abs(pwmL),0,100,0,4095);
  int pwmValueR = map(abs(pwmR),0,100,0,4095);

  pwmValueL = constrain((int)(pwmValueL * scale),0,4095);
  pwmValueR = constrain((int)(pwmValueR * scale),0,4095);

  // LEFT MOTOR
  if (pwmL > 0) {
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
  }
  else if (pwmL < 0) {
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,HIGH);
  }
  else {
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,LOW);
    pwmValueL = 0;
  }

  // RIGHT MOTOR
  if (pwmR > 0) {
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,LOW);
  }
  else if (pwmR < 0) {
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,HIGH);
  }
  else {
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,LOW);
    pwmValueR = 0;
  }

  analogWrite(PWMA,pwmValueL);
  analogWrite(PWMB,pwmValueR);

  //--------------------------------
  // แสดงสถานะแบต
  //--------------------------------
  /*
  if(batteryUsed)
  {
    Serial.print("BATTERY OK  V=");
    Serial.print(Vbat,2);
  }
  else
  {
    Serial.print("NO BATTERY");
  }
 */
  Serial.print("  PWM=");
  Serial.print(pwmValueL);
  Serial.print(",");
  Serial.println(pwmValueR);
 
}


//--------------------------------

//-------------------------------------------------------------------------------------->>ควบคุมserv


#include <Servo.h>

// กำหนดขาเซอร์โว
#define servo0 0
#define servo1 1
#define servo10 10
#define servo28 28

// สร้างออบเจ็กต์เซอร์โว
Servo servo_0;
Servo servo_1;
Servo servo_10;
Servo servo_28;

// ตัวแปรสำหรับเก็บค่า trim และมุมก่อนหน้า
int servo_tim10 = 0;
int servo_tim0 = 0;
int servo_tim1 = 0;
int servo_tim28 = 0;
int num_steps = 20;
float s10_before_deg = 120;
float s0_before_deg = 120;
float s1_before_deg = 50;
float s28_before_deg = 0;

// ฟังก์ชันตั้งค่า trim
void s0_trim(int _s0) {
    servo_tim0 = _s0;
}

void s1_trim(int _s1) {
    servo_tim1 = _s1; // แก้ไขจาก servo_tim10 เป็น servo_tim1
}

void s10_trim(int _s10) {
    servo_tim10 = _s10; // แก้ไขจาก servo_tim10 เป็น servo_tim28
}

void s28_trim(int _s28) {
    servo_tim28 = _s28; // แก้ไขจาก servo_tim10 เป็น servo_tim28
}

// ฟังก์ชันควบคุมเซอร์โว
void servo(int servo, int angle) {      
    if (servo == 10) { 
        servo_10.attach(servo10, 500, 2500);
        servo_10.write(angle);        
    }else if(servo == 0) {
        servo_0.attach(servo0, 500, 2500);
        servo_0.write(180 - angle);        
    } else if (servo == 1) {
        servo_1.attach(servo1, 500, 2500);
        servo_1.write(angle);        
    }  else if (servo == 28) {
        servo_28.attach(servo28, 500, 2500);
        servo_28.write(angle+servo_tim28);      
    }
}

// ฟังก์ชันควบคุมเซอร์โว 10 และ 1 พร้อมกัน
void arm_left_right(float sl, float sr, int sp) {
    float servo10_step = (sl - s10_before_deg) / num_steps;
    float servo1_step = (sr - s1_before_deg) / num_steps;
    
    for (int i = 0; i < num_steps; i++) {
        float servo10_pos = s10_before_deg + (i * servo10_step); // แก้ไขจาก servo1_step เป็น servo10_step
        float servo1_pos = s1_before_deg + (i * servo1_step);
        servo(10, servo10_pos);
        servo(1, servo1_pos);
        delay(sp);
        Serial.print("S10: "); Serial.println(servo10_pos); 
        Serial.print("S1: "); Serial.println(servo1_pos);  
    }
    s10_before_deg = sl;
    s1_before_deg = sr;
}

void arm_up_down(float sl, int sp) {
    float servo0_step = (sl - s0_before_deg) / num_steps;
    
    for (int i = 0; i < num_steps; i++) {
        float servo0_pos = s0_before_deg + (i * servo0_step); // แก้ไขจาก servo1_step เป็น servo10_step6
        servo(0, servo0_pos);
        delay(sp);
        Serial.print("S0: "); Serial.println(servo0_pos); 
    }
    s0_before_deg = sl;
}
//-------------------------------------------------------------------------------------->>ควบคุมservo


//-------------------------------------------------------------------------------------->>ควบคุม PID

/*
int position_A(int *sensor_pins, int *sensor_Minvalues, int *sensor_Maxvalues, int num_sensors, int *last_position) {
    bool onLine = false;
    long avg = 0;
    long sum = 0;

    for (uint8_t i = 0; i < num_sensors; i++) {
        // อ่านค่าเซนเซอร์และแปลงด้วย map
        long value = map(read_sensorA(sensor_pins[i]), sensor_Minvalues[i], sensor_Maxvalues[i], 1000, 0);

        if (value > 200) {
            onLine = true;
        }
        if (value > 50) {
            avg += (long)value * (i * 1000);
            sum += value;
        }
        delayMicroseconds(50);
    }

    if (!onLine) {
        // ถ้าไม่เจอเส้น ใช้ last_position เพื่อตัดสินใจ
        if (*last_position < (num_sensors - 1) * 1000 / 2) {
            return 0;
        } else {
            return (num_sensors - 1) * 1000; // คืนค่าสูงสุดตามจำนวนเซนเซอร์
        }
    }

    // อัปเดต last_position ผ่านพอยน์เตอร์
    *last_position = (sum == 0) ? *last_position : avg / sum;

    return *last_position;
}
*/
int position_A()  
   {        
      int Minsensor_values_A[] = { sensorMinA[1], sensorMinA[2], sensorMinA[3], sensorMinA[4], sensorMinA[5], sensorMinA[6] }; // ค่าที่อ่านได้น้อยสุดหรือ สีดำ
      int Maxsensor_values_A[] = { sensorMaxA[1], sensorMaxA[2], sensorMaxA[3], sensorMaxA[4], sensorMaxA[5], sensorMaxA[6] }; // ค่าที่อ่านได้มากสุด สีขาว              
      bool onLine = false;
      long avg = 0;
      long sum = 0;
      for (uint8_t i = 0; i < 6 ; i++) 
          {              
              long value = map( read_sensorA(sensor_pin_A[i]), Minsensor_values_A[i], Maxsensor_values_A[i], 1000, 0);                                                                         // จากนั้นก็เก็บเข้าไปยังตัวแป value

              if (value > 200) 
                 { 
                    onLine = true;
                 }
              if (value > 50)   
                 {
                    avg += (long)value * (i * 1000);  
                    sum += value;                 
                 }
               //delayMicroseconds(50); 
         }
      if (!onLine)        //เมื่อหุ่นยนต์ไม่อยู่หรือไม่เจอเส้นดำ
         {
            if (_lastPosition < (numSensor - 1) * 1000 / 2)  // ถ้าค่าก่อนหน้าที่จะไม่เจอเส้นดำหรือหลุดจะให้ค่านั้นเป็น 0
               {
                  return 0;
               }
            else                                          //แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
               {
                 return 5000;                  

               }

          }
        _lastPosition = avg / sum;        //นำมาหาค่าเฉลี่ย

        return _lastPosition;            //ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
    }
int position_A_none()  
   {        
      int Minsensor_values_A[] = { sensorMinA[1], sensorMinA[2], sensorMinA[3], sensorMinA[4], sensorMinA[5], sensorMinA[6] }; // ค่าที่อ่านได้น้อยสุดหรือ สีดำ
      int Maxsensor_values_A[] = { sensorMaxA[1], sensorMaxA[2], sensorMaxA[3], sensorMaxA[4], sensorMaxA[5], sensorMaxA[6] }; // ค่าที่อ่านได้มากสุด สีขาว              
      bool onLine = false;
      long avg = 0;
      long sum = 0;
      for (uint8_t i = 0; i < 6 ; i++) 
          {              
              long value = map( read_sensorA(sensor_pin_A[i]), Minsensor_values_A[i], Maxsensor_values_A[i], 1000, 0);                                                                         // จากนั้นก็เก็บเข้าไปยังตัวแป value

              if (value > 200) 
                 { 
                    onLine = true;
                 }
              if (value > 50)   
                 {
                    avg += (long)value * (i * 1000);  
                    sum += value;                 
                 }
         }
      if (!onLine)        //เมื่อหุ่นยนต์ไม่อยู่หรือไม่เจอเส้นดำ
         {
            if (_lastPosition < (numSensor - 1) * 1000 / 2)  // ถ้าค่าก่อนหน้าที่จะไม่เจอเส้นดำหรือหลุดจะให้ค่านั้นเป็น 0
               {
                  return 2500;
               }
            else                                          //แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
               {
                 return 2500;                  

               }

          }
        _lastPosition = avg / sum;        //นำมาหาค่าเฉลี่ย

        return _lastPosition;            //ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
    }

float error_A()
    {
      if(pid_error == true)
        {
          present_position = position_A_none()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;
        }
      else
        {
          present_position = position_A()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;
        }                            
    }
float error_AA()
    {
          present_position = position_A()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;                          
    }
float error_AN()
    {
          present_position = position_A_none() / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;                          
    }
int position_B()  
   {        
      int Minsensor_values_A[] = { sensorMinB[1], sensorMinB[2], sensorMinB[3], sensorMinB[4], sensorMinB[5], sensorMinB[6] }; // ค่าที่อ่านได้น้อยสุดหรือ สีดำ
      int Maxsensor_values_A[] = { sensorMaxB[1], sensorMaxB[2], sensorMaxB[3], sensorMaxB[4], sensorMaxB[5], sensorMaxB[6] }; // ค่าที่อ่านได้มากสุด สีขาว              
      bool onLine = false;
      long avg = 0;
      long sum = 0;
      for (uint8_t i = 0; i < 6 ; i++) 
          {              
              long value = map( read_sensorB(sensor_pin_A[i]), Minsensor_values_A[i], Maxsensor_values_A[i], 1000, 0);                                                                         // จากนั้นก็เก็บเข้าไปยังตัวแป value

              if (value > 200) 
                 { 
                    onLine = true;
                 }
              if (value > 50)   
                 {
                    avg += (long)value * (i * 1000);  
                    sum += value;                 
                 }
         }
      if (!onLine)        //เมื่อหุ่นยนต์ไม่อยู่หรือไม่เจอเส้นดำ
         {
            if (_lastPosition < (numSensor - 1) * 1000 / 2)  // ถ้าค่าก่อนหน้าที่จะไม่เจอเส้นดำหรือหลุดจะให้ค่านั้นเป็น 0
               {
                  return 0;
               }
            else                                          //แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
               {
                 return 5000;                  

               }

          }
        _lastPosition = avg / sum;        //นำมาหาค่าเฉลี่ย

        return _lastPosition;            //ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
    }
int position_B_none()  
   {        
      int Minsensor_values_A[] = { sensorMinB[1], sensorMinB[2], sensorMinB[3], sensorMinB[4], sensorMinB[5], sensorMinB[6] }; // ค่าที่อ่านได้น้อยสุดหรือ สีดำ
      int Maxsensor_values_A[] = { sensorMaxB[1], sensorMaxB[2], sensorMaxB[3], sensorMaxB[4], sensorMaxB[5], sensorMaxB[6] }; // ค่าที่อ่านได้มากสุด สีขาว              
      bool onLine = false;
      long avg = 0;
      long sum = 0;
      for (uint8_t i = 0; i < 6 ; i++) 
          {              
              long value = map( read_sensorB(sensor_pin_A[i]), Minsensor_values_A[i], Maxsensor_values_A[i], 1000, 0);                                                                         // จากนั้นก็เก็บเข้าไปยังตัวแป value

              if (value > 200) 
                 { 
                    onLine = true;
                 }
              if (value > 50)   
                 {
                    avg += (long)value * (i * 1000);  
                    sum += value;                 
                 }
         }
      if (!onLine)        //เมื่อหุ่นยนต์ไม่อยู่หรือไม่เจอเส้นดำ
         {
            if (_lastPosition < (numSensor - 1) * 1000 / 2)  // ถ้าค่าก่อนหน้าที่จะไม่เจอเส้นดำหรือหลุดจะให้ค่านั้นเป็น 0
               {
                  return 2500;
               }
            else                                          //แต่ถ้ามากกว่าแสดงว่าหลุดออกอีกฝั่ง ค่าก็จะเป็น 1000 คุณด้วยจำนวนเซ็นเซอร์
               {
                 return 2500;                  

               }

          }
        _lastPosition = avg / sum;        //นำมาหาค่าเฉลี่ย

        return _lastPosition;            //ส่งค่าที่อ่านได้จากการเฉลี่ยแล้วกลับไปยังฟังก์ชั้น readline
    }

float error_B()
    {
      if(pid_error == true)
        {
          present_position = position_B_none()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;
        }
      else
        {
          present_position = position_B()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;
        }                            
    }

float error_BB()
    {
      
          present_position = position_B()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;
                                
    }

float error_BN()
    {
      
          present_position = position_B_none()  / ((numSensor - 1) * 10) ;
          setpoint = 50.0;
          errors = setpoint - present_position;
          return errors;
                                 
    }



//------------------------------------------------------------------------------------------------------------------------>>>คำสั่งเดินตามเส้น
void fw(int sl, int sr, float kp)
  {
    while(1) 
    {
      updateBattery();
      delayMicroseconds(50);
      errors = error_A();
      updateBattery();
P = errors;
      D = errors - previous_error;                  
      previous_error = errors;
      PID_output = (kp * P) + (0.0001 * I) + (0.025 * D); 
      Motor(sl - PID_output, sr + PID_output);
      
    }
      
  }
void fws(int sl, int sr, float kp) 
{
    int current_speed = 0;    // เริ่มจากความเร็ว 0
    int target_speed = (sl < sr) ? sl : sr;  // เลือกความเร็วต่ำสุดเป็นเป้าหมายเพื่อสมดุล
    const int ramp_step = 2;  // ความเร็วเพิ่มครั้งละ 2 (ปรับได้ตามความนุ่มนวลที่ต้องการ)
    const int ramp_delay = 10; // หน่วงระหว่างเพิ่มความเร็ว (ms)

    // Soft start เพิ่มความเร็วจนถึงเป้าหมาย
    while (current_speed < target_speed) 
    {
       updateBattery();
        errors = error_A();
        updateBattery();
P = errors;
        D = errors - previous_error;
        previous_error = errors;

        PID_output = (kp * P) + (0.0001 * I) + (0.025 * D);
        Motor(current_speed - PID_output, current_speed + PID_output);

        current_speed += ramp_step;
        if (current_speed > target_speed) current_speed = target_speed;

        delay(ramp_delay);
    }

    // เดินหน้าปกติเมื่อถึงความเร็วที่ต้องการ
    while (1) 
    {
        delayMicroseconds(50);
        errors = error_A();
        updateBattery();
P = errors;
        D = errors - previous_error;
        previous_error = errors;

        PID_output = (kp * P) + (0.0001 * I) + (0.025 * D);
        Motor(sl - PID_output, sr + PID_output);
    }
}



void fws(int sl, int sr, float kp, float distance) 
{
    int current_speed = 0;    // เริ่มจากความเร็ว 0 (หน่วย: cm/s)
    int target_speed = (sl < sr) ? sl : sr;  // เลือกความต่ำสุดเพื่อสมดุล
    const int ramp_step = 2;  // ความเร็วเพิ่มครั้งละ 2
    const int ramp_delay = 10; // หน่วงระหว่างเพิ่มความเร็ว (ms)
    float errors, P, D, previous_error = 0, PID_output;
    float I = 0;  // ตัวแปร Integral
    float traveled_distance = 0;  // ระยะทาง (หน่วย: เซนติเมตร)
    unsigned long last_time = millis();  // เก็บเวลาเริ่มต้น

    // Soft start เพิ่มความเร็วจนถึงเป้าหมาย
    while (current_speed < target_speed) 
    {
        
        errors = error_A();
        updateBattery();
P = errors;
        I += errors * (ramp_delay / 1000.0);  // อัพเดท Integral
        D = errors - previous_error;
        previous_error = errors;

        PID_output = (kp * P) + (0.0001 * I) + (0.025 * D);
        Motor(current_speed - PID_output, current_speed + PID_output);

        // คำนวณระยะทางถ้ากำหนด distance > 0
        if (distance > 0) {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;  // หน่วยวินาที
            traveled_distance += current_speed * delta_time;  // ความเร็ว cm/s * เวลา วินาที = ระยะทาง cm
            last_time = current_time;

            // ตรวจสอบระยะทาง
            if (traveled_distance >= distance) 
            {
                Motor(0, 0);  // หยุดมอเตอร์
                return;  // ออกจากฟังก์ชัน
            }
        }

        current_speed += ramp_step;
        if (current_speed > target_speed) current_speed = target_speed;

        delay(ramp_delay);
    }

    // เดินหน้าปกติ
    while (1) 
    {
        updateBattery();
        errors = error_A();
        updateBattery();
P = errors;
        I += errors * 0.00005;  // อัพเดท Integral (สำหรับ 50us)
        D = errors - previous_error;
        previous_error = errors;

        PID_output = (kp * P) + (0.0001 * I) + (0.025 * D);
        Motor(sl - PID_output, sr + PID_output);

        // คำนวณระยะทางถ้ากำหนด distance > 0
        if (distance > 0) {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;  // หน่วยวินาที
            traveled_distance += target_speed * delta_time;  // ความเร็ว cm/s * เวลา วินาที = ระยะทาง cm
            last_time = current_time;

            // ตรวจสอบระยะทาง
            if (traveled_distance >= distance) 
            {
                Motor(0, 0);  // หยุดมอเตอร์
                return;  // ออกจากฟังก์ชัน
            }
        }

        delayMicroseconds(50);
    }

    ///////////////////////////////-------------------------------------------------------->>>>>>>>  PID

}



void wheel_redius(float rediuss) {
    redius_wheel = rediuss;
}

float wheel_distance() {
    return 2 * 3.14159 * redius_wheel / 10; // เซนติเมตร
}

void test_distance(int distance1) {
    new_encoder = 440 * distance1 / wheel_distance();
    Serial.println(new_encoder);
    encoder.resetEncoders();
    do {
        Motor(30, 30);
    } while (encoder.Poss_R() < new_encoder);
    Motor(-30, -30);
    delay(10);
    Motor(1, 1);
}

void to_slow_motor(int sl, int sr) {
    slmotor = sl;
    srmotor = sr;
}

void to_turn_center_l(int ml, int mr) {
    clml = ml;
    clmr = mr;
}

void to_turn_center_r(int ml, int mr) {
    crml = ml;
    crmr = mr;
}

void to_turn_front_l(int ml, int mr) {
    flml = ml;
    flmr = mr;
}

void to_turn_front_r(int ml, int mr) {
    frml = ml;
    frmr = mr;
}

void to_brake_fc(int ff, int fc) {
    break_ff = ff;
    break_fc = fc;
}

void to_brake_bc(int bf, int bc) {
    break_bf = bf;
    break_bc = bc;
}

void to_delay_f(int ff) {
    delay_f = ff;
}

void to_speed_turn_fl(int inM, int outM, int delayM) {
    llmotor = inM;
    lrmotor = outM;
    ldelaymotor = delayM;
}

void to_speed_turn_fr(int inM, int outM, int delayM) {
    rlmotor = inM;
    rrmotor = outM;
    rdelaymotor = delayM;
}

void kd_fw(float kd) {
    kd_f = kd;
}

void kd_bw(float kd) {
    kd_b = kd;
}

void kp_sl(float kp_sl, float ki_sl) {
    kp_slow = kp_sl;
    ki_slow = ki_sl;
}

void turn_speed_fl() {
    for (int t = 0; t < ldelaymotor; t++) {
        errors = error_A();
        updateBattery();
P = errors;
        I += errors * (ramp_delay / 1000.0);
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (0.45 * P) + (0.00001 * I) + (kd_f * D);
        Motor(llmotor + PID_output, lrmotor - PID_output);
        delay(ramp_delay);
    }
}

void turn_speed_fr() {
    for (int t = 0; t < rdelaymotor; t++) {
        errors = error_A();
        updateBattery();
P = errors;
        I += errors * (ramp_delay / 1000.0);
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (0.45 * P) + (0.00001 * I) + (kd_f * D);
        Motor(rlmotor + PID_output, rrmotor - PID_output);
        delay(ramp_delay);
    }
}

void bturn_speed_fl() {
    for (int t = 0; t < ldelaymotor; t++) {
        errors = error_A();
        updateBattery();
P = errors;
        I += errors * (ramp_delay / 1000.0);
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (0.45 * P) + (0.00001 * I) + (kd_b * D);
        Motor(-(llmotor + PID_output), -((lrmotor - 10) - PID_output));
        delay(ramp_delay);
    }
    delay(2);
}

void bturn_speed_fr() {
    for (int t = 0; t < rdelaymotor; t++) {
        errors = error_A();
        updateBattery();
P = errors;
        I += errors * (ramp_delay / 1000.0);
        D = errors - previous_error;
        previous_error = errors;
        PID_output = (0.45 * P) + (0.00001 * I) + (kd_b * D);
        Motor(-(rlmotor + PID_output), -((rrmotor - 10) - PID_output));
        delay(ramp_delay);
    }
    delay(2);
}


// ============================================================================

   
// ================================================
// ค่าคงที่สำหรับ fline() และ bline()
// ================================================
const float CRUISE_PWM_VAL    = 95.0f;
const float MIN_PWM_VAL       = 15.0f;
const float ACCEL_RATE_PWM_S  = 100.0f;
const float DECEL_RATE_PWM_S  = 100.0f;
const float SPEED_FACTOR_CM_S = 2.4f;     // ค่านี้สำคัญมากสำหรับระยะทาง

bool crossedLine = false;

// ================================================
// fline() - เวอร์ชันแก้ไขให้วิ่งระยะทางใกล้เคียงจริงมากขึ้น
// ================================================
void fline(int spl, int spr, float kp, String target, char nfc, char splr, int power, String sensor, int endt)
{
     _fw = true;
    float distance = target.toFloat();

    char sensors[4];
    sensor.toCharArray(sensors, 4);
    int sensor_f = atoi(&sensors[1]);

    // ==============================

    float cruise = min((float)min(spl, spr), CRUISE_PWM_VAL);
    float minp   = max(MIN_PWM_VAL, cruise * 0.3f);

    float I_local = 0.0f, prevE = 0.0f;
    if (kp == 0.0f) kp_slow = ki_slow = 0.0f;
    pid_error = (kp < 6.5f);

    const float Kd_min = 0.015f;
    const float Kd_max = 0.070f;

    if (spl == 0) goto _line;

    // =============== ส่วนหลัก: วิ่งตรง + Accel/Cruise/Decel + PID ===============
    {
        unsigned long t0 = millis(), prevT = t0;
        float pwm = minp, dist = 0.0f, prevBase = minp;
        bool accel = true, cruise_mode = false, decel = false;
        unsigned long decel_start = 0;

        float decel_time = (cruise - minp) / DECEL_RATE_PWM_S;

        bool skipAccel = crossedLine;

        while (true)
        {
            unsigned long now = millis();
            float dt = (now - prevT) / 1000.0f;
            if (dt < 0.001f) dt = 0.001f;
            prevT = now;

            // PID Error
            if (kp <= 0.45f) {
                if (read_sensorA(2)>md_sensorA(2) && read_sensorA(3)>md_sensorA(3) &&
                    read_sensorA(4)>md_sensorA(4) && read_sensorA(5)>md_sensorA(5)) errors = 0;
                else if (read_sensorA(5)<md_sensorA(5) && read_sensorA(6)<md_sensorA(6) && read_sensorA(7)<md_sensorA(7)) errors = 10;
                else if (read_sensorA(2)<md_sensorA(2) && read_sensorA(1)<md_sensorA(1) && read_sensorA(0)<md_sensorA(0)) errors = -10;
                else errors = error_AA();
            } else {
                errors = error_AA();
            }

            Serial.println(errors); 

            updateBattery();
            I_local += errors * dt;

            float speed_factor = (pwm - MIN_PWM_VAL) / (CRUISE_PWM_VAL - MIN_PWM_VAL + 1.0f);
            speed_factor = constrain(speed_factor, 0.0f, 1.0f);
            float current_Kd = Kd_min + (Kd_max - Kd_min) * speed_factor * speed_factor;

            float PID = kp * errors + 0.0001f * I_local + current_Kd * (errors - prevE);
            prevE = errors;

            float base_pwm;
            String mode = target;
            bool isSpecialMode = (mode == "a07" || mode == "a70" || 
                                  mode == "FRL" || mode == "frl" || 
                                  mode == "FLR" || mode == "flr");

            if (isSpecialMode) {
                base_pwm = cruise;
            }
            else if (isDigit(mode[0])) {
                if (skipAccel  || endt == 0) {
                    base_pwm = cruise;                // ข้ามเส้นมา → วิ่งปกติทันที ไม่เร่ง
                }
                else if (accel) {
                    pwm += ACCEL_RATE_PWM_S * dt;
                    if (pwm >= cruise) {
                        pwm = cruise;
                        accel = false;
                        cruise_mode = true;
                    }
                    base_pwm = pwm;
                } 
                else if (cruise_mode) {
                    base_pwm = cruise;
                    if (dist >= distance * 0.65f) {
                        cruise_mode = false;
                        decel = true;
                        decel_start = now;
                    }
                } 
                else if (decel) {
                    float decel_rate = (pwm < 40.0f) ? DECEL_RATE_PWM_S * 0.5f : DECEL_RATE_PWM_S;
                    pwm -= decel_rate * dt;
                    if (pwm < minp) pwm = minp;
                    base_pwm = pwm;
                    if (now - decel_start > (unsigned long)(decel_time * 2000.0f)) break;
                }
            }
            else {
                base_pwm = cruise;
            }

            dist += ((prevBase + base_pwm) / 5.0f) * SPEED_FACTOR_CM_S * dt;
            prevBase = base_pwm;

            // ตรวจจับหยุด
            bool should_stop = false;
            if (isDigit(mode[0])) {
                if (dist >= distance * 0.85f) 
                  {should_stop = true;}
            }
            else if (mode == "FL" || mode == "fl" || mode == "a0" || mode == "a1") {
                if (read_sensorA(0) < md_sensorA(0)) should_stop = true;
            }
            else if (mode == "FR" || mode == "fr" || mode == "a7" || mode == "a6") {
                if (read_sensorA(7) < md_sensorA(7)) should_stop = true;
            }
            else if (mode == "BL" || mode == "bl" || mode == "b7") {
                if (read_sensorB(7) < md_sensorB(7)) should_stop = true;
            }
            else if (mode == "BR" || mode == "br" || mode == "b0") {
                if (read_sensorB(0) < md_sensorB(0)) should_stop = true;
            }
            else if (mode == "CL" || mode == "cl" || mode == "27") {
                if (analogRead(27) < (sensorMinC[1] + md_sensorC(1)) / 2.0f) should_stop = true;
            }
            else if (mode == "CR" || mode == "cr" || mode == "26") {
                if (analogRead(26) < (sensorMinC[0] + md_sensorC(0)) / 2.0f) should_stop = true;
            }
            else if (mode == "a07" || mode == "a70" || mode == "FRL" || mode == "frl" || 
                     mode == "FLR" || mode == "flr") {
                if (read_sensorA(0) < md_sensorA(0) && read_sensorA(7) < md_sensorA(7)) should_stop = true;
            }

            if (should_stop) break;

            Motor(constrain((int)(base_pwm - PID), -100, 100),
                  constrain((int)(base_pwm + PID), -100, 100));
            delayMicroseconds(55);
        }
        Motor(0, 0);
    }

    _line:
    delay(10);
    // ch_p = 0;   ← ตัดออกตามที่คุณต้องการ

    // รีเซ็ตสถานะ crossedLine เมื่อเจอ splr ที่ไม่ใช่ 'p'
    if (splr != 'p') {
        crossedLine = false;
    }

    // Lambda PID Loop
    auto pidLoop = [&](float kpu, float ki, float kd_base, int bl, int br, int dly, auto&& stopCond) {
        while (true) {
            if (kp <= 0.65f) {
                if (read_sensorA(2)>md_sensorA(2)&&read_sensorA(3)>md_sensorA(3)&&
                    read_sensorA(4)>md_sensorA(4)&&read_sensorA(5)>md_sensorA(5)) errors=0;
                else if (read_sensorA(5)<md_sensorA(5)&&read_sensorA(6)<md_sensorA(6)&&read_sensorA(7)<md_sensorA(7)) errors=10;
                else if (read_sensorA(2)<md_sensorA(2)&&read_sensorA(1)<md_sensorA(1)&&read_sensorA(0)<md_sensorA(0)) errors=-10;
                else errors = error_A();
            } else errors = error_A();

            updateBattery();
            I += errors * ki;

            float speed_factor = (float)abs(bl) / 85.0f;
            float current_Kd = kd_base * (1.0f + 1.2f * speed_factor);

            float PID = kpu * errors + 0.000001f * I + current_Kd * (errors - prevE);
            prevE = errors;

            Motor(bl - PID, br + PID);
            delayMicroseconds(dly);
            if (stopCond()) break;
        }
    };

    // =============== โหมด nfc ===============
    if (nfc == 'n') {
        if (splr == 'p') {
            crossedLine = true;        // เก็บสถานะ 'p'
            pidLoop(kp_slow, 0.00005f, 0.025f, slmotor, srmotor, 80, [](){
                return (read_sensorA(0)<md_sensorA(0)&&read_sensorA(1)<md_sensorA(1)) ||
                       (read_sensorA(7)<md_sensorA(7)&&read_sensorA(6)<md_sensorA(6));
            });
        }
        else if (splr == 's') {
            if (endt > 0) {
                Motor(-(spl/3), -(spr/30)); delay(endt); Motor(-1, -1); delay(10);
            } else {
                    Motor(0, 0);
                    crossedLine = true;               
            }
            goto _entN;
        }
    }
    else if (nfc == 'f') {
        if (distance > 0.0f) {
            if (spl == 0) {
                pidLoop(kp_slow, 0.00005f, 0.025f, slmotor, srmotor, 80, [](){
                    return (read_sensorA(0)<md_sensorA(0)-50 && read_sensorA(1)<md_sensorA(1)-50) ||
                           (read_sensorA(7)<md_sensorA(7)-50 && read_sensorA(6)<md_sensorA(6)-50);
                });
            } else {
                float k = (kp > 0.65f) ? kp : kp_slow;
                pidLoop(k, 0.00005f, 0.025f, slmotor, srmotor, 80, [&](){
                    if (kp >= 5.5f)
                        return (read_sensorA(0)<md_sensorA(0)&&read_sensorA(1)<md_sensorA(1)&&read_sensorA(2)<md_sensorA(2)) ||
                               (read_sensorA(7)<md_sensorA(7)&&read_sensorA(6)<md_sensorA(6)&&read_sensorA(5)<md_sensorA(5));
                    return (read_sensorA(0)<md_sensorA(0)&&read_sensorA(1)<md_sensorA(1)) ||
                           (read_sensorA(7)<md_sensorA(7)&&read_sensorA(6)<md_sensorA(6));
                });
            }
        }
        if (splr == 'p') {
            crossedLine = true;
            while (true) { 
                Motor(spl, spr); 
                delayMicroseconds(80); 
                if (read_sensorA(0) > md_sensorA(0) && read_sensorA(7) > md_sensorA(7)) break; 
            }
            if (endt > 0) { Motor(-spl, -spr); delay(endt); Motor(-1, -1); delay(10); }
            else Motor(0, 0);
        }
        else if (splr == 's') { 
            Motor(-spl, -spr); delay(endt); Motor(0, 0); delay(2); 
        }
    }
    else if (nfc == 'c') {
        auto stopC = [](){
            return analogRead(26) < (sensorMinC[0] + md_sensorC(0))/2 ||
                   analogRead(27) < (sensorMinC[1] + md_sensorC(1))/2;
        };
        if (splr == 'p') {
            crossedLine = true;
            pidLoop(kp/7.5f, 1e-7f, 0.0125f, spl, spr, 80, stopC);
            if (endt > 0) { Motor(spl, spr); delay(endt); Motor(0, 0); delay(10); }
            else Motor(0, 0);
        } else {
            if (distance > 0.0f) pidLoop(kp/7.5f, 0.00000001f, 0.00000125f, slmotor, slmotor, 80, stopC);
            pidLoop(kp/5.5f, 1e-7f, 0.0125f, slmotor, slmotor, 80, stopC);
        }
        if (splr == 's') { Motor(spl, spr); delay(endt); Motor(0,0); delay(2); }
    }


    // =============== หมุนซ้าย / ขวา ===============
    if (splr == 'l' || splr == 'r') {
        bool isLeft = (splr == 'l');
        if (nfc == 'f') {
            while (true) {
                delayMicroseconds(80);
                Motor(slmotor, srmotor);
                if (read_sensorA(7) > md_sensorA(7) && read_sensorA(0) > md_sensorA(0)) {
                    delay(delay_f); break;
                }
            }
            Motor(-slmotor, -srmotor); delay(break_ff);

            if (isLeft) {
                for (int i = 0; i <= sensor_f; i++) {
                    do { Motor((flml*power)/100, (flmr*power)/100); delayMicroseconds(80); }
                    while (read_sensorA(i) > md_sensorA(i) - 50);
                }
            } else {
                for (int i = 7; i >= sensor_f; i--) {
                    do { Motor((frml*power)/100, (frmr*power)/100); delayMicroseconds(80); }
                    while (read_sensorA(i) > md_sensorA(i) - 50);
                }
            }
        } 
        else {
            Motor((nfc == 'n' ? 0 : -slmotor), (nfc == 'n' ? 0 : -srmotor));
            delay(nfc == 'n' ? 2 : break_fc);

            int start_i = (sensor[0] == 'a' && ((isLeft && sensor_f >= 5) || (!isLeft && sensor_f <= 2))) ? 5 : 0;

            if (isLeft) {
                for (int i = start_i; i <= sensor_f; i++) {
                    do {
                        Motor((clml*power)/100, (clmr*power)/100);
                        delayMicroseconds(80);
                    } while ((sensor[0]=='a' ? read_sensorA(i) : read_sensorB(i)) > 
                             (sensor[0]=='a' ? md_sensorA(i) : md_sensorB(i)));
                }
            } else {
                for (int i = 7; i >= (sensor[0]=='a' ? sensor_f - start_i : sensor_f); i--) {
                    do {
                        Motor((crml*power)/100, (crmr*power)/100);
                        delayMicroseconds(80);
                    } while ((sensor[0]=='a' ? read_sensorA(i - start_i) : read_sensorB(i)) > 
                             (sensor[0]=='a' ? md_sensorA(i - start_i) : md_sensorB(i)));
                }
            }
        }

        if (endt == 0) {
            if (isLeft) turn_speed_fl(); else turn_speed_fr();
        } else {
            int ml = isLeft ? clml : crml;
            int mr = isLeft ? clmr : crmr;
            Motor( -(ml*power)/100, -(mr*power)/100 );
            delay(endt); Motor(-1, -1); delay(10);
        }
    }
 setpoint_fw = false;
_entN:
    delay(5);
}


// ================================================
// ฟังก์ชัน bline() - ถอยหลังตามเส้น (โครงสร้างเหมือน fline)
// ================================================
void bline(int spl, int spr, float kp, String target, char nfc, char splr, int power, String sensor, int endt)
{
    _fw = false;                    // สำคัญ: กำลังถอยหลัง
     float distance = target.toFloat();

    char sensors[4];
    sensor.toCharArray(sensors, 4);
    int sensor_f = atoi(&sensors[1]);

    // ==============================

    float cruise = min((float)min(spl, spr), CRUISE_PWM_VAL);
    float minp   = max(MIN_PWM_VAL, cruise * 0.3f);

    float I_local = 0.0f, prevE = 0.0f;
    if (kp == 0.0f) kp_slow = ki_slow = 0.0f;
    pid_error = (kp < 6.5f);

    const float Kd_min = 0.015f;
    const float Kd_max = 0.070f;

    if (spl == 0) goto _line;

    // =============== ส่วนหลัก: วิ่งตรง + Accel/Cruise/Decel + PID ===============
    {
        unsigned long t0 = millis(), prevT = t0;
        float pwm = minp, dist = 0.0f, prevBase = minp;
        bool accel = true, cruise_mode = false, decel = false;
        unsigned long decel_start = 0;

        float decel_time = (cruise - minp) / DECEL_RATE_PWM_S;

        bool skipAccel = crossedLine;

        while (true)
        {
            unsigned long now = millis();
            float dt = (now - prevT) / 1000.0f;
            if (dt < 0.001f) dt = 0.001f;
            prevT = now;

            // PID Error
            if (kp <= 0.45f) {
                if (read_sensorA(2)>md_sensorA(2) && read_sensorA(3)>md_sensorA(3) &&
                    read_sensorA(4)>md_sensorA(4) && read_sensorA(5)>md_sensorA(5)) errors = 0;
                else if (read_sensorA(5)<md_sensorA(5) && read_sensorA(6)<md_sensorA(6) && read_sensorA(7)<md_sensorA(7)) errors = 10;
                else if (read_sensorA(2)<md_sensorA(2) && read_sensorA(1)<md_sensorA(1) && read_sensorA(0)<md_sensorA(0)) errors = -10;
                else errors = error_BB();
            } else {
                errors = error_BB();
            }

            Serial.println(errors); 

            updateBattery();
            I_local += errors * dt;

            float speed_factor = (pwm - MIN_PWM_VAL) / (CRUISE_PWM_VAL - MIN_PWM_VAL + 1.0f);
            speed_factor = constrain(speed_factor, 0.0f, 1.0f);
            float current_Kd = Kd_min + (Kd_max - Kd_min) * speed_factor * speed_factor;

            float PID = kp * errors + 0.0001f * I_local + current_Kd * (errors - prevE);
            prevE = errors;

            float base_pwm;
            String mode = target;
            bool isSpecialMode = (mode == "a07" || mode == "a70" || 
                                  mode == "FRL" || mode == "frl" || 
                                  mode == "FLR" || mode == "flr");

            if (isSpecialMode) {
                base_pwm = cruise;
            }
            else if (isDigit(mode[0])) {
                if (skipAccel  || endt == 0) {
                    base_pwm = cruise;                // ข้ามเส้นมา → วิ่งปกติทันที ไม่เร่ง
                }
                else if (accel) {
                    pwm += ACCEL_RATE_PWM_S * dt;
                    if (pwm >= cruise) {
                        pwm = cruise;
                        accel = false;
                        cruise_mode = true;
                    }
                    base_pwm = pwm;
                } 
                else if (cruise_mode) {
                    base_pwm = cruise;
                    if (dist >= distance * 0.65f) {
                        cruise_mode = false;
                        decel = true;
                        decel_start = now;
                    }
                } 
                else if (decel) {
                    float decel_rate = (pwm < 40.0f) ? DECEL_RATE_PWM_S * 0.5f : DECEL_RATE_PWM_S;
                    pwm -= decel_rate * dt;
                    if (pwm < minp) pwm = minp;
                    base_pwm = pwm;
                    if (now - decel_start > (unsigned long)(decel_time * 2000.0f)) break;
                }
            }
            else {
                base_pwm = cruise;
            }

            dist += ((prevBase + base_pwm) / 5.0f) * SPEED_FACTOR_CM_S * dt;
            prevBase = base_pwm;

            // ตรวจจับหยุด
            bool should_stop = false;
            if (isDigit(mode[0])) {
                if (dist >= distance * 0.85f) 
                  {should_stop = true;}
            }
            
            else if (mode == "FL" || mode == "fl" || mode == "a0" || mode == "a1") {
                if (read_sensorA(0) < md_sensorA(0)) should_stop = true;
            }
            else if (mode == "FR" || mode == "fr" || mode == "a7" || mode == "a6") {
                if (read_sensorA(7) < md_sensorA(7)) should_stop = true;
            }
            else if (mode == "BL" || mode == "bl" || mode == "b7") {
                if (read_sensorB(7) < md_sensorB(7)) should_stop = true;
            }
            else if (mode == "BR" || mode == "br" || mode == "b0") {
                if (read_sensorB(0) < md_sensorB(0)) should_stop = true;
            }
            else if (mode == "CL" || mode == "cl" || mode == "27") {
                if (analogRead(27) < (sensorMinC[1] + md_sensorC(1)) / 2.0f) {
                    should_stop = true;
                }
            }
            else if (mode == "CR" || mode == "cr" || mode == "26") {
                if (analogRead(26) < (sensorMinC[0] + md_sensorC(0)) / 2.0f) {
                    should_stop = true;
                }
            }
            else if (mode == "a07" || mode == "a70" || mode == "FRL" || mode == "frl" || 
                     mode == "FLR" || mode == "flr") {
                if (read_sensorA(0) < md_sensorA(0) && read_sensorA(7) < md_sensorA(7)) {
                    should_stop = true;
                }
            }
            else if (mode == "b07" || mode == "b70" || mode == "BRL" || mode == "brl" || 
                     mode == "BLR" || mode == "blr") {
                if (read_sensorB(0) < md_sensorB(0) && read_sensorB(7) < md_sensorB(7)) {
                    should_stop = true;
                }
            }

            if (should_stop) break;

            // ส่ง PWM ถอยหลัง
            Motor(constrain((int)(-(base_pwm + PID)), -100, 100),
                  constrain((int)(-(base_pwm - PID)), -100, 100));
            delayMicroseconds(60);
        }
        Motor(0, 0);
        
    }

_line:
    delay(10);

    // รีเซ็ตสถานะ crossedLine เมื่อเจอ splr ที่ไม่ใช่ 'p'
    if (splr != 'p') {
        crossedLine = false;
    }

    // Lambda PID Loop (สำหรับส่วน nfc)
    auto pidLoop = [&](float kpu, float ki, float kd_base, int bl, int br, int dly, auto&& stopCond) {
        while (true) {
            if (kp <= 0.65f) {
                if (read_sensorB(2)>md_sensorB(2)&&read_sensorB(3)>md_sensorB(3)&&
                    read_sensorB(4)>md_sensorB(4)&&read_sensorB(5)>md_sensorB(5)) errors=0;
                else if (read_sensorB(5)<md_sensorB(5)&&read_sensorB(6)<md_sensorB(6)&&read_sensorB(7)<md_sensorB(7)) errors=10;
                else if (read_sensorB(2)<md_sensorB(2)&&read_sensorB(1)<md_sensorB(1)&&read_sensorB(0)<md_sensorB(0)) errors=-10;
                else errors = error_B();
            } else errors = error_B();

            updateBattery();
            I += errors * ki;

            float speed_factor = (float)abs(bl) / 85.0f;
            float current_Kd = kd_base * (1.0f + 1.2f * speed_factor);

            float PID = kpu * errors + 0.000001f * I + current_Kd * (errors - prevE);
            prevE = errors;

            Motor(-(bl + PID), -(br - PID));        // สำหรับถอยหลังจะถูกปรับเครื่องหมายใน caller
            delayMicroseconds(dly);
            if (stopCond()) break;
        }
    };

    // =============== โหมด nfc ===============
    if (nfc == 'n') {
        if (splr == 'p') {
            crossedLine = true;
            pidLoop(kp_slow, 0.00005f, 0.025f, slmotor, srmotor, 80, [](){
                return (read_sensorB(0)<md_sensorB(0)&&read_sensorB(1)<md_sensorB(1)) ||
                       (read_sensorB(7)<md_sensorB(7)&&read_sensorB(6)<md_sensorB(6));
            });
        }
        else if (splr == 's') {
            if (endt > 0) {
                Motor(3, 3); delay(endt); Motor(1, 1); delay(10);   // เบรก
            } else {
                    Motor(0, 0);
                    crossedLine = true;               
            }
            goto _entN;
        }
    }
    else if (nfc == 'f') {
        if (distance > 0.0f) {
            if (spl == 0) {
                pidLoop(kp_slow, 0.00005f, 0.025f, slmotor, srmotor, 80, [](){
                    return (read_sensorB(0)<md_sensorB(0)-50 && read_sensorB(1)<md_sensorB(1)-50) ||
                           (read_sensorB(7)<md_sensorB(7)-50 && read_sensorB(6)<md_sensorB(6)-50);
                });
            } else {
                float k = (kp > 0.65f) ? kp : kp_slow;
                pidLoop(k, 0.00005f, 0.025f, slmotor, srmotor, 80, [&](){
                    if (kp >= 5.5f)
                        return (read_sensorB(0)<md_sensorB(0)&&read_sensorB(1)<md_sensorB(1)&&read_sensorB(2)<md_sensorB(2)) ||
                               (read_sensorB(7)<md_sensorB(7)&&read_sensorB(6)<md_sensorB(6)&&read_sensorB(5)<md_sensorB(5));
                    return (read_sensorB(0)<md_sensorB(0)&&read_sensorB(1)<md_sensorB(1)) ||
                           (read_sensorB(7)<md_sensorB(7)&&read_sensorB(6)<md_sensorB(6));
                });
            }
        }
        if (splr == 'p') {
            crossedLine = true;
            while (true) { 
                Motor(-spl, -spr); 
                delayMicroseconds(80); 
                if (read_sensorB(0) > md_sensorB(0) && read_sensorB(7) > md_sensorB(7)) break; 
            }
            if (endt > 0) { Motor(spl, spr); delay(endt); Motor(0, 0); delay(10); }
            else Motor(0, 0);
        }
        else if (splr == 's') { 
            Motor(spl, spr); delay(endt); Motor(0, 0); delay(2); 
        }
    }
    else if (nfc == 'c') {
        auto stopC = [](){
            return analogRead(26) < (sensorMinC[0] + md_sensorC(0))/2 ||
                   analogRead(27) < (sensorMinC[1] + md_sensorC(1))/2;
        };
        if (splr == 'p') {
            crossedLine = true;
            pidLoop(kp/7.5f, 1e-7f, 0.0125f, spl, spr, 80, stopC);
            if (endt > 0) { Motor(spl, spr); delay(endt); Motor(0, 0); delay(10); }
            else Motor(0, 0);
        } else {
            if (distance > 0.0f) pidLoop(kp/7.5f, 0.00000001f, 0.00000125f, slmotor, slmotor, 80, stopC);
            pidLoop(kp/5.5f, 1e-7f, 0.0125f, slmotor, slmotor, 80, stopC);
        }
        if (splr == 's') { Motor(spl, spr); delay(endt); Motor(0,0); delay(2); }
    }

    // =============== หมุนซ้าย / ขวา (เหมือน fline แต่ปรับสำหรับถอยหลัง) ===============
    if (splr == 'l' || splr == 'r') {
        bool isLeft = (splr == 'l');
        if (nfc == 'f') {
            while (true) {
                delayMicroseconds(80);
                Motor(-slmotor, -srmotor);                    // ถอยหลัง
                if (read_sensorB(7) > md_sensorB(7) && read_sensorB(0) > md_sensorB(0)) {
                    delay(delay_f); break;
                }
            }
            Motor(slmotor, srmotor); delay(break_ff);

            if (isLeft) {
                for (int i = 0; i <= sensor_f; i++) {
                    do { Motor(-((flml*power)/100), -((flmr*power)/100)); delayMicroseconds(80); }
                    while (read_sensorB(i) > md_sensorB(i) - 50);
                }
            } else {
                for (int i = 7; i >= sensor_f; i--) {
                    do { Motor(-((frml*power)/100), -((frmr*power)/100)); delayMicroseconds(80); }
                    while (read_sensorB(i) > md_sensorB(i) - 50);
                }
            }
        } 
        else {
            Motor((nfc == 'n' ? 0 : slmotor), (nfc == 'n' ? 0 : srmotor));
            delay(nfc == 'n' ? 2 : break_fc);

            int start_i = (sensor[0] == 'a' && ((isLeft && sensor_f >= 5) || (!isLeft && sensor_f <= 2))) ? 5 : 0;

            if (isLeft) {
                for (int i = start_i; i <= sensor_f; i++) {
                    do {
                        Motor((clml*power)/100, (clmr*power)/100);
                        delayMicroseconds(80);
                    } while ((sensor[0]=='a' ? read_sensorA(i) : read_sensorB(i)) > 
                             (sensor[0]=='a' ? md_sensorA(i) : md_sensorB(i)));
                }
            } else {
                for (int i = 7; i >= (sensor[0]=='a' ? sensor_f - start_i : sensor_f); i--) {
                    do {
                        Motor((crml*power)/100, (crmr*power)/100);
                        delayMicroseconds(80);
                    } while ((sensor[0]=='a' ? read_sensorA(i - start_i) : read_sensorB(i)) > 
                             (sensor[0]=='a' ? md_sensorA(i - start_i) : md_sensorB(i)));
                }
            }
        }

        if (endt == 0) {
            if (isLeft) turn_speed_fl(); else turn_speed_fr();
        } else {
            int ml = isLeft ? clml : crml;
            int mr = isLeft ? clmr : crmr;
            Motor( -(ml*power)/100, -(mr*power)/100 );
            delay(endt); Motor(-1, -1); delay(10);
        }
    }

_entN:
    delay(5);
}


// ================================================
// ตรวจสอบสถานะ Gyro (ปรับตามผลสแกนของคุณ)
// ================================================
bool gyroOK()
{
    Serial.println("=== ตรวจสอบ Gyro ===");

    // วิธี 1: ตรวจโดยตรงด้วย I2C Scanner (แม่นยำที่สุด)
    Wire.beginTransmission(0x68);
    byte error68 = Wire.endTransmission();

    Wire.beginTransmission(0x69);
    byte error69 = Wire.endTransmission();

    bool hasGyro = (error68 == 0) || (error69 == 0);

    if (hasGyro) {
        Serial.println("พบ Gyro ที่ address 0x68 หรือ 0x69");
        
        // ตรวจเพิ่มว่าอ่านค่าได้ปกติไหม
        float g = my.gyro('z');
        if (isnan(g)) {
            Serial.println("แต่ไม่สามารถอ่านค่า gyro('z') ได้ (NaN)");
            return false;
        }
        
        Serial.printf("อ่านค่าได้: %.2f → Gyro ใช้งานได้\n", g);
        return true;
    } 
    else {
        Serial.println("ไม่พบ Gyro ที่ address 0x68 หรือ 0x69");
        Serial.println("→ ใช้โหมดล้อสำรองแทน");
        return false;
    }
}
// ----------------------- place_left_in เวอร์ชันปลอดภัย -----------------------
void place_left_in(int mr, int degree, int offset)
{
    my.resetAngles();

    float initialDegree = 0.0f;
    for (int i = 0; i < 10; i++) {
        initialDegree += my.gyro('z');
        delay(6);
        }
    initialDegree /= 10.0f;
    float targetDegree = initialDegree + (-(float)degree);

    if (degree == 0 ) {   // ไม่หมุน หรือความเร็วเท่ากัน
    Motor(0, 0);
    return;
        }
        int targetDeg = abs(degree);
        int speedDiff = abs(1 - mr);    // ความแตกต่างความเร็ว → ยิ่งมากยิ่งหมุนเร็ว
        if (speedDiff < 10) {            // ป้องกันหารด้วยศูนย์หรือค่ามากเกิน
            Motor(0, 0);
            return;
        }
        // คำนวณเวลาหมุนหลักจากความเร็ว (หน่วย ms)
        // offset ที่นี่ถูกนำมาใช้ร่วมเป็น "ตัวคูณปรับเทียบ" + เบรค
        long moveTime = ((long)targetDeg * offset * 11L) / speedDiff;  
        // ค่า 12 เป็นตัวคูณเริ่มต้น (ปรับได้ตามหุ่นยนต์จริง)
        // เริ่มหมุนด้วยความเร็วที่กำหนด
        Motor(-1, mr);
        delay(moveTime/5);    
   
   // Serial.print("my.gyro('z') :");Serial.print(my.gyro('z')); Serial.print( "   " );  
   // Serial.print("my.gyro('z') :");Serial.print(abs(my.gyro('z'))); Serial.println( "   " );
   // Serial.print("useGyro :");Serial.print(useGyro); Serial.println( "   " );
    if (abs(my.gyro('z')) > 0 &&  abs(my.gyro('z')) < 360)
    {
        Serial.println("[rotate] → ใช้โหมด Gyro");
        // ==================== โหมด Gyro ====================       
        const float kp = 0.355f;
        const float ki = 0.00000004f;
        const float kd = 0.015f;

        float error = 0.0f, prev_error = 0.0f;
        float integral = 0.0f, output = 0.0f;
        float currentDegree = 0.0f;

        unsigned long lastTime = millis();
        unsigned long startTime = millis();
        const unsigned long timeout = 2200;

        while (true)
        {
            updateBattery();
            currentDegree = my.gyro('z');
            error = targetDegree - currentDegree;

            if (abs(error) < 0.25f && abs(output) < 7.0f) break;

            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0f;
            lastTime = now;

            if (dt > 0.0f) {
                integral += error * dt;
                if (integral > 280) integral = 280;
                if (integral < -280) integral = -280;

                float derivative = (error - prev_error) / dt;
                prev_error = error;

                output = kp * error + ki * integral + kd * derivative;
            }

            int maxSpeed = max(abs(-1), abs(mr));
            output = constrain(output, -maxSpeed, maxSpeed);

            Motor(-1, mr - output);
            delayMicroseconds(60);

            if (millis() - startTime > timeout) break;
        }
    }
    else
    {
        // ==================== โหมดล้อสำรอง (ปรับใหม่ ลดการหมุนเกิน) ====================
        Serial.println("[rotate] Gyro ไม่ทำงาน → ใช้โหมดล้อสำรอง");
        myTone(9, 2600, 60); 
         Motor(-1, mr);
        delay(moveTime);                 // หมุนตามเวลาที่คำนวณ

        Motor(1, -mr);
        delay(offset);                       // เวลาเบรกสั้น ๆ (ปรับได้ 40-70 ms)
        Motor(-1, -1);
        delay(10);
        Motor(0, 0);                     // หยุดสนิท
        
    }

    Motor(-1, -1);
    delay(15);    
}
// ----------------------- place_left_out เวอร์ชันปลอดภัย -----------------------
void place_left_out(int mr, int degree, int offset) 
{
    my.resetAngles();

    float initialDegree = 0.0f;
    for (int i = 0; i < 10; i++) {
        initialDegree += my.gyro('z');
        delay(6);
        }
    initialDegree /= 10.0f;
    float targetDegree = initialDegree + (float)degree;

    if (degree == 0 || 1 == mr) {   // ไม่หมุน หรือความเร็วเท่ากัน
    Motor(0, 0);
    return;
        }
        int targetDeg = abs(degree);
        int speedDiff = abs(1 - mr);    // ความแตกต่างความเร็ว → ยิ่งมากยิ่งหมุนเร็ว
        if (speedDiff < 10) {            // ป้องกันหารด้วยศูนย์หรือค่ามากเกิน
            Motor(0, 0);
            return;
        }
        // คำนวณเวลาหมุนหลักจากความเร็ว (หน่วย ms)
        // offset ที่นี่ถูกนำมาใช้ร่วมเป็น "ตัวคูณปรับเทียบ" + เบรค
        long moveTime = ((long)targetDeg * offset * 11L) / speedDiff;  
        // ค่า 12 เป็นตัวคูณเริ่มต้น (ปรับได้ตามหุ่นยนต์จริง)
        // เริ่มหมุนด้วยความเร็วที่กำหนด
        Motor(1, -mr);
        delay(moveTime/5);    
   
   // Serial.print("my.gyro('z') :");Serial.print(my.gyro('z')); Serial.print( "   " );  
   // Serial.print("my.gyro('z') :");Serial.print(abs(my.gyro('z'))); Serial.println( "   " );
   // Serial.print("useGyro :");Serial.print(useGyro); Serial.println( "   " );
    if (abs(my.gyro('z')) > 0 &&  abs(my.gyro('z')) < 360)
    {
        Serial.println("[rotate] → ใช้โหมด Gyro");
        // ==================== โหมด Gyro ====================       
        const float kp = 0.255f;
        const float ki = 0.00000004f;
        const float kd = 0.015f;

        float error = 0.0f, prev_error = 0.0f;
        float integral = 0.0f, output = 0.0f;
        float currentDegree = 0.0f;

        unsigned long lastTime = millis();
        unsigned long startTime = millis();
        const unsigned long timeout = 2200;

        while (true)
        {
            updateBattery();
            currentDegree = my.gyro('z');
            error = targetDegree - currentDegree;

            if (abs(error) < 0.25f && abs(output) < 7.0f) break;

            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0f;
            lastTime = now;

            if (dt > 0.0f) {
                integral += error * dt;
                if (integral > 280) integral = 280;
                if (integral < -280) integral = -280;

                float derivative = (error - prev_error) / dt;
                prev_error = error;

                output = kp * error + ki * integral + kd * derivative;
            }

            int maxSpeed = max(abs(-1), abs(mr));
            output = constrain(output, -maxSpeed, maxSpeed);

            Motor(1, -(mr + output));
            delayMicroseconds(60);

            if (millis() - startTime > timeout) break;
        }
    }
    else
    {
        // ==================== โหมดล้อสำรอง (ปรับใหม่ ลดการหมุนเกิน) ====================
        Serial.println("[rotate] Gyro ไม่ทำงาน → ใช้โหมดล้อสำรอง");
        myTone(9, 2600, 60); 
         Motor(1, -mr);
        delay(moveTime);                 // หมุนตามเวลาที่คำนวณ

        Motor(-1, mr);
        delay(offset);                       // เวลาเบรกสั้น ๆ (ปรับได้ 40-70 ms)
        Motor(-1, -1);
        delay(10);
        Motor(0, 0);                     // หยุดสนิท
        
    }

    Motor(-1, -1);
    delay(15);    
}

void place_right_in(int mr, int degree, int offset) 
  {
      my.resetAngles();

    float initialDegree = 0.0f;
    for (int i = 0; i < 10; i++) {
        initialDegree += my.gyro('z');
        delay(6);
        }
    initialDegree /= 10.0f;
    float targetDegree = initialDegree + (float)degree;

    if (degree == 0 || mr == 1) {   // ไม่หมุน หรือความเร็วเท่ากัน
    Motor(0, 0);
    return;
        }
        int targetDeg = abs(degree);
        int speedDiff = abs(mr - 1);    // ความแตกต่างความเร็ว → ยิ่งมากยิ่งหมุนเร็ว
        if (speedDiff < 10) {            // ป้องกันหารด้วยศูนย์หรือค่ามากเกิน
            Motor(0, 0);
            return;
        }
        // คำนวณเวลาหมุนหลักจากความเร็ว (หน่วย ms)
        // offset ที่นี่ถูกนำมาใช้ร่วมเป็น "ตัวคูณปรับเทียบ" + เบรค
        long moveTime = ((long)targetDeg * offset * 11L) / speedDiff;  
        // ค่า 12 เป็นตัวคูณเริ่มต้น (ปรับได้ตามหุ่นยนต์จริง)
        // เริ่มหมุนด้วยความเร็วที่กำหนด
        Motor(mr, -1);
        delay(moveTime/5);    
   
   // Serial.print("my.gyro('z') :");Serial.print(my.gyro('z')); Serial.print( "   " );  
   // Serial.print("my.gyro('z') :");Serial.print(abs(my.gyro('z'))); Serial.println( "   " );
   // Serial.print("useGyro :");Serial.print(useGyro); Serial.println( "   " );
    if (abs(my.gyro('z')) > 0 &&  abs(my.gyro('z')) < 360)
    {
        Serial.println("[rotate] → ใช้โหมด Gyro");
        // ==================== โหมด Gyro ====================       
        const float kp = 0.255f;
        const float ki = 0.00000004f;
        const float kd = 0.015f;

        float error = 0.0f, prev_error = 0.0f;
        float integral = 0.0f, output = 0.0f;
        float currentDegree = 0.0f;

        unsigned long lastTime = millis();
        unsigned long startTime = millis();
        const unsigned long timeout = 2200;

        while (true)
        {
            updateBattery();
            currentDegree = my.gyro('z');
            error = targetDegree - currentDegree;

            if (abs(error) < 0.25f && abs(output) < 7.0f) break;

            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0f;
            lastTime = now;

            if (dt > 0.0f) {
                integral += error * dt;
                if (integral > 280) integral = 280;
                if (integral < -280) integral = -280;

                float derivative = (error - prev_error) / dt;
                prev_error = error;

                output = kp * error + ki * integral + kd * derivative;
            }

            int maxSpeed = max(abs(-1), abs(mr));
            output = constrain(output, -maxSpeed, maxSpeed);

            Motor(mr + output, -1);
            delayMicroseconds(60);

            if (millis() - startTime > timeout) break;
        }
    }
    else
    {
        // ==================== โหมดล้อสำรอง (ปรับใหม่ ลดการหมุนเกิน) ====================
        Serial.println("[rotate] Gyro ไม่ทำงาน → ใช้โหมดล้อสำรอง");
        myTone(9, 2600, 60); 
         Motor(mr, -1);
        delay(moveTime);                 // หมุนตามเวลาที่คำนวณ

        Motor(-mr, 1);
        delay(offset);                       // เวลาเบรกสั้น ๆ (ปรับได้ 40-70 ms)
        Motor(-1, -1);
        delay(10);
        Motor(0, 0);                     // หยุดสนิท
        
    }

    Motor(-1, -1);
    delay(15);
}


void place_right_out(int mr, int degree, int offset)
  {
     my.resetAngles();

    float initialDegree = 0.0f;
    for (int i = 0; i < 10; i++) {
        initialDegree += my.gyro('z');
        delay(6);
        }
    initialDegree /= 10.0f;
    float targetDegree = initialDegree + (-(float)degree);

    if (degree == 0 ) {   // ไม่หมุน หรือความเร็วเท่ากัน
    Motor(0, 0);
    return;
        }
        int targetDeg = abs(degree);
        int speedDiff = abs(mr - 1);    // ความแตกต่างความเร็ว → ยิ่งมากยิ่งหมุนเร็ว
        if (speedDiff < 10) {            // ป้องกันหารด้วยศูนย์หรือค่ามากเกิน
            Motor(0, 0);
            return;
        }
        // คำนวณเวลาหมุนหลักจากความเร็ว (หน่วย ms)
        // offset ที่นี่ถูกนำมาใช้ร่วมเป็น "ตัวคูณปรับเทียบ" + เบรค
        long moveTime = ((long)targetDeg * offset * 11L) / speedDiff;  
        // ค่า 12 เป็นตัวคูณเริ่มต้น (ปรับได้ตามหุ่นยนต์จริง)
        // เริ่มหมุนด้วยความเร็วที่กำหนด
        Motor(-mr, 1);
        delay(moveTime/5);    
   
   // Serial.print("my.gyro('z') :");Serial.print(my.gyro('z')); Serial.print( "   " );  
   // Serial.print("my.gyro('z') :");Serial.print(abs(my.gyro('z'))); Serial.println( "   " );
   // Serial.print("useGyro :");Serial.print(useGyro); Serial.println( "   " );
    if (abs(my.gyro('z')) > 0 &&  abs(my.gyro('z')) < 360)
    {
        Serial.println("[rotate] → ใช้โหมด Gyro");
        // ==================== โหมด Gyro ====================       
        const float kp = 0.355f;
        const float ki = 0.00000004f;
        const float kd = 0.015f;

        float error = 0.0f, prev_error = 0.0f;
        float integral = 0.0f, output = 0.0f;
        float currentDegree = 0.0f;

        unsigned long lastTime = millis();
        unsigned long startTime = millis();
        const unsigned long timeout = 2200;

        while (true)
        {
            updateBattery();
            currentDegree = my.gyro('z');
            error = targetDegree - currentDegree;

            if (abs(error) < 0.25f && abs(output) < 7.0f) break;

            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0f;
            lastTime = now;

            if (dt > 0.0f) {
                integral += error * dt;
                if (integral > 280) integral = 280;
                if (integral < -280) integral = -280;

                float derivative = (error - prev_error) / dt;
                prev_error = error;

                output = kp * error + ki * integral + kd * derivative;
            }

            int maxSpeed = max(abs(-1), abs(mr));
            output = constrain(output, -maxSpeed, maxSpeed);

            Motor(-(mr - output), 1);
            delayMicroseconds(60);

            if (millis() - startTime > timeout) break;
        }
    }
    else
    {
        // ==================== โหมดล้อสำรอง (ปรับใหม่ ลดการหมุนเกิน) ====================
        Serial.println("[rotate] Gyro ไม่ทำงาน → ใช้โหมดล้อสำรอง");
        myTone(9, 2600, 60); 
         Motor(-mr, 1);
        delay(moveTime);                 // หมุนตามเวลาที่คำนวณ

        Motor(mr, -1);
        delay(offset);                       // เวลาเบรกสั้น ๆ (ปรับได้ 40-70 ms)
        Motor(-1, -1);
        delay(10);
        Motor(0, 0);                     // หยุดสนิท
        
    }

    Motor(-1, -1);
    delay(15);  
   
}

// ================================================
// ฟังก์ชัน rotate() - เวอร์ชันแก้ไข (ลดหมุนหลายรอบ)
// ================================================
void rotate(int ml, int mr, int degree, int offset)
{
    my.resetAngles();

    float initialDegree = 0.0f;
    for (int i = 0; i < 10; i++) {
        initialDegree += my.gyro('z');
        delay(6);
        }
    initialDegree /= 10.0f;
    float targetDegree = initialDegree + (float)degree;

    if (degree == 0 || ml == mr) {   // ไม่หมุน หรือความเร็วเท่ากัน
    Motor(0, 0);
    return;
        }
        int targetDeg = abs(degree);
        int speedDiff = abs(ml - mr);    // ความแตกต่างความเร็ว → ยิ่งมากยิ่งหมุนเร็ว
        if (speedDiff < 10) {            // ป้องกันหารด้วยศูนย์หรือค่ามากเกิน
            Motor(0, 0);
            return;
        }
        // คำนวณเวลาหมุนหลักจากความเร็ว (หน่วย ms)
        // offset ที่นี่ถูกนำมาใช้ร่วมเป็น "ตัวคูณปรับเทียบ" + เบรค
        long moveTime = ((long)targetDeg * offset * 11L) / speedDiff;  
        // ค่า 12 เป็นตัวคูณเริ่มต้น (ปรับได้ตามหุ่นยนต์จริง)
        // เริ่มหมุนด้วยความเร็วที่กำหนด
        Motor(ml, mr);
        delay(moveTime/5);    
   
   // Serial.print("my.gyro('z') :");Serial.print(my.gyro('z')); Serial.print( "   " );  
   // Serial.print("my.gyro('z') :");Serial.print(abs(my.gyro('z'))); Serial.println( "   " );
   // Serial.print("useGyro :");Serial.print(useGyro); Serial.println( "   " );
    if (abs(my.gyro('z')) > 0 &&  abs(my.gyro('z')) < 360)
    {
        Serial.println("[rotate] → ใช้โหมด Gyro");
        // ==================== โหมด Gyro ====================       
        const float kp = 0.255f;
        const float ki = 0.00000004f;
        const float kd = 0.00015f;

        float error = 0.0f, prev_error = 0.0f;
        float integral = 0.0f, output = 0.0f;
        float currentDegree = 0.0f;

        unsigned long lastTime = millis();
        unsigned long startTime = millis();
        const unsigned long timeout = 2200;

        while (true)
        {
            updateBattery();
            currentDegree = my.gyro('z');
            error = targetDegree - currentDegree;

            if (abs(error) < 0.55f && abs(output) < 7.0f) break;

            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0f;
            lastTime = now;

            if (dt > 0.0f) {
                integral += error * dt;
                if (integral > 280) integral = 280;
                if (integral < -280) integral = -280;

                float derivative = (error - prev_error) / dt;
                prev_error = error;

                output = kp * error + ki * integral + kd * derivative;
            }

            int maxSpeed = max(abs(ml), abs(mr));
            output = constrain(output, -maxSpeed, maxSpeed);

            Motor(ml + output, mr - output);
            delayMicroseconds(60);

            if (millis() - startTime > timeout) break;
        }
    }
    else
    {
        // ==================== โหมดล้อสำรอง (ปรับใหม่ ลดการหมุนเกิน) ====================
        Serial.println("[rotate] Gyro ไม่ทำงาน → ใช้โหมดล้อสำรอง");
        myTone(9, 2600, 60); 
         Motor(ml, mr);
        delay(moveTime);                 // หมุนตามเวลาที่คำนวณ

        Motor(-ml, -mr);
        delay(offset);                       // เวลาเบรกสั้น ๆ (ปรับได้ 40-70 ms)
        Motor(-1, -1);
        delay(10);
        Motor(0, 0);                     // หยุดสนิท
        
    }

    Motor(-1, -1);
    delay(15);    
}
void fw_gyro(int spl, int spr, float kp,  float distance, int offset) 
{     
    int target_speed = min(spl, spr); 
    float traveled_distance = 0;
    unsigned long last_time = millis();
    
    float speed_scale = 1.5;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง

    my.resetAngles();  // รีเซ็ตมุมทั้งหมด
    ////my.reCalibrateGyro();         // calibrate offset ใหม่ (เร็ว ~150 ms)
           // รีเซ็ต bias และ low-pass filter สนิท → ไม่ลอยแน่นอน!
    my.resetAngles();  // รีเซ็ตมุมทั้งหมด
    float yaw_offset = my.gyro('z'); 
    float _integral = 0;
    float _prevErr = 0;
    unsigned long prevT = millis();   

    int maxLeftSpeed = spl;
    int maxRightSpeed = spr; 

    while (1) 
    {
      updateBattery();
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001; 
        prevT = now;

        float yaw = my.gyro('z') - yaw_offset;
        float err = yaw;

        _integral += err * dt;
        float deriv = (err - _prevErr) / dt;
        _prevErr = err;
        float corr = kp * err + 0.0001 * _integral + 0.05 * deriv;

        int leftSpeed  = constrain(spl - corr, -100, 100);
        int rightSpeed = constrain(spr + corr, -100, 100);
        Motor(leftSpeed, rightSpeed);

        if (distance > 0) 
        {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;
            traveled_distance += (target_speed * speed_scale) * delta_time;
            last_time = current_time;

            if (traveled_distance >= distance) break;
        }

        delayMicroseconds(80);
    }

    // soft stop
    if(offset >0)
      {
        Motor(-15, -15); delay(offset);
        Motor(-1, -1);   delay(10);
      }
    else{Motor(0, 0);delay(5);}
    
}

void setrobot_bline(int num)
  {
    for(int i=0; i< num; i++)
      {
        while(1)
          {
            delay(5);      
            if(read_sensorB(0) < (md_sensorB(0)+sensorMinB[0])/2  && read_sensorB(7) > (md_sensorB(7)+sensorMinB[7])/2)
              {
                Motor(-10 ,2);
              }
            else if(read_sensorB(0) > (md_sensorB(0)+sensorMinB[0])/2&& read_sensorB(7) < (md_sensorB(7)+sensorMinB[7])/2)
              {
                Motor(2 ,-10);
              }
            else if(read_sensorB(0) > (md_sensorB(0)+sensorMinB[0])/2 && read_sensorB(7) > (md_sensorB(7)+sensorMinB[7])/2)
              {          
                Motor(-10 ,-10);
              }
            else 
              {
                Motor(1 ,1);
                break;
              }      
          }
        if(num > 1)
          {
            Motor(12 ,12);
            delay(50);
            while(1)
              {
                if(read_sensorB(0) > (md_sensorB(0)+sensorMinB[0])/2 && read_sensorB(7) > (md_sensorB(7)+sensorMinB[7])/2)
                  {
                    break;
                  }
                else
                  {
                    Motor(12 ,12);
                  }
                delay(5);
              }
            Motor(1 ,1);
          }
      }
    
  }

  void setrobot_fline(int num)
  {
    for(int i=0; i< num; i++)
      {
        while(1)
          {
            delay(5);      
            if(read_sensorA(0) < (md_sensorA(0)+sensorMinA[0])/2  && read_sensorA(7) > (md_sensorA(7)+sensorMinA[7])/2)
              {
                Motor(-2 ,10);
              }
            else if(read_sensorA(0) > (md_sensorA(0)+sensorMinA[0])/2&& read_sensorA(7) < (md_sensorA(7)+sensorMinA[7])/2)
              {
                Motor(10 ,-2);
              }
            else if(read_sensorA(0) > (md_sensorA(0)+sensorMinA[0])/2 && read_sensorA(7) > (md_sensorA(7)+sensorMinA[7])/2)
              {          
                Motor(10 ,10);
              }
            else 
              {
                Motor(-1 ,-1);
                break;
              }      
          }
        if(num > 1)
          {
            Motor(-12 ,-12);
            delay(50);
            while(1)
              {
                if(read_sensorA(0) > (md_sensorA(0)+sensorMinA[0])/2 && read_sensorA(7) > (md_sensorA(7)+sensorMinA[7])/2)
                  {
                    break;
                  }
                else
                  {
                    Motor(-12 ,-12);
                  }
                delay(5);
              }
            Motor(1 ,1);
          }
      }
    
  }



#endif