#include "MyRP_Pico2.h"

// ==================== Constructor ====================
MyRP_Pico2::MyRP_Pico2() {
    DC_Motors = true;
    _lastPosition = 2500;
    pid_error = true;
    crossedLine = false;

    slmotor = srmotor = 20;
    redius_wheel = 3.0;
    speed_scale_fw = 1.0;
    speed_scale_bw = 1.05;

    kd_f = 0.55f; 
    kd_b = 0.025f;
    kp_slow = 0.1f; 
    ki_slow = 0.0001f;

    num_steps = 20;
    s10_before_deg = 120; 
    s0_before_deg = 120;
    s1_before_deg = 50; 
    s28_before_deg = 0;

    clml = -90; clmr = 90;
    crml = 90; crmr = -90;
    flml = -15; flmr = 100;
    frml = 100; frmr = -15;
}
// ==================== begin() ====================
void MyRP_Pico2::begin() {
    Serial.begin(115200);
    delay(100);
    Wire.setSDA(4);
    Wire.setSCL(5);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    digitalWrite(4, HIGH);
    digitalWrite(5, HIGH);
    delay(100);
    for (int i = 0; i < 9; i++) {
      digitalWrite(5, LOW); delayMicroseconds(5);
      digitalWrite(5, HIGH); delayMicroseconds(5);
    }

    Wire.end();
    delay(800);
    
    Wire.begin();

    bat.begin();
    analogReadResolution(12);
    analogWriteResolution(12);
    analogWriteFreq(DC_Motors ? 1000 : 20000);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(3, INPUT_PULLUP);
    pinMode(2, INPUT_PULLUP);
    pinMode(9, OUTPUT);

    pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

    adc.begin(14, 15, 16, 13);   // Initialize ADC ครั้งเดียว

    loadCalibration();

    if (my.begin()) {
        my.resetAngles();
    } else {
        Serial.println("BMI160 init failed!");
    }

    blink(3);
    Serial.println("MyRP_Pico2 Class Ready for Competition!");
}

// ==================== Sensor Reading ====================
uint16_t MyRP_Pico2::read_sensorA(int sensor) {
     if (sensor < 0 || sensor > 7) return 0;
     adc.begin(14, 15, 16, 13 );
     adc.begin(14, 15, 16, 17 );  //adc.begin(5, 4, 12, 20 );    // 5=clk, 4=IN, 12=out
     return adc.readADC(sensor);  
}

uint16_t MyRP_Pico2::read_sensorB(int sensor) {
    if (sensor < 0 || sensor > 7) return 0;
     adc.begin(14, 15, 16, 17 ); 
     adc.begin(14, 15, 16, 13 );  //adc.begin(5, 4, 12, 20 );    // 5=clk, 4=IN, 12=out
     return adc.readADC(sensor);  
}

int MyRP_Pico2::md_sensorA(int sensor) {
    return (sensorMaxA[sensor] + sensorMinA[sensor]) / 2;
}

int MyRP_Pico2::md_sensorB(int sensor) {
    return (sensorMaxB[sensor] + sensorMinB[sensor]) / 2;
}

int MyRP_Pico2::md_sensorC(int sensor) {
    return (sensorMaxC[sensor] + sensorMinC[sensor]) / 2;
}

// ==================== EEPROM Helper ====================
void MyRP_Pico2::writeEEPROM(int deviceAddress, unsigned int eeAddress, byte *data, int dataLength) {
    Wire.beginTransmission(deviceAddress);
    Wire.write((int)(eeAddress >> 8));
    Wire.write((int)(eeAddress & 0xFF));
    for (int i = 0; i < dataLength; i++) {
        Wire.write(data[i]);
    }
    Wire.endTransmission();
    delay(5);
}

void MyRP_Pico2::readEEPROM(int deviceAddress, unsigned int eeAddress, byte *buffer, int dataLength) {
    Wire.beginTransmission(deviceAddress);
    Wire.write((int)(eeAddress >> 8));
    Wire.write((int)(eeAddress & 0xFF));
    Wire.endTransmission();
    
    Wire.requestFrom(deviceAddress, dataLength);
    for (int i = 0; i < dataLength; i++) {
        if (Wire.available()) {
            buffer[i] = Wire.read();
        }
    }
}

// ==================== Calibration Functions (เวอร์ชันสมบูรณ์) ====================

void MyRP_Pico2::calibrateA() {
    const int samples = 1000;
    int values[8][1000];

    Serial.println("กำลัง calibrate Sensor A (Line Front)...");

    for (int s = 0; s < samples; s++) {
        for (int i = 0; i < 8; i++) {
            values[i][s] = read_sensorA(i);
            delay(1);
        }
    }

    for (int i = 0; i < 8; i++) {
        sensorMaxA[i] = values[i][0];
        sensorMinA[i] = values[i][0];
        for (int s = 1; s < samples; s++) {
            int v = values[i][s];
            if (v > sensorMaxA[i]) sensorMaxA[i] = v;
            if (v < sensorMinA[i]) sensorMinA[i] = v;
        }
    }

    // บันทึกลง EEPROM
    byte buf[16];
    for (int i = 0; i < 8; i++) {
        buf[i*2]   = highByte(sensorMaxA[i]);
        buf[i*2+1] = lowByte(sensorMaxA[i]);
    }
    writeEEPROM(EEPROM_ADDRESS, 0, buf, 16);

    for (int i = 0; i < 8; i++) {
        buf[i*2]   = highByte(sensorMinA[i]);
        buf[i*2+1] = lowByte(sensorMinA[i]);
    }
    writeEEPROM(EEPROM_ADDRESS, 16, buf, 16);

    beep(3000, 100); delay(150);
    beep(3000, 200);
    Serial.println("✓ Calibrate Sensor A เสร็จสิ้น");
}

void MyRP_Pico2::calibrateB() {
    const int samples = 1000;
    int values[8][1000];

    Serial.println("กำลัง calibrate Sensor B (Line Back)...");

    for (int s = 0; s < samples; s++) {
        for (int i = 0; i < 8; i++) {
            values[i][s] = read_sensorB(i);
            delay(1);
        }
    }

    for (int i = 0; i < 8; i++) {
        sensorMaxB[i] = values[i][0];
        sensorMinB[i] = values[i][0];
        for (int s = 1; s < samples; s++) {
            int v = values[i][s];
            if (v > sensorMaxB[i]) sensorMaxB[i] = v;
            if (v < sensorMinB[i]) sensorMinB[i] = v;
        }
    }

    byte buf[16];
    for (int i = 0; i < 8; i++) {
        buf[i*2]   = highByte(sensorMaxB[i]);
        buf[i*2+1] = lowByte(sensorMaxB[i]);
    }
    writeEEPROM(EEPROM_ADDRESS, 32, buf, 16);

    for (int i = 0; i < 8; i++) {
        buf[i*2]   = highByte(sensorMinB[i]);
        buf[i*2+1] = lowByte(sensorMinB[i]);
    }
    writeEEPROM(EEPROM_ADDRESS, 48, buf, 16);

    beep(3000, 100); delay(150);
    beep(3000, 200);
    Serial.println("✓ Calibrate Sensor B เสร็จสิ้น");
}

void MyRP_Pico2::calibrateC() {
    const int samples = 1000;
    const int PIN_C0 = 26;
    const int PIN_C1 = 27;
    int values[2][1000];

    Serial.println("กำลัง calibrate Sensor C (Side Sensors)...");

    for (int s = 0; s < samples; s++) {
        values[0][s] = analogRead(PIN_C0);
        values[1][s] = analogRead(PIN_C1);
        delay(5);
    }

    for (int i = 0; i < 2; i++) {
        sensorMaxC[i] = 0;
        sensorMinC[i] = 4095;
        for (int s = 0; s < samples; s++) {
            uint16_t v = values[i][s];
            if (v > sensorMaxC[i]) sensorMaxC[i] = v;
            if (v < sensorMinC[i]) sensorMinC[i] = v;
        }
        sensorMaxC[i] = constrain(sensorMaxC[i], 0, 4000);
        sensorMinC[i] = constrain(sensorMinC[i], 0, 4000);
    }

    uint8_t buf[4];
    for (int i = 0; i < 2; i++) {
        buf[i*2]   = highByte(sensorMaxC[i]);
        buf[i*2+1] = lowByte(sensorMaxC[i]);
    }
    writeEEPROM(EEPROM_ADDRESS, 64, buf, 4);

    for (int i = 0; i < 2; i++) {
        buf[i*2]   = highByte(sensorMinC[i]);
        buf[i*2+1] = lowByte(sensorMinC[i]);
    }
    writeEEPROM(EEPROM_ADDRESS, 68, buf, 4);

    beep(3000, 150); delay(200);
    beep(3000, 200);
    Serial.println("✓ Calibrate Sensor C เสร็จสิ้น");
}

// ==================== Load & Print Calibration ====================
void MyRP_Pico2::loadCalibration() {
    read_eepA();
    read_eepB();
    read_eepC();
    printCalibration();
}

void MyRP_Pico2::read_eepA() {
    byte buf[16];
    readEEPROM(EEPROM_ADDRESS, 0, buf, 16);
    for (int i = 0; i < 8; i++) {
        sensorMaxA[i] = (buf[i*2] << 8) | buf[i*2 + 1];
    }
    readEEPROM(EEPROM_ADDRESS, 16, buf, 16);
    for (int i = 0; i < 8; i++) {
        sensorMinA[i] = (buf[i*2] << 8) | buf[i*2 + 1];
    }
}

void MyRP_Pico2::read_eepB() {
    byte buf[16];
    readEEPROM(EEPROM_ADDRESS, 32, buf, 16);
    for (int i = 0; i < 8; i++) {
        sensorMaxB[i] = (buf[i*2] << 8) | buf[i*2 + 1];
    }
    readEEPROM(EEPROM_ADDRESS, 48, buf, 16);
    for (int i = 0; i < 8; i++) {
        sensorMinB[i] = (buf[i*2] << 8) | buf[i*2 + 1];
    }
}

void MyRP_Pico2::read_eepC() {
    byte buf[4];
    readEEPROM(EEPROM_ADDRESS, 64, buf, 4);
    for (int i = 0; i < 2; i++) {
        sensorMaxC[i] = (buf[i*2] << 8) | buf[i*2 + 1];
    }
    readEEPROM(EEPROM_ADDRESS, 68, buf, 4);
    for (int i = 0; i < 2; i++) {
        sensorMinC[i] = (buf[i*2] << 8) | buf[i*2 + 1];
    }
}

void MyRP_Pico2::printCalibration() {
    Serial.println("\n=== Calibration Values Loaded ===");
    Serial.println("--- Sensor A (Front Line) ---");
    for (int i = 0; i < 8; i++) {
        Serial.printf("A%d → Max: %4d  Min: %4d\n", i, sensorMaxA[i], sensorMinA[i]);
    }
    
    Serial.println("--- Sensor B (Back Line) ---");
    for (int i = 0; i < 8; i++) {
        Serial.printf("B%d → Max: %4d  Min: %4d\n", i, sensorMaxB[i], sensorMinB[i]);
    }
    
    Serial.println("--- Sensor C (Side) ---");
    Serial.printf("C0 → Max: %4d  Min: %4d\n", sensorMaxC[0], sensorMinC[0]);
    Serial.printf("C1 → Max: %4d  Min: %4d\n", sensorMaxC[1], sensorMinC[1]);
    Serial.println("=================================\n");
}

// ==================== Motor Control (เวอร์ชันสมบูรณ์สำหรับ Class) ====================
void MyRP_Pico2::Motor(int left, int right)
{
    bat_control();     // ← สำคัญมาก เรียกทุกครั้งเพื่อปรับ scale แบต

    int pwmL = map(abs(left), 0, 100, 0, 4095);
    int pwmR = map(abs(right), 0, 100, 0, 4095);

    // ปรับตาม scale จากแบต
    pwmL = constrain((int)(pwmL * scale), 0, 4095);
    pwmR = constrain((int)(pwmR * scale), 0, 4095);

    // LEFT MOTOR
    if (left > 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else if (left < 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        pwmL = 0;
    }

    // RIGHT MOTOR
    if (right > 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    }
    else if (right < 0) {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
    }
    else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
        pwmR = 0;
    }

    analogWrite(PWMA, pwmL);
    analogWrite(PWMB, pwmR);

    // สำหรับทดสอบเท่านั้น (คอมเมนต์หรือลบเมื่อแข่งจริง)
    Serial.print("Vbat: "); Serial.println(getBatteryVoltage(), 2);
    // Serial.print("  Scale: "); Serial.print(scale, 3);
    // Serial.print("  PWM L:"); Serial.print(pwmL);
    // Serial.print("  R:"); Serial.println(pwmR);
}

// ==================== Battery Control ====================

void MyRP_Pico2::bat_control() {
    float voltage = getBatteryVoltage();

    // ป้องกันการหารด้วยศูนย์และค่าผิดปกติ
    if (voltage > 0.5f && voltage >= VMIN && voltage <= VMAX) {
        scale = pow(VNOM / voltage, 0.95f);
        batteryUsed = true;
    } else {
        scale = 1.0f;
        batteryUsed = false;
    }
}

void MyRP_Pico2::updateBattery() {
    bat.update();
}

float MyRP_Pico2::getBatteryVoltage() {
    return bat.getVoltage();
}

// ==================== Servo Control ====================
// ==================== Servo Control ====================
void MyRP_Pico2::servo(int num, int angle) {
    if (num == 10) {
        servo_10.attach(10, 500, 2500);   // พิน servo10
        servo_10.write(angle);
    } 
    else if (num == 0) {
        servo_0.attach(0, 500, 2500);     // พิน servo0
        servo_0.write(180 - angle);
    } 
    else if (num == 1) {
        servo_1.attach(1, 500, 2500);     // พิน servo1
        servo_1.write(angle);
    } 
    else if (num == 28) {
        servo_28.attach(28, 500, 2500);   // พิน servo28
        servo_28.write(angle + servo_tim28);
    }
}

void MyRP_Pico2::armLeftRight(float sl, float sr, int sp) {
    float step10 = (sl - s10_before_deg) / num_steps;
    float step1  = (sr - s1_before_deg)  / num_steps;

    for (int i = 0; i < num_steps; i++) {
        servo(10, s10_before_deg + i * step10);
        servo(1,  s1_before_deg  + i * step1);
        delay(sp);
    }
    s10_before_deg = sl;
    s1_before_deg  = sr;
}

void MyRP_Pico2::armUpDown(float sl, int sp) {
    float step0 = (sl - s0_before_deg) / num_steps;
    for (int i = 0; i < num_steps; i++) {
        servo(0, s0_before_deg + i * step0);
        delay(sp);
    }
    s0_before_deg = sl;
}

void MyRP_Pico2::servoTrim(int s0, int s1, int s10, int s28) {
    servo_tim0  = s0;
    servo_tim1  = s1;
    servo_tim10 = s10;
    servo_tim28 = s28;
}

// ==================== Utility ====================
void MyRP_Pico2::beep(int freq, int dur) {
    tone(9, freq, dur);
    delay(dur + 50);
}

void MyRP_Pico2::blink(int times) {
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_BUILTIN, HIGH); delay(150);
        digitalWrite(LED_BUILTIN, LOW);  delay(150);
    }
}

void MyRP_Pico2::waitButton() {
    while (digitalRead(3) == HIGH); delay(200);
    while (digitalRead(3) == LOW);  delay(200);
}

// ==================== Setting Functions ====================
void MyRP_Pico2::setMotorFreq(String type) {
    DC_Motors = (type == "DC_Motors" || type == "dc");
    analogWriteFreq(DC_Motors ? 1000 : 20000);
    Serial.println(DC_Motors ? "Motor: DC mode (1000Hz)" : "Motor: High freq (20kHz)");
}

void MyRP_Pico2::setSpeed(int sl, int sr) {
    slmotor = constrain(sl, 0, 100);
    srmotor = constrain(sr, 0, 100);
}

void MyRP_Pico2::setTurnCenter(int l, int r) {
    clml = l; clmr = r;
}

void MyRP_Pico2::setTurnFront(int l, int r) {
    flml = l; flmr = r;
}

void MyRP_Pico2::setKD(float kdf, float kdb) {
    kd_f = kdf;
    kd_b = kdb;
}

void MyRP_Pico2::setWheelRadius(float r) {
    if (r > 0) redius_wheel = r;
}

void MyRP_Pico2::setBrake(int ff, int fc, int bf, int bc) {
    break_ff = ff; break_fc = fc;
    break_bf = bf; break_bc = bc;
}


void MyRP_Pico2::setSlowPID(float kp, float ki) {
    kp_slow = kp;
    ki_slow = ki;
    Serial.printf("Slow PID set → Kp: %.4f, Ki: %.6f\n", kp_slow, ki_slow);
}

// ================================================
// fline() - เวอร์ชันสมบูรณ์สำหรับ Class MyRP_Pico2
// ================================================
void MyRP_Pico2::sp_factor_cm_s_forFW(float SP_FACTOR_CM_SS)
     {
        SPEED_FACTOR_CM_Sfw = SP_FACTOR_CM_SS;
     }
void MyRP_Pico2::fline(int spl, int spr, float kp, String target, 
                       char nfc, char splr, int power, String sensor, int endt)
{
    _fw = true;
    float distance = target.toFloat();

    char sensors[4];
    sensor.toCharArray(sensors, 4);
    int sensor_f = atoi(&sensors[1]);

    // ค่าคงที่การควบคุมความเร็ว
    float cruise = min((float)min(spl, spr), CRUISE_PWM_VAL);
    float minp   = max(MIN_PWM_VAL, cruise * 0.3f);

    float I_local = 0.0f;
    float prevE   = 0.0f;

    if (kp == 0.0f) kp_slow = ki_slow = 0.0f;
    pid_error = (kp < 6.5f);

    const float Kd_min = 0.015f;
    const float Kd_max = 0.070f;

    if (spl == 0) goto _line_f;

    // =============== ส่วนหลัก: Accel / Cruise / Decel + PID ===============
    {
        unsigned long prevT = millis();
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

            // คำนวณ PID Error
            if (kp <= 0.45f) {
                if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) &&
                    read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5)) {
                    errors = 0;
                }
                else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) {
                    errors = 10;
                }
                else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) {
                    errors = -10;
                }
                else {
                    errors = error_AA();
                }
            } else {
                errors = error_AA();
            }

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
                if (skipAccel || endt == 0) {
                    base_pwm = cruise;
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

            dist += ((prevBase + base_pwm) / 5.0f) * SPEED_FACTOR_CM_Sfw * dt;
            prevBase = base_pwm;

            // ตรวจจับจุดหยุด
            bool should_stop = false;
             if (isDigit(mode[0])) {
                if (dist >= distance * 0.85f) should_stop = true;
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
            else if (mode == "b07" || mode == "b70" || mode == "BRL" || mode == "brl" ||
                     mode == "BLR" || mode == "blr") {
                if (read_sensorB(0) < md_sensorB(0) && read_sensorB(7) < md_sensorB(7)) should_stop = true;
            }
            else if (mode == "CL" || mode == "cl" || mode == "27") {
                if (analogRead(27) < (sensorMinC[1] + md_sensorC(1)) / 2.0f) should_stop = true;
            }
            else if (mode == "CR" || mode == "cr" || mode == "26") {
                if (analogRead(26) < (sensorMinC[0] + md_sensorC(0)) / 2.0f) should_stop = true;
            }

            if (should_stop) break;

            // ส่ง PWM
            Motor(constrain((int)(base_pwm - PID), -100, 100),
                  constrain((int)(base_pwm + PID), -100, 100));

            delayMicroseconds(55);
        }

        Motor(0, 0);        // หยุดมอเตอร์เมื่อจบ
    }

_line_f:
    delay(10);

    // รีเซ็ตสถานะ crossedLine
    if (splr != 'p') {
        crossedLine = false;
    }

    // ==================== Lambda PID Loop (ใช้ซ้ำในหลายโหมด) ====================
    auto pidLoop = [&](float kpu, float ki, float kd_base, int bl, int br, int dly, auto&& stopCond) {
        float localI = 0.0f;
        float localPrevE = 0.0f;

        while (true) {
            if (kp <= 0.65f) {
                if (read_sensorA(2) > md_sensorA(2) && read_sensorA(3) > md_sensorA(3) &&
                    read_sensorA(4) > md_sensorA(4) && read_sensorA(5) > md_sensorA(5)) errors = 0;
                else if (read_sensorA(5) < md_sensorA(5) && read_sensorA(6) < md_sensorA(6) && read_sensorA(7) < md_sensorA(7)) errors = 10;
                else if (read_sensorA(2) < md_sensorA(2) && read_sensorA(1) < md_sensorA(1) && read_sensorA(0) < md_sensorA(0)) errors = -10;
                else errors = error_A();
            } else {
                errors = error_A();
            }

            updateBattery();
            localI += errors * ki;

            float speed_factor = (float)abs(bl) / 85.0f;
            float current_Kd = kd_base * (1.0f + 1.2f * speed_factor);

            float PID = kpu * errors + 0.000001f * localI + current_Kd * (errors - localPrevE);
            localPrevE = errors;

            Motor(bl - PID, br + PID);
            delayMicroseconds(dly);

            if (stopCond()) break;
        }
    };

    // =============== โหมด nfc และ splr ===============
    if (nfc == 'n') {
        if (splr == 'p') {
            crossedLine = true;
            pidLoop(kp_slow, 0.00005f, 0.025f, slmotor, srmotor, 80, [&](){
                return (read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1)) ||
                       (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6));
            });
        }
        else if (splr == 's') {
            if (endt > 0) {
                Motor(-(spl/3), -(spr/30)); delay(endt);
                Motor(-1, -1); delay(10);
            } else {
                Motor(0, 0);
                crossedLine = true;
            }
            goto _entN_f;
        }
    }
    else if (nfc == 'f') {
        if (distance > 0.0f) {
            float k = (kp > 0.65f) ? kp : kp_slow;
            pidLoop(k, 0.00005f, 0.025f, slmotor, srmotor, 80, [&](){
                return (read_sensorA(0) < md_sensorA(0) && read_sensorA(1) < md_sensorA(1)) ||
                       (read_sensorA(7) < md_sensorA(7) && read_sensorA(6) < md_sensorA(6));
            });
        }

        if (splr == 'p') {
            crossedLine = true;
            while (true) {
                Motor(spl, spr);
                delayMicroseconds(80);
                if (read_sensorA(0) > md_sensorA(0) && read_sensorA(7) > md_sensorA(7)) break;
            }
            if (endt > 0) {
                Motor(-spl, -spr); delay(endt);
                Motor(-1, -1); delay(10);
            } else {
                Motor(0, 0);
            }
        }
        else if (splr == 's') {
            Motor(-spl, -spr); delay(endt);
            Motor(0, 0); delay(2);
        }
    }
    else if (nfc == 'c') {
        auto stopC = [&](){
            return analogRead(26) < (sensorMinC[0] + md_sensorC(0))/2.0f ||
                   analogRead(27) < (sensorMinC[1] + md_sensorC(1))/2.0f;
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

    // =============== โหมดเลี้ยวซ้าย-ขวา ===============
    if (splr == 'l' || splr == 'r') {
        bool isLeft = (splr == 'l');

        if (nfc == 'f') {
            while (true) {
                delayMicroseconds(80);
                Motor(slmotor, srmotor);
                if (read_sensorA(7) > md_sensorA(7) && read_sensorA(0) > md_sensorA(0)) {
                    delay(delay_f);
                    break;
                }
            }
            Motor(-slmotor, -srmotor);
            delay(break_ff);

            if (isLeft) {
                for (int i = 0; i <= sensor_f; i++) {
                    do {
                        Motor((flml*power)/100, (flmr*power)/100);
                        delayMicroseconds(80);
                    } while (read_sensorA(i) > md_sensorA(i) - 50);
                }
            } else {
                for (int i = 7; i >= sensor_f; i--) {
                    do {
                        Motor((frml*power)/100, (frmr*power)/100);
                        delayMicroseconds(80);
                    } while (read_sensorA(i) > md_sensorA(i) - 50);
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
            if (isLeft) turn_speed_fl(); 
            else turn_speed_fr();
        } else {
            int ml = isLeft ? clml : crml;
            int mr = isLeft ? clmr : crmr;
            Motor(-(ml*power)/100, -(mr*power)/100);
            delay(endt);
            Motor(-1, -1);
            delay(10);
        }
    }

    setpoint_fw = false;

_entN_f:
    delay(5);
}

// ================================================
// bline() - ถอยหลังตามเส้น (เวอร์ชันสมบูรณ์สำหรับ Class)
// ================================================
void MyRP_Pico2::sp_factor_cm_s_forBW(float SP_FACTOR_CM_SS)
     {
        SPEED_FACTOR_CM_Sbw = SP_FACTOR_CM_SS;
     }
void MyRP_Pico2::bline(int spl, int spr, float kp, String target, 
                       char nfc, char splr, int power, String sensor, int endt)
{
    _fw = false;                    // สำคัญ: กำลังถอยหลัง

    float distance = target.toFloat();

    char sensors[4];
    sensor.toCharArray(sensors, 4);
    int sensor_f = atoi(&sensors[1]);

    // ค่าคงที่การควบคุมความเร็ว
    float cruise = min((float)min(spl, spr), CRUISE_PWM_VAL);
    float minp   = max(MIN_PWM_VAL, cruise * 0.3f);

    float I_local = 0.0f;
    float prevE   = 0.0f;

    if (kp == 0.0f) kp_slow = ki_slow = 0.0f;
    pid_error = (kp < 6.5f);

    const float Kd_min = 0.015f;
    const float Kd_max = 0.070f;

    if (spl == 0) goto _line_b;

    // =============== ส่วนหลัก: Accel / Cruise / Decel + PID ===============
    {
        unsigned long prevT = millis();
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

            // PID Error (ใช้ Sensor B เป็นหลัก)
            if (kp <= 0.45f) {
                if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) &&
                    read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5)) {
                    errors = 0;
                }
                else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7)) {
                    errors = 10;
                }
                else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0)) {
                    errors = -10;
                }
                else {
                    errors = error_BB();
                }
            } else {
                errors = error_BB();
            }

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
                if (skipAccel || endt == 0) {
                    base_pwm = cruise;
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

            dist += ((prevBase + base_pwm) / 5.0f) * SPEED_FACTOR_CM_Sbw * dt;
            prevBase = base_pwm;

            // ตรวจจับจุดหยุด
            bool should_stop = false;
            if (isDigit(mode[0])) {
                if (dist >= distance * 0.85f) should_stop = true;
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
            else if (mode == "b07" || mode == "b70" || mode == "BRL" || mode == "brl" ||
                     mode == "BLR" || mode == "blr") {
                if (read_sensorB(0) < md_sensorB(0) && read_sensorB(7) < md_sensorB(7)) should_stop = true;
            }
            else if (mode == "CL" || mode == "cl" || mode == "27") {
                if (analogRead(27) < (sensorMinC[1] + md_sensorC(1)) / 2.0f) should_stop = true;
            }
            else if (mode == "CR" || mode == "cr" || mode == "26") {
                if (analogRead(26) < (sensorMinC[0] + md_sensorC(0)) / 2.0f) should_stop = true;
            }

            if (should_stop) break;

            // ส่ง PWM ถอยหลัง (สำคัญ: ใส่เครื่องหมายลบ)
            Motor(constrain((int)(-(base_pwm + PID)), -100, 100),
                  constrain((int)(-(base_pwm - PID)), -100, 100));

            delayMicroseconds(60);
        }

        Motor(0, 0);        // หยุดมอเตอร์เมื่อจบ
    }

_line_b:
    delay(10);

    // รีเซ็ตสถานะ crossedLine
    if (splr != 'p') {
        crossedLine = false;
    }

    // ==================== Lambda PID Loop สำหรับ bline ====================
    auto pidLoop = [&](float kpu, float ki, float kd_base, int bl, int br, int dly, auto&& stopCond) {
        float localI = 0.0f;
        float localPrevE = 0.0f;

        while (true) {
            if (kp <= 0.65f) {
                if (read_sensorB(2) > md_sensorB(2) && read_sensorB(3) > md_sensorB(3) &&
                    read_sensorB(4) > md_sensorB(4) && read_sensorB(5) > md_sensorB(5)) errors = 0;
                else if (read_sensorB(5) < md_sensorB(5) && read_sensorB(6) < md_sensorB(6) && read_sensorB(7) < md_sensorB(7)) errors = 10;
                else if (read_sensorB(2) < md_sensorB(2) && read_sensorB(1) < md_sensorB(1) && read_sensorB(0) < md_sensorB(0)) errors = -10;
                else errors = error_B();
            } else {
                errors = error_B();
            }

            updateBattery();
            localI += errors * ki;

            float speed_factor = (float)abs(bl) / 85.0f;
            float current_Kd = kd_base * (1.0f + 1.2f * speed_factor);

            float PID = kpu * errors + 0.000001f * localI + current_Kd * (errors - localPrevE);
            localPrevE = errors;

            // สำหรับถอยหลัง ต้องใส่เครื่องหมายลบทั้งสองข้าง
            Motor(-(bl + PID), -(br - PID));
            delayMicroseconds(dly);

            if (stopCond()) break;
        }
    };

    // =============== โหมด nfc ===============
    if (nfc == 'n') {
        if (splr == 'p') {
            crossedLine = true;
            pidLoop(kp_slow, 0.00005f, 0.025f, slmotor, srmotor, 80, [&](){
                return (read_sensorB(0) < md_sensorB(0) && read_sensorB(1) < md_sensorB(1)) ||
                       (read_sensorB(7) < md_sensorB(7) && read_sensorB(6) < md_sensorB(6));
            });
        }
        else if (splr == 's') {
            if (endt > 0) {
                Motor(3, 3); delay(endt); 
                Motor(1, 1); delay(10);     // เบรกเบาๆ
            } else {
                Motor(0, 0);
                crossedLine = true;
            }
            goto _entN_b;
        }
    }
    else if (nfc == 'f') {
        if (distance > 0.0f) {
            float k = (kp > 0.65f) ? kp : kp_slow;
            pidLoop(k, 0.00005f, 0.025f, slmotor, srmotor, 80, [&](){
                return (read_sensorB(0) < md_sensorB(0) && read_sensorB(1) < md_sensorB(1)) ||
                       (read_sensorB(7) < md_sensorB(7) && read_sensorB(6) < md_sensorB(6));
            });
        }

        if (splr == 'p') {
            crossedLine = true;
            while (true) {
                Motor(-spl, -spr);
                delayMicroseconds(80);
                if (read_sensorB(0) > md_sensorB(0) && read_sensorB(7) > md_sensorB(7)) break;
            }
            if (endt > 0) {
                Motor(spl, spr); delay(endt);
                Motor(0, 0); delay(10);
            } else {
                Motor(0, 0);
            }
        }
        else if (splr == 's') {
            Motor(spl, spr); delay(endt);
            Motor(0, 0); delay(2);
        }
    }
    else if (nfc == 'c') {
        auto stopC = [&](){
            return analogRead(26) < (sensorMinC[0] + md_sensorC(0))/2.0f ||
                   analogRead(27) < (sensorMinC[1] + md_sensorC(1))/2.0f;
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

    // =============== โหมดเลี้ยวซ้าย-ขวา (สำหรับถอยหลัง) ===============
    if (splr == 'l' || splr == 'r') {
        bool isLeft = (splr == 'l');

        if (nfc == 'f') {
            while (true) {
                delayMicroseconds(80);
                Motor(-slmotor, -srmotor);          // ถอยหลัง
                if (read_sensorB(7) > md_sensorB(7) && read_sensorB(0) > md_sensorB(0)) {
                    delay(delay_f); break;
                }
            }
            Motor(slmotor, srmotor); delay(break_ff);

            if (isLeft) {
                for (int i = 0; i <= sensor_f; i++) {
                    do {
                        Motor(-((flml*power)/100), -((flmr*power)/100));
                        delayMicroseconds(80);
                    } while (read_sensorB(i) > md_sensorB(i) - 50);
                }
            } else {
                for (int i = 7; i >= sensor_f; i--) {
                    do {
                        Motor(-((frml*power)/100), -((frmr*power)/100));
                        delayMicroseconds(80);
                    } while (read_sensorB(i) > md_sensorB(i) - 50);
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
            if (isLeft) turn_speed_fl(); 
            else turn_speed_fr();
        } else {
            int ml = isLeft ? clml : crml;
            int mr = isLeft ? clmr : crmr;
            Motor(-(ml*power)/100, -(mr*power)/100);
            delay(endt);
            Motor(-1, -1);
            delay(10);
        }
    }

_entN_b:
    delay(5);
}

// ================================================
// Gyro & Rotate Functions (เวอร์ชันสมบูรณ์สำหรับ Class)
// ================================================

// ----------------------- place_left_in -----------------------
void MyRP_Pico2::place_left_in(int mr, int degree, int offset)
{
    my.resetAngles();

    float initialDegree = 0.0f;
    for (int i = 0; i < 10; i++) {
        initialDegree += my.gyro('z');
        delay(6);
    }
    initialDegree /= 10.0f;
    float targetDegree = initialDegree - (float)degree;

    if (degree == 0) {
        Motor(0, 0);
        return;
    }

    int targetDeg = abs(degree);
    int speedDiff = abs(1 - mr);
    if (speedDiff < 10) {
        Motor(0, 0);
        return;
    }

    long moveTime = ((long)targetDeg * offset * 11L) / speedDiff;

    Motor(-1, mr);
    delay(moveTime / 5);

    if (abs(my.gyro('z')) > 0 && abs(my.gyro('z')) < 360)
    {
        Serial.println("[rotate] → ใช้โหมด Gyro");

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
                integral = constrain(integral, -280.0f, 280.0f);

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
        Serial.println("[rotate] Gyro ไม่ทำงาน → ใช้โหมดล้อสำรอง");
        beep(2600, 60);

        Motor(-1, mr);
        delay(moveTime);

        Motor(1, -mr);
        delay(offset);

        Motor(-1, -1);
        delay(10);
        Motor(0, 0);
    }

    Motor(-1, -1);
    delay(15);
}

// ----------------------- place_left_out -----------------------
void MyRP_Pico2::place_left_out(int mr, int degree, int offset)
{
    my.resetAngles();

    float initialDegree = 0.0f;
    for (int i = 0; i < 10; i++) {
        initialDegree += my.gyro('z');
        delay(6);
    }
    initialDegree /= 10.0f;
    float targetDegree = initialDegree + (float)degree;

    if (degree == 0 || mr == 1) {
        Motor(0, 0);
        return;
    }

    int targetDeg = abs(degree);
    int speedDiff = abs(1 - mr);
    if (speedDiff < 10) {
        Motor(0, 0);
        return;
    }

    long moveTime = ((long)targetDeg * offset * 11L) / speedDiff;

    Motor(1, -mr);
    delay(moveTime / 5);

    if (abs(my.gyro('z')) > 0 && abs(my.gyro('z')) < 360)
    {
        Serial.println("[rotate] → ใช้โหมด Gyro");

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
                integral = constrain(integral, -280.0f, 280.0f);

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
        Serial.println("[rotate] Gyro ไม่ทำงาน → ใช้โหมดล้อสำรอง");
        beep(2600, 60);

        Motor(1, -mr);
        delay(moveTime);

        Motor(-1, mr);
        delay(offset);

        Motor(-1, -1);
        delay(10);
        Motor(0, 0);
    }

    Motor(-1, -1);
    delay(15);
}

// ----------------------- place_right_in -----------------------
void MyRP_Pico2::place_right_in(int mr, int degree, int offset)
{
    my.resetAngles();

    float initialDegree = 0.0f;
    for (int i = 0; i < 10; i++) {
        initialDegree += my.gyro('z');
        delay(6);
    }
    initialDegree /= 10.0f;
    float targetDegree = initialDegree + (float)degree;

    if (degree == 0 || mr == 1) {
        Motor(0, 0);
        return;
    }

    int targetDeg = abs(degree);
    int speedDiff = abs(mr - 1);
    if (speedDiff < 10) {
        Motor(0, 0);
        return;
    }

    long moveTime = ((long)targetDeg * offset * 11L) / speedDiff;

    Motor(mr, -1);
    delay(moveTime / 5);

    if (abs(my.gyro('z')) > 0 && abs(my.gyro('z')) < 360)
    {
        Serial.println("[rotate] → ใช้โหมด Gyro");

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
                integral = constrain(integral, -280.0f, 280.0f);

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
        Serial.println("[rotate] Gyro ไม่ทำงาน → ใช้โหมดล้อสำรอง");
        beep(2600, 60);

        Motor(mr, -1);
        delay(moveTime);

        Motor(-mr, 1);
        delay(offset);

        Motor(-1, -1);
        delay(10);
        Motor(0, 0);
    }

    Motor(-1, -1);
    delay(15);
}

// ----------------------- place_right_out -----------------------
void MyRP_Pico2::place_right_out(int mr, int degree, int offset)
{
    my.resetAngles();

    float initialDegree = 0.0f;
    for (int i = 0; i < 10; i++) {
        initialDegree += my.gyro('z');
        delay(6);
    }
    initialDegree /= 10.0f;
    float targetDegree = initialDegree - (float)degree;

    if (degree == 0) {
        Motor(0, 0);
        return;
    }

    int targetDeg = abs(degree);
    int speedDiff = abs(mr - 1);
    if (speedDiff < 10) {
        Motor(0, 0);
        return;
    }

    long moveTime = ((long)targetDeg * offset * 11L) / speedDiff;

    Motor(-mr, 1);
    delay(moveTime / 5);

    if (abs(my.gyro('z')) > 0 && abs(my.gyro('z')) < 360)
    {
        Serial.println("[rotate] → ใช้โหมด Gyro");

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
                integral = constrain(integral, -280.0f, 280.0f);

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
        Serial.println("[rotate] Gyro ไม่ทำงาน → ใช้โหมดล้อสำรอง");
        beep(2600, 60);

        Motor(-mr, 1);
        delay(moveTime);

        Motor(mr, -1);
        delay(offset);

        Motor(-1, -1);
        delay(10);
        Motor(0, 0);
    }

    Motor(-1, -1);
    delay(15);
}


// ================================================
// Rotate & Gyro Functions (เวอร์ชันสมบูรณ์สำหรับ Class)
// ================================================

// ตรวจสอบสถานะ Gyro
bool MyRP_Pico2::gyroOK()
{
    Serial.println("=== ตรวจสอบ Gyro ===");

    Wire.beginTransmission(0x68);
    byte error68 = Wire.endTransmission();
    Wire.beginTransmission(0x69);
    byte error69 = Wire.endTransmission();

    bool hasGyro = (error68 == 0) || (error69 == 0);

    if (hasGyro) {
        Serial.println("พบ Gyro ที่ address 0x68 หรือ 0x69");
        
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

// ----------------------- rotate() -----------------------
void MyRP_Pico2::rotate(int ml, int mr, int degree, int offset)
{
    my.resetAngles();

    float initialDegree = 0.0f;
    for (int i = 0; i < 10; i++) {
        initialDegree += my.gyro('z');
        delay(6);
    }
    initialDegree /= 10.0f;
    float targetDegree = initialDegree + (float)degree;

    if (degree == 0 || ml == mr) {
        Motor(0, 0);
        return;
    }

    int targetDeg = abs(degree);
    int speedDiff = abs(ml - mr);
    if (speedDiff < 10) {
        Motor(0, 0);
        return;
    }

    long moveTime = ((long)targetDeg * offset * 11L) / speedDiff;

    Motor(ml, mr);
    delay(moveTime / 5);

    if (abs(my.gyro('z')) > 0 && abs(my.gyro('z')) < 360)
    {
        Serial.println("[rotate] → ใช้โหมด Gyro");

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
                integral = constrain(integral, -280.0f, 280.0f);

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
        Serial.println("[rotate] Gyro ไม่ทำงาน → ใช้โหมดล้อสำรอง");
        beep(2600, 60);

        Motor(ml, mr);
        delay(moveTime);

        Motor(-ml, -mr);
        delay(offset);

        Motor(-1, -1);
        delay(10);
        Motor(0, 0);
    }

    Motor(-1, -1);
    delay(15);
}



// ----------------------- setrobot_fline & setrobot_bline -----------------------
void MyRP_Pico2::setrobot_fline(int num)
{
    for (int i = 0; i < num; i++) {
        while (true) {
            delay(5);
            if (read_sensorA(0) < (md_sensorA(0) + sensorMinA[0]) / 2 && 
                read_sensorA(7) > (md_sensorA(7) + sensorMinA[7]) / 2) {
                Motor(-2, 10);
            }
            else if (read_sensorA(0) > (md_sensorA(0) + sensorMinA[0]) / 2 && 
                     read_sensorA(7) < (md_sensorA(7) + sensorMinA[7]) / 2) {
                Motor(10, -2);
            }
            else if (read_sensorA(0) > (md_sensorA(0) + sensorMinA[0]) / 2 && 
                     read_sensorA(7) > (md_sensorA(7) + sensorMinA[7]) / 2) {
                Motor(10, 10);
            }
            else {
                Motor(-1, -1);
                break;
            }
        }

        if (num > 1) {
            Motor(-12, -12);
            delay(50);
            while (true) {
                if (read_sensorA(0) > (md_sensorA(0) + sensorMinA[0]) / 2 && 
                    read_sensorA(7) > (md_sensorA(7) + sensorMinA[7]) / 2) {
                    break;
                }
                Motor(-12, -12);
                delay(5);
            }
            Motor(1, 1);
        }
    }
}

void MyRP_Pico2::setrobot_bline(int num)
{
    for (int i = 0; i < num; i++) {
        while (true) {
            delay(5);
            if (read_sensorB(0) < (md_sensorB(0) + sensorMinB[0]) / 2 && 
                read_sensorB(7) > (md_sensorB(7) + sensorMinB[7]) / 2) {
                Motor(-10, 2);
            }
            else if (read_sensorB(0) > (md_sensorB(0) + sensorMinB[0]) / 2 && 
                     read_sensorB(7) < (md_sensorB(7) + sensorMinB[7]) / 2) {
                Motor(2, -10);
            }
            else if (read_sensorB(0) > (md_sensorB(0) + sensorMinB[0]) / 2 && 
                     read_sensorB(7) > (md_sensorB(7) + sensorMinB[7]) / 2) {
                Motor(-10, -10);
            }
            else {
                Motor(1, 1);
                break;
            }
        }

        if (num > 1) {
            Motor(12, 12);
            delay(50);
            while (true) {
                if (read_sensorB(0) > (md_sensorB(0) + sensorMinB[0]) / 2 && 
                    read_sensorB(7) > (md_sensorB(7) + sensorMinB[7]) / 2) {
                    break;
                }
                Motor(12, 12);
                delay(5);
            }
            Motor(1, 1);
        }
    }
}

// ==================== Turn Speed Functions ====================
void MyRP_Pico2::turn_speed_fl() {
    for (int t = 0; t < ldelaymotor; t++) {
        errors = error_A();
        updateBattery();
        P = errors;
        I += errors * (ramp_delay / 1000.0f);
        D = errors - previous_error;
        previous_error = errors;

        PID_output = (0.45f * P) + (0.00001f * I) + (kd_f * D);
        Motor(llmotor + PID_output, lrmotor - PID_output);
        delay(ramp_delay);
    }
}

void MyRP_Pico2::turn_speed_fr() {
    for (int t = 0; t < rdelaymotor; t++) {
        errors = error_A();
        updateBattery();
        P = errors;
        I += errors * (ramp_delay / 1000.0f);
        D = errors - previous_error;
        previous_error = errors;

        PID_output = (0.45f * P) + (0.00001f * I) + (kd_f * D);
        Motor(rlmotor + PID_output, rrmotor - PID_output);
        delay(ramp_delay);
    }
}

void MyRP_Pico2::bturn_speed_fl() {
    for (int t = 0; t < ldelaymotor; t++) {
        errors = error_A();
        updateBattery();
        P = errors;
        I += errors * (ramp_delay / 1000.0f);
        D = errors - previous_error;
        previous_error = errors;

        PID_output = (0.45f * P) + (0.00001f * I) + (kd_b * D);
        Motor(-(llmotor + PID_output), -((lrmotor - 10) - PID_output));
        delay(ramp_delay);
    }
    delay(2);
}

void MyRP_Pico2::bturn_speed_fr() {
    for (int t = 0; t < rdelaymotor; t++) {
        errors = error_A();
        updateBattery();
        P = errors;
        I += errors * (ramp_delay / 1000.0f);
        D = errors - previous_error;
        previous_error = errors;

        PID_output = (0.45f * P) + (0.00001f * I) + (kd_b * D);
        Motor(-(rlmotor + PID_output), -((rrmotor - 10) - PID_output));
        delay(ramp_delay);
    }
    delay(2);
}

// ==================== Position & Error Functions ====================

void MyRP_Pico2::sensor_position(int position)
    {
        _setpoint = position;
    }

int MyRP_Pico2::position_A() {
    int Minsensor_values_A[] = { sensorMinA[1], sensorMinA[2], sensorMinA[3], sensorMinA[4], sensorMinA[5], sensorMinA[6] };
    int Maxsensor_values_A[] = { sensorMaxA[1], sensorMaxA[2], sensorMaxA[3], sensorMaxA[4], sensorMaxA[5], sensorMaxA[6] };

    bool onLine = false;
    long avg = 0;
    long sum = 0;

    for (uint8_t i = 0; i < 6; i++) {
        long value = map(read_sensorA(sensor_pin_A[i]), Minsensor_values_A[i], Maxsensor_values_A[i], 1000, 0);
        if (value > 200) onLine = true;
        if (value > 50) {
            avg += value * (i * 1000L);
            sum += value;
        }
    }

    if (!onLine) {
        return (_lastPosition < (numSensor - 1) * 1000 / 2) ? 0 : 5000;
    }

    _lastPosition = (sum == 0) ? _lastPosition : avg / sum;
    return _lastPosition;
}

int MyRP_Pico2::position_A_none() {
    int Minsensor_values_A[] = { sensorMinA[1], sensorMinA[2], sensorMinA[3], sensorMinA[4], sensorMinA[5], sensorMinA[6] };
    int Maxsensor_values_A[] = { sensorMaxA[1], sensorMaxA[2], sensorMaxA[3], sensorMaxA[4], sensorMaxA[5], sensorMaxA[6] };

    bool onLine = false;
    long avg = 0;
    long sum = 0;

    for (uint8_t i = 0; i < 6; i++) {
        long value = map(read_sensorA(sensor_pin_A[i]), Minsensor_values_A[i], Maxsensor_values_A[i], 1000, 0);
        if (value > 200) onLine = true;
        if (value > 50) {
            avg += value * (i * 1000L);
            sum += value;
        }
    }

    if (!onLine) {
        return 2500;
    }

    _lastPosition = (sum == 0) ? _lastPosition : avg / sum;
    return _lastPosition;
}

float MyRP_Pico2::error_A() {
    if (pid_error) {
        present_position = position_A_none() / ((numSensor - 1) * 10.0f);
    } else {
        present_position = position_A() / ((numSensor - 1) * 10.0f);
    }
    setpoint = _setpoint;
    errors = setpoint - present_position;
    return errors;
}

float MyRP_Pico2::error_AA() {
    present_position = position_A() / ((numSensor - 1) * 10.0f);
    setpoint = _setpoint;
    errors = setpoint - present_position;
    return errors;
}

float MyRP_Pico2::error_AN() {
    present_position = position_A_none() / ((numSensor - 1) * 10.0f);
    setpoint = _setpoint;
    errors = setpoint - present_position;
    return errors;
}

// ==================== Position & Error Functions for Sensor B ====================

int MyRP_Pico2::position_B()
{
    int Minsensor_values_B[] = { sensorMinB[1], sensorMinB[2], sensorMinB[3], sensorMinB[4], sensorMinB[5], sensorMinB[6] };
    int Maxsensor_values_B[] = { sensorMaxB[1], sensorMaxB[2], sensorMaxB[3], sensorMaxB[4], sensorMaxB[5], sensorMaxB[6] };

    bool onLine = false;
    long avg = 0;
    long sum = 0;

    for (uint8_t i = 0; i < 6; i++)
    {
        long value = map(read_sensorB(sensor_pin_B[i]), 
                         Minsensor_values_B[i], 
                         Maxsensor_values_B[i], 
                         1000, 0);

        if (value > 200) onLine = true;
        if (value > 50)
        {
            avg += (long)value * (i * 1000L);
            sum += value;
        }
    }

    if (!onLine)
    {
        if (_lastPosition < (numSensor - 1) * 1000 / 2)
            return 0;
        else
            return 5000;
    }

    _lastPosition = (sum == 0) ? _lastPosition : avg / sum;
    return _lastPosition;
}

int MyRP_Pico2::position_B_none()
{
    int Minsensor_values_B[] = { sensorMinB[1], sensorMinB[2], sensorMinB[3], sensorMinB[4], sensorMinB[5], sensorMinB[6] };
    int Maxsensor_values_B[] = { sensorMaxB[1], sensorMaxB[2], sensorMaxB[3], sensorMaxB[4], sensorMaxB[5], sensorMaxB[6] };

    bool onLine = false;
    long avg = 0;
    long sum = 0;

    for (uint8_t i = 0; i < 6; i++)
    {
        long value = map(read_sensorB(sensor_pin_B[i]), 
                         Minsensor_values_B[i], 
                         Maxsensor_values_B[i], 
                         1000, 0);

        if (value > 200) onLine = true;
        if (value > 50)
        {
            avg += (long)value * (i * 1000L);
            sum += value;
        }
    }

    if (!onLine)
    {
        return 2500;        // คืนค่ากลางเมื่อหลุดเส้น
    }

    _lastPosition = (sum == 0) ? _lastPosition : avg / sum;
    return _lastPosition;
}

float MyRP_Pico2::error_B()
{
    if (pid_error == true)
    {
        present_position = position_B_none() / ((numSensor - 1) * 10.0f);
    }
    else
    {
        present_position = position_B() / ((numSensor - 1) * 10.0f);
    }

    setpoint = _setpoint;
    errors = setpoint - present_position;
    return errors;
}

float MyRP_Pico2::error_BB()
{
    present_position = position_B() / ((numSensor - 1) * 10.0f);
    setpoint = _setpoint;
    errors = setpoint - present_position;
    return errors;
}

float MyRP_Pico2::error_BN()
{
    present_position = position_B_none() / ((numSensor - 1) * 10.0f);
    setpoint = _setpoint;
    errors = setpoint - present_position;
    return errors;
}

// ==================== Wrapper Functions สำหรับ setup() ====================

void MyRP_Pico2::set_Freq(String type) {
    if (type == "Coreless_Motors" || type == "coreless") {
        DC_Motors = false;
        analogWriteFreq(20000);
        Serial.println("Motor: Coreless_Motors (20kHz)");
    } else {
        DC_Motors = true;
        analogWriteFreq(1000);
        Serial.println("Motor: DC_Motors (1kHz)");
    }
}

void MyRP_Pico2::distance_scale_fw(float scale) {
    speed_scale_fw = scale;
}

void MyRP_Pico2::distance_scale_bw(float scale) {
    speed_scale_bw = scale;
}

void MyRP_Pico2::set_slow_motor(int sl, int sr) {
    slmotor = sl;
    srmotor = sr;
}

void MyRP_Pico2::set_turn_center_l(int l, int r) {
    clml = l; clmr = r;
}

void MyRP_Pico2::set_turn_center_r(int l, int r) {
    crml = l; crmr = r;
}

void MyRP_Pico2::set_turn_front_l(int l, int r) {
    flml = l; flmr = r;
}

void MyRP_Pico2::set_turn_front_r(int l, int r) {
    frml = l; frmr = r;
}

void MyRP_Pico2::set_brake_fc(int ff, int fc) {
    break_ff = ff; break_fc = fc;
}

void MyRP_Pico2::set_brake_bc(int bf, int bc) {
    break_bf = bf; break_bc = bc;
}

void MyRP_Pico2::set_delay_f(int delayTime) {
    delay_f = constrain(delayTime, 0, 100);
}

void MyRP_Pico2::fw_gyros(int spl, int spr, float kp,  float distance, int offset) 
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

void MyRP_Pico2::fw_gyro(int spl, int spr, float kp, float distance_cm, int offset)
{
    if (distance_cm <= 0) {
        Motor(0, 0);
        return;
    }

    int base_speed = min(abs(spl), abs(spr));
    bool is_forward = (spl >= 0 && spr >= 0);

    float kpG = kp;
    float kdG = 0.22;

    float previous_error = 0.0;
    float traveled_distance = 0.0;
    unsigned long last_time = millis();

    // ====================== ค่าที่สามารถปรับได้ ======================
    const float ACCEL_DISTANCE_CM = 20.0;
    const float DECEL_DISTANCE_CM = 25.0;
    const float MIN_SPEED = 10.0;

    // ค่า speed_scale ที่คุณต้องการปรับได้ (ค่าดีฟอลต์ = 0.99)
    float speed_scale = 0.99;        // ← คุณสามารถปรับตรงนี้ได้

    // ตัดสินใจว่าใช้ Ramp หรือไม่
    bool enableRamp = (distance_cm >= 30.0);

    // ถ้าระยะสั้นมาก (< 30) ให้ปรับ speed_scale ได้ง่ายขึ้น
    if (!enableRamp) {
        speed_scale = 1.7;   // คุณสามารถเปลี่ยนเป็น 0.95, 0.98, 1.0 ได้ตามต้องการ
    }

    my.resetAngles();
    float yaw_offset = my.gyro('z');

    while (true)
    {
        updateBattery();
        float yaw = my.gyro('z') - yaw_offset;
        float error = yaw;
        if (error > 180) error -= 360;
        else if (error < -180) error += 360;

        float derivative = error - previous_error;
        float corr = (kpG * error) + (kdG * derivative);

        // คำนวณระยะทาง
        unsigned long current_time = millis();
        float delta_time = (current_time - last_time) / 1000.0;
        traveled_distance += (base_speed * speed_scale) * delta_time;
        last_time = current_time;

        float remaining_cm = distance_cm - traveled_distance;

        if (remaining_cm <= 0.7f) break;

        // ====================== คำนวณ target_speed ======================
        float target_speed = base_speed;

        if (enableRamp)
        {
            if (traveled_distance < ACCEL_DISTANCE_CM) {
                // เร่งช่วงแรก
                target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
            }
            else if (remaining_cm < DECEL_DISTANCE_CM) {
                // ชะลอช่วงสุดท้าย
                target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);
            }
        }
        // ถ้า enableRamp = false → ใช้ความเร็วคงที่ตลอดทาง

        // ====================== คำนวณความเร็วซ้าย-ขวา ======================
        int leftSpeed, rightSpeed;
        if (is_forward) {
            leftSpeed  = constrain(target_speed - corr, 0, spl);
            rightSpeed = constrain(target_speed + corr, 0, spr);
        } else {
            leftSpeed  = constrain(-(target_speed - corr), spl, 0);
            rightSpeed = constrain(-(target_speed + corr), spr, 0);
        }

        Motor(leftSpeed, rightSpeed);
        previous_error = error;

        delayMicroseconds(80);
    }

    // ====================== Soft Stop ======================
    if (offset > 0) {
        if (is_forward) {
            Motor(-3, -2); delay(offset);
        } else {
            Motor(2, 3); delay(offset);
        }
        Motor(-1, -1); delay(10);
    } 
    else {
        Motor(0, 0);
    }
    
    Motor(1, 1);      // Pulse เล็กน้อยเพื่อหยุดตรงขึ้น
    delay(5);
}

void MyRP_Pico2::bw_gyro(int spl, int spr, float kp, float distance_cm, int offset)
{
    if (distance_cm <= 0) {
        Motor(0, 0);
        return;
    }

    int base_speed = min(abs(spl), abs(spr));
    bool is_backward = true;   // ตรวจว่ากำลังถอยหลัง

    float kpG = kp;
    float kdG = 0.22;

    float previous_error = 0.0;
    float traveled_distance = 0.0;
    unsigned long last_time = millis();

    float speed_scale = 0.99;                    // ค่าเริ่มต้นสำหรับถอยหลัง

    const float ACCEL_DISTANCE_CM = 20.0;
    const float DECEL_DISTANCE_CM = 25.0;
    const float MIN_SPEED = 10.0;

    // ตัดสินใจว่าใช้ Ramp หรือไม่
    bool enableRamp = (distance_cm >= 30.0);

    // ถ้าระยะสั้นมาก (< 30) → ไม่ใช้ Ramp + ปรับ speed_scale
    if (!enableRamp) {
        speed_scale = 1.5;     // คุณสามารถปรับตรงนี้ได้ (แนะนำ 0.92 - 0.97)
    }

    my.resetAngles();
    float yaw_offset = my.gyro('z');

    while (true)
    {
        updateBattery();
        float yaw = my.gyro('z') - yaw_offset;
        float error = yaw;
        if (error > 180) error -= 360;
        else if (error < -180) error += 360;

        float derivative = error - previous_error;
        float corr = (kpG * error) + (kdG * derivative);

        // คำนวณระยะทาง
        unsigned long current_time = millis();
        float delta_time = (current_time - last_time) / 1000.0;
        traveled_distance += (base_speed * speed_scale) * delta_time;
        last_time = current_time;

        float remaining_cm = distance_cm - traveled_distance;

        if (remaining_cm <= 0.8f) break;

        // ====================== คำนวณความเร็ว ======================
        float target_speed = base_speed;

        if (enableRamp)   // ใช้ Ramp เฉพาะระยะยาว (>=30 cm)
        {
            if (traveled_distance < ACCEL_DISTANCE_CM) {
                target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
            }
            else if (remaining_cm < DECEL_DISTANCE_CM) {
                target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);
            }
        }
        // ถ้า !enableRamp → ใช้ความเร็วคงที่ตลอดทาง (base_speed)

        int leftSpeed, rightSpeed;

        // ====================== การถอยหลัง ======================
        if (is_backward) {
            leftSpeed  = constrain(-(target_speed + corr), -spl, 0);
            rightSpeed = constrain(-(target_speed - corr), -spr, 0);
        } else {
            leftSpeed  = constrain(target_speed - corr, 0, spl);
            rightSpeed = constrain(target_speed + corr, 0, spr);
        }

        Motor(leftSpeed, rightSpeed);
        previous_error = error;

        delayMicroseconds(80);
    }

    // ====================== Soft Stop ======================
    if (offset > 0) {
        if (is_backward) {
            // ถอยหลัง → เบรกด้วยการเดินหน้าเล็กน้อย
            Motor(4, 4); delay(offset);
        } else {
            // เดินหน้า → เบรกด้วยการถอยหลัง
            Motor(-4, -4); delay(offset);
        }
        Motor(0, 0); delay(10);
    } 
    else {
        Motor(0, 0);
    }

    Motor(1, 1);
    delay(5);
}

void MyRP_Pico2::bw_gyros(int spl, int spr, float kp, float distance, int offset) {
    int target_speed = min(spl, spr);
    float traveled_distance = 0;
    unsigned long last_time = millis();

    float speed_scale = 1.6f;
    my.resetAngles();
    float yaw_offset = -my.gyro('z');
    float _integral = 0;
    float _prevErr = 0;
    unsigned long prevT = millis();

    while (true) {
        updateBattery();
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001f;
        prevT = now;

        float yaw = my.gyro('z') - yaw_offset;
        float err = yaw;
        _integral += err * dt;
        float deriv = (err - _prevErr) / dt;
        _prevErr = err;

        float corr = kp * err + 0.0001f * _integral + 0.05f * deriv;

        int leftSpeed = constrain(spl + corr, -100, 100);
        int rightSpeed = constrain(spr - corr, -100, 100);

        Motor(-leftSpeed, -rightSpeed);

        if (distance > 0) {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;
            traveled_distance += (target_speed * speed_scale) * delta_time;
            last_time = current_time;
            if (traveled_distance >= distance) break;
        }
        delayMicroseconds(80);
    }

    if (offset > 0) {
        Motor(15, 15); delay(offset);
        Motor(1, 1); delay(10);
    } else {
        Motor(0, 0); delay(5);
    }
}


int MyRP_Pico2::ADC_i2c()
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
// ==================== sw() - เมนู Calibration + แสดงเซนเซอร์ ====================
void MyRP_Pico2::sw()
{
    tone(9, 2000, 100);   // โด
    delay(100);
    tone(9, 2400, 100);   // เร
    delay(100);
    tone(9, 3000, 160);   // มี
    delay(500);
    tone(9, 3000, 60);
    delay(80);
    tone(9, 3000, 60);
    delay(80);

    unsigned long pressStartTime = 0;
    bool isPressed = false;
    unsigned long lastBuzz = 0;
    // ส่วนตรวจสอบปุ่ม + แสดงค่าเซนเซอร์
    while (true)
    {
           bat.update_led();
            updateBattery();
            float voltage = getBatteryVoltage(); // เตือนเมื่อแบตต่ำกว่า 11.0V แต่ยังไม่ถึงขั้นวิกฤติ
            if (voltage <= 11.0 && voltage > 7.0) {
                unsigned long now = millis();
                // ส่งเสียงทุก 3 วินาที (3000 ms)
                if (now - lastBuzz >= 3000) {
                    Serial.println(voltage);              // แสดงแรงดันไฟ
                    // เสียงเตือนแบบ Sci-Fi / Cyber Tech (เท่และชัดเจน)
                    tone(9, 2800, 50);
                    delay(30);
                    tone(9, 3200, 50);
                    delay(30);
                    tone(9, 2400, 120);
                    delay(100);
                    tone(9, 1800, 80);
                    lastBuzz = now;        // อัพเดทเวลาล่าสุดที่ส่งเสียง
                }
              }

        // ปุ่มบน (ปุ่ม 3) → Calibrate A
        if (digitalRead(3) == LOW)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            tone(9, 3000, 300);
            calibrateA();        // แทน get_maxMinA()
            blink(5);
            delay(300);
            digitalWrite(LED_BUILTIN, LOW);
        }
        
        if(ADC_i2c() < 2500)
                {
                  digitalWrite(LED_BUILTIN, HIGH);
                        delay(200);
                        digitalWrite(LED_BUILTIN, LOW);
                        delay(200);
                        digitalWrite(LED_BUILTIN, HIGH);
                        tone(9, 3000, 100);
                  calibrateB();
                  digitalWrite(LED_BUILTIN, LOW);
                        delay(100);
                        digitalWrite(LED_BUILTIN, HIGH);
                        delay(100);
                        digitalWrite(LED_BUILTIN, LOW);
                        delay(100);
                        digitalWrite(LED_BUILTIN, HIGH);
                        delay(100);
                        digitalWrite(LED_BUILTIN, LOW);
                        delay(100);
                        while (digitalRead(2) == LOW);   // รอปล่อยปุ่ม
                        delay(200);
                  delay(200); // รอ 1 วินาที
                }
       
        // แสดงค่าซีเรียลทุก 100ms
        Serial.print("From A: ");
        for (int i = 0; i < 8; i++) {
            Serial.print(read_sensorA(i));
            Serial.print(" ");
        }
        Serial.print("   From B: ");
        for (int i = 0; i < 8; i++) {
            Serial.print(read_sensorB(i));
            Serial.print(" ");
        }
        Serial.println();

        // ตรวจจับกดค้างปุ่ม 2 นาน 3 วินาที → Calibrate C

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
                      { 
                        digitalWrite(LED_BUILTIN, HIGH);
                        delay(200);
                        digitalWrite(LED_BUILTIN, LOW);
                        delay(200);
                        digitalWrite(LED_BUILTIN, HIGH);
                        tone(9, 3000, 100);
                        tone(9, 3000, 200);
                        calibrateC();
                        digitalWrite(LED_BUILTIN, LOW);
                        delay(100);
                        digitalWrite(LED_BUILTIN, HIGH);
                        delay(100);
                        digitalWrite(LED_BUILTIN, LOW);
                        delay(100);
                        digitalWrite(LED_BUILTIN, HIGH);
                        delay(100);
                        digitalWrite(LED_BUILTIN, LOW);
                        delay(100);
                        while (digitalRead(2) == LOW);   // รอปล่อยปุ่ม
                        delay(200);
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




    // โหลดค่า calibration จาก EEPROM หลังออกจากเมนู
    loadCalibration();

    tone(9, 3000, 400);
    delay(500);
}
