/*
PiperPico2_Basic — คู่มือฟังก์ชันและตัวอย่างการใช้งาน

โครงสร้าง 3 แท็บ
  PiperPico2_Basic : setup(), loop() และโปรแกรมหุ่นยนต์
  README          : คำอธิบายและตัวอย่างคำสั่ง (เป็นคอมเมนต์ จึงไม่ทำงานเอง)
  set_bord        : เลือกไดรเวอร์, กลับทิศ Motor, ช่วงพัลส์ Servo และความถี่

เริ่มต้น
  1. เลือกบอร์ด Raspberry Pi Pico 2 ใน Arduino-Pico core ของ Earle Philhower
  2. ติดตั้ง PiperPico2 และ dependency Adafruit ตาม library.properties
  3. เปิด PiperPico2_Basic.ino ไฟล์ README และ set_bord จะเปิดเป็นแท็บด้วย
  4. ตั้งค่าฮาร์ดแวร์ใน set_bord แล้วอัปโหลด เปิด Serial Monitor ที่ 115200
  5. หมุน knob เลือกเมนู กดสั้นยืนยัน กดค้าง 3 วินาทีย้อนกลับ
  6. กดสั้นเลือก RUN มีเสียงบิ๊ปยาว 500 ms แล้วออกลูปเมนูเข้าโปรแกรมหุ่นยนต์ทันที

เมื่อเลือก RUN สเก็ตช์นี้อ่าน Sensor / แบตเตอรี่ / Encoder อย่างเดียวเป็นค่าเริ่มต้น
ยังไม่มีคำสั่งให้ Motor หมุนหรือ Servo เคลื่อน
คัดลอกตัวอย่างคำสั่งด้านล่างไปยังตำแหน่งที่ต้องการในแท็บหลัก
ไม่ต้องสร้าง robot, setup(), loop() หรือเรียก begin() ซ้ำ

การเริ่มบอร์ดและวินิจฉัยเปิดไฟครั้งแรก
  begin() ปิด PWM Motor ก่อน รอไฟ 250 ms แล้วลองเริ่ม I2C/PCA9685/OLED ใหม่
  มีช่วงลองใหม่ประมาณ 2 วินาที บวกเวลา I/O และตั้ง I2C timeout 25 ms ต่อรายการ
  บัสที่ยังไม่ปล่อยสายจะถูกเช็ก/กู้คืนก่อนเริ่ม I2C ไม่ใช่หยุดตั้งแต่ครั้งแรก
  ถ้าเริ่มไม่ได้และจอยังตอบ จะขึ้น BOARD INIT FAILED พร้อมสถานะ PWM
  หากบอร์ดเริ่มไม่ผ่าน Serial Monitor 115200 จะแสดงสถานะซ้ำทุกวินาที
  หลังแก้ต้องทดลองตัดไฟ/เปิดจริงหลายครั้ง การทดสอบคอมไพล์ไม่ยืนยันวงจรจ่ายไฟ

  ตรวจพบ BNO055 กับ TCS0 ใช้ 0x29 บน Wire เดียวกัน:
    ขึ้น ADDR CONFLICT 0x29 แทนการเริ่มอุปกรณ์ที่ชนกัน
    ยังเริ่ม TCS1 บน Wire1 ได้ และเข้าหน้าเมนูได้
    ต้องเปลี่ยน ADR ของ BNO055 จริงเป็น 0x28 และแก้ BOARD_BNO055_ADDRESS=0x28
    แก้แต่ค่าในโค้ดไม่สามารถย้าย address ของฮาร์ดแวร์ให้เอง
    ค่าตั้ง 0x29 ปัจจุบันยังคงไว้ตามบอร์ดจริงที่ผู้ใช้ยืนยัน

เมนู OLED ที่ปรับจาก example/piper_spt/UI.ino
  RUN                 บิ๊ปยาวแล้วออกเมนูเข้าโปรแกรมหลังลูปรอใน setup() และ loop()
  READSENSOR > ADC     adcRead / adcMD / adcMax / adcMin ครบ 10 ช่อง
             > COLOR  อ่านชื่อสี T0/T1 แยกตามสถานะอุปกรณ์
             > GYRO   BNO055: มุม Yaw (deg), อัตราหมุน Z (deg/s), สถานะคาลิเบรต
             > ENCODER แสดง Encoder 4 ตัวแบบ 2 แถว 2 คอลัมน์
  CALIBRATESENSOR > ADC-Cal    เก็บ 10 วินาทีและแสดงผลบันทึก EEPROM
                  > TCS-Cal > TCS0-Cal เก็บสีเฉพาะ TCS0
                            > TCS1-Cal เก็บสีเฉพาะ TCS1

  knob ที่ MUX 10 ใช้เลือกเมนู: ปรับ BOARD_KNOB_MAX ตาม ADC สูงสุดที่วัดจริง
  หน้าจอ 128x32 ใช้ 3 แถวใต้หัวข้อ เมนู ADC เลื่อนเพื่อแสดงรายการที่ 4
  กดสั้นเลือกเมื่อปล่อยปุ่ม มี debounce 30 ms; กดค้าง 3 วินาทีย้อนกลับ
  ขณะอยู่ในเมนูคาลิเบรต กดค้าง 3 วินาทีเพื่อยกเลิกการเก็บค่า
  setup() รอ while (!ui_update()) จนเลือก RUN และเสียงบิ๊ปจบ แล้วออกจากลูปนี้
  เขียนโปรแกรมทำครั้งเดียวต่อท้ายลูปรอใน setup() และโปรแกรมทำซ้ำใน loop()
  หลังเข้า RUN ไม่มีการอ่านปุ่ม UI ไม่มีกดค้างหยุดหรือกลับเมนูอัตโนมัติ
  ไม่มีเสียงแบตเตอรี่แบบบล็อก; เปิด BOARD_UI_BATTERY_BEEP ได้หากต้องการ
  LOW ใช้ BOARD_LOW_BATTERY_V ไม่ใช่การวัดเปอร์เซ็นต์แบตเตอรี่

อุปกรณ์เสริมใน set_bord
  BOARD_ENABLE_COLOR=1 : ต้องติดตั้ง my_TCS และ SoftwareWire
    T0 อยู่บน Wire SDA4/SCL5, T1 อยู่บน Wire1 SDA26/SCL27 ที่ address 0x29
    เลือก CALIBRATESENSOR > TCS-Cal > TCS0-Cal หรือ TCS1-Cal
    เลือกตัวแล้ววาง RED และกดสั้น 1 ครั้ง เพื่อเก็บ 1 สีเมื่อปล่อยปุ่ม
    ทำตามหน้าจอต่อ: GREEN, BLUE, YELLOW, WHITE, BLACK รวม 6 ครั้ง
    ระหว่างกดค้างไม่เก็บสีซ้ำ; กดค้าง 3 วินาทีเป็นการยกเลิก
    เก็บครบจะบันทึกเฉพาะตัวที่เลือก แสดงผล และกลับเมนู TCS-Cal
    อีก Sensor ไม่ถูกเริ่ม/ล้าง/บันทึกใหม่ จนกว่าจะเลือกคาลิเบรตตัวนั้นเอง
    การอ่านสีจาก my_TCS::getColor() ตรวจ RED/YELLOW/GREEN/BLUE ก่อน
    รวมการจับคู่ระยะห่างเฉพาะ 4 สีหลัก ก่อนพิจารณาขาว/ดำ
    หากไม่ตรงสีหลัก จึงใช้ clear channel c อย่างเดียว:
      c < 180  -> BLACK (เมื่อมี calibration BLACK)
      c > 1800 -> WHITE (เมื่อมี calibration WHITE)
      ระหว่างเกณฑ์ -> UNKNOWN
    c ต่ำกว่า 30 รวมถึง 0 ยังพิจารณา BLACK ได้ โดยไม่หารด้วยศูนย์
    ต้องมี calibration ก่อนใช้การจำแนก; ตัวอย่าง UI จะอ่านค่าจริงสำเร็จก่อนเรียก getColor()
    ยกเลิกก่อนบันทึกจะโหลดค่าเดิมกลับเฉพาะตัวที่เลือก
    หากอ่านไม่สำเร็จหรือแสงต่ำจนคาลิเบรตไม่ได้ จะคงสีเดิมและให้กดสั้นลองใหม่
    ค่า EEPROM และการตอบสนองจริงของ Sensor ต้องตรวจบนฮาร์ดแวร์ด้วย
    ใช้ API startColorCalibration/allColorsCalibrated/lastCalibrationSaveSucceeded
    ของ my_TCS ที่ติดตั้งในเครื่องนี้; รุ่นอื่นต้องรองรับ API เหล่านี้
    EEPROM สีอยู่ 0x0080..0x00A4 และ 0x00C0..0x00E4
    ส่วน ADC ของ PiperPico2 อยู่ 0x0000..0x0027 จึงไม่ทับกันในผังนี้
    my_TCS ตรวจพื้นที่ก่อนบันทึก หากชนข้อมูลอื่นจะรายงาน save failed
  BOARD_ENABLE_GYRO=1 เปิดไว้แล้ว โดย BOARD_GYRO_TYPE=BOARD_GYRO_BNO055
    ใช้ไลบรารี BNO055.h ที่ติดตั้ง และตั้ง BOARD_BNO055_ADDRESS ให้ตรงโมดูล
    BNO055 ใช้ Wire SDA4/SCL5; หากใช้ TCS0 ร่วมด้วย ต้องตั้ง ADR จริงและโค้ดเป็น 0x28
    ค่าเริ่มต้น BOARD_ENABLE_COLOR=0 สำหรับอ่าน BNO055 ที่ 0x29; ถอด TCS0 ออกจาก Wire
    เมื่อเปิด COLOR: TCS0 เป็น 0x29 บน Wire และ TCS1 เป็น 0x29 บน Wire1
    หากเปิด COLOR ด้วย ห้ามให้ BNO055 0x29 ชน TCS0 0x29 บน Wire เดียวกัน
    เริ่มต้น BNO055 ด้วย bus ที่บอร์ดเริ่มแล้ว และโหลด gyro offset เดิมถ้ามี
    ไม่มีการสั่งคาลิเบรตใหม่หรือบันทึก EEPROM อัตโนมัติจากหน้าอ่านค่า
    หน้าจอแสดง:
      BNO055 SGAM:....  ตัวเลข 0..3 ตามลำดับ System/Gyro/Accel/Mag
      Y: ... deg        มุม Yaw สะสมที่กรองแล้ว เทียบจุดเริ่มต้น ไม่ใช่ heading absolute
      Z: ... deg/s      อัตราการหมุนแกน Z เป็นคนละค่ากับมุม Yaw
      Hold3s back       ย้อนกลับเมนู READSENSOR
    0=ยังไม่คาลิเบรต, 3=คาลิเบรตครบ; ???? หมายถึงอ่านสถานะไม่สำเร็จ
    อ่านค่าไม่สำเร็จแสดง READ ERROR ไม่แสดงค่าเก่าราวกับเป็นค่าปัจจุบัน
    HEADING REJECTED หมายถึง driver ปฏิเสธค่ามุมที่ไม่สมเหตุสมผล
    มุมเกินขอบเขตแสดงผล +/-99999.9 deg จะแสดง out of range
    หลัง RUN UI ไม่อ่านเซ็นเซอร์ต่อ หากต้องใช้ในโปรแกรมหุ่นยนต์ให้เรียกเอง เช่น:
      float yaw, zRate;
      if (readGyro(yaw, zRate)) {
        Serial.println(yaw);
      }
    ตัวอย่างนี้ใช้ API loadStoredGyroCalibration/calibrationStatus/lastUpdateError
    ของไลบรารี BNO055 ที่ติดตั้งในเครื่องนี้
  หากต้องการ BMI160 เดิม ตั้ง BOARD_GYRO_TYPE=BOARD_GYRO_BMI160
    ใช้ my_BMI160 และ address ของ uiGyro; ค่า Z ของ driver นี้เป็นมุม deg
    เริ่มต้นเก็บ offset แบบบล็อก และไม่มีสถานะ read error รายครั้งแบบ BNO055
  ค่าเริ่มต้นเปิด GYRO และปิด COLOR ทั้งสองช่อง
  ถ้าเปิดแล้วหาอุปกรณ์ไม่พบ จะแสดง NOT FOUND และยังใช้เมนูหลักได้
  driver เสริมยังบล็อกช่วงเริ่มต้นและการเก็บสีบางขั้น

1) ตั้งค่าบอร์ดและเลือก Motor driver — ทำใน set_bord ก่อน begin()
  เปลี่ยนสมาชิก BOARD_MOTOR_DRIVERS ตามลำดับ A, B, C, D เช่น
    PiperPico2::MotorDriver::DRV8874
  หรือใช้คำสั่งตั้งค่ารายตัวก่อน robot.begin():
    robot.setMotorDriver('A', PiperPico2::MotorDriver::DRV8874);
    robot.setMotorDriver('C', PiperPico2::MotorDriver::DRV8874, true);
  true ตัวท้ายกลับทิศของความเร็ว; false ใช้ทิศปกติ
  เลือกไดรเวอร์ได้ก่อน begin() ครั้งแรกเท่านั้น; คืน false หากตั้งไม่สำเร็จ

  DRV8874 รองรับโหมด PH/EN (PMODE=GND), EN เป็น GPIO PWM:
    Motor   GPIO EN    PCA PH    PCA nSLEEP
      A        8         0           1
      B        9         4           5
      C        7         2           3
      D        6         6           7
  A/C ตรงกับวงจรที่ให้มา โดย ledN หมายถึง PCA9685 ช่อง N
  B/D ใช้คู่ช่องตามผังของไลบรารี ต้องตรวจการต่อจริงหากใช้ DRV8874

2) สั่ง Motor — ความเร็ว -100 ถึง 100
    robot.motor('A', 30);    // ทิศบวก 30%
    robot.motor('C', -30);   // ทิศลบ 30%
    delay(200);             // ตัวอย่างรอแบบบล็อก 200 ms
    robot.stopMotors();     // หยุดทั้ง A, B, C, D

    robot.motor('A', 0);    // หยุดเฉพาะ A
    bool ok = robot.motorOK(); // สถานะคำสั่ง Motor ล่าสุด
  DRV8874: ความเร็ว 0 เป็นการเบรก โดยยังคง nSLEEP HIGH
  motorOK() ตรวจผลเขียนคำสั่ง ไม่ใช่การอ่านขา nFAULT หรือการยืนยันว่ามอเตอร์หมุน
  คำสั่ง Motor คงอยู่จนกว่าจะสั่งใหม่ ให้กำหนดเงื่อนไขหยุด/timeout ในโปรแกรมจริง

3) ตั้งช่วงพัลส์และสั่ง Servo — ช่อง 0 ถึง 5
  ปรับ BOARD_SERVO_MIN_US / BOARD_SERVO_MAX_US ใน set_bord เป็นรายช่อง
  หน่วยเป็นไมโครวินาที ค่าตั้งต้องมาจาก Servo และขีดจำกัดกลไกจริง
  ตัวอย่าง API หากต้องเปลี่ยนช่วงก่อนคำสั่งครั้งถัดไป:
    bool accepted = robot.setServoPulseRangeUs(0, 1000, 2000);
    if (accepted) {
      robot.setservo(0, 90); // กึ่งกลางของช่วงพัลส์ที่ตั้งไว้
    }
  ค่าตัวอย่างนี้ไม่ใช่การรับรองช่วงของ Servo ทุกรุ่น
  ช่วงนี้ถูกแมปจากมุมคำสั่ง 0–180 ไม่ใช่การวัดองศาจริงของข้อต่อ
  อีก API ใช้หน่วย PWM counts แบบเดิม ไม่ใช่ไมโครวินาที:
    robot.setServoPulseRange(1, 120, 600);
  counts ต้องอยู่ในช่วง 1..4095 และ min < max
  การตั้งช่วงไม่ทำให้เคลื่อนทันที; เริ่มใช้เมื่อเรียก setservo() ครั้งถัดไป
  หากตั้งไม่ผ่าน ค่าเดิมยังอยู่; ควรตรวจผล bool ก่อนสั่งเคลื่อน
  ช่อง 0,1,2,3,4,5 -> PCA9685 ช่อง 15,14,13,12,11,10
  ช่วงพัลส์เก็บใน RAM ต้องตั้งใหม่หลังเปิดเครื่อง; set_bord จัดการให้ทุกครั้ง
  ความถี่ Servo ยังคง 60 Hz; ไม่มีการไล่มุม/จำกัดความเร็ว/อ่านตำแหน่งกลับ
  setservo() ไม่รอให้เคลื่อนถึงเป้าหมาย และไม่มีผลลัพธ์ยืนยันตำแหน่ง

4) อ่าน Sensor เส้น 10 ช่อง — ช่อง 0 ถึง 9
    uint16_t raw = robot.readLine(0);       // ค่าดิบ 12 บิต 0..4095
    uint16_t values[NUM_SENSORS];
    robot.readLines(values);               // อ่านครบ 10 ช่องในอาร์เรย์
    float normalized = robot.readNormalized(0); // ช่วง 0.0..1.0
    uint16_t minimum = robot.adcMin(0);     // ค่าต่ำสุดจากคาลิเบรต
    uint16_t maximum = robot.adcMax(0);     // ค่าสูงสุดจากคาลิเบรต
    uint16_t middle = robot.adcMD(0);       // ค่าเฉลี่ย min/max
  normalized จะมีความหมายตามคาลิเบรตที่เก็บไว้; ตรวจ calibrationReady()
  อ่าน MUX โดยตรง:
    uint16_t muxValue = robot.adcRead(3);   // รับช่อง MUX 0..15
  ช่อง 10 ของบอร์ดเป็นปุ่มหมุน ช่อง 11 เป็นแบตเตอรี่:
    float knob = robot.knopRead();         // ค่าที่ไลบรารีแมปจากปุ่มหมุน

5) คาลิเบรต Sensor แบบรอจนจบ
    robot.stopMotors();
    robot.calibrate();                    // เก็บประมาณ 10 วินาที แล้วบันทึก EEPROM
    bool saved = robot.calibrationSaved();
  ระหว่างเก็บ ให้ทุก Sensor เห็นพื้นผิวทั้งสองระดับที่ต้องการคาลิเบรต
  โหลด/บันทึกด้วยตัวเองเมื่อจำเป็น และตรวจค่า bool:
    bool loaded = robot.loadCalibration();
    bool written = robot.saveCalibration();
  ไม่ควร saveCalibration() ทุก loop เพราะเขียน EEPROM ซ้ำโดยไม่จำเป็น

6) คาลิเบรตแบบทำทีละรอบ
  เรียกเริ่มครั้งเดียวเมื่อผู้ใช้สั่ง:
    robot.stopMotors();
    bool started = robot.startCalibration(10000);
  จากนั้นเรียกส่วนนี้ใน loop() ระหว่างรอ:
    if (robot.isCalibrating()) {
      robot.updateCalibration();
      if (!robot.isCalibrating()) {
        bool saved = robot.calibrationSaved();
      }
    }
  อย่าเริ่มใหม่อัตโนมัติทุก loop หลังจบ; ตัวอย่างนี้เป็นส่วนแยกสำหรับนำไปใช้
  ยกเลิกได้ด้วย robot.cancelCalibration();
  ช่วงอ่านเป็นทีละรอบ แต่รอบสุดท้ายยังบล็อกขณะบันทึก EEPROM

7) Encoder — ใช้เจ้าของชุดเดียวจาก robot
  เปิด READSENSOR > ENCODER จะแสดงแบบ 2 แถว 2 คอลัมน์:
    1L: enc1Left()       1R: enc1Right()
    2L: enc2Left()       2R: enc2Right()
  ค่าปรับปรุงตามรอบ UI; กดค้าง 3 วินาทีกลับ READSENSOR
  อ่านอย่างเดียว ไม่ reset หรือเปลี่ยนทิศการนับเมื่อเข้า/ออกหน้านี้
  เมนู READSENSOR มี 4 รายการ หมุน knob ลงท้ายเพื่อเลื่อนเห็น ENCODER
  ค่าที่ยาวเกินช่องจอจะแสดงย่อด้วย M (ล้านพัลส์) และปัดเศษเฉพาะการแสดงผล
  ค่าจริงเต็มยังอ่านได้จาก getter และ Serial ในโปรแกรมหลัก
    long e1Left  = robot.enc1Left();
    long e1Right = robot.enc1Right();
    long e2Left  = robot.enc2Left();
    long e2Right = robot.enc2Right();
    robot.resetEncoders(); // ล้างทั้ง 4 ตัวพร้อมกันบน core ที่เรียก
  ตั้งลำดับขา A/B ได้ที่ BOARD_ENCODER_PINS ในแท็บ set_bord:
    index 0 = enc1Left()  : {20,19} กลับทิศเป็น {19,20}
    index 1 = enc1Right() : {21,22} กลับทิศเป็น {22,21}
    index 2 = enc2Left()  : {16,17} กลับทิศเป็น {17,16}
    index 3 = enc2Right() : {10,18} กลับทิศเป็น {18,10}
  ตัวอย่าง API ก่อน begin():
    bool accepted = robot.setEncoderPins(0, 19, 20);
  ขาแรกเป็น A สำหรับ interrupt ขอบขาขึ้น ขาที่สองเป็น B สำหรับอ่านทิศ
  การสลับ A/B กลับเครื่องหมายการนับ อาจเปลี่ยนตำแหน่งขอบที่นับในหนึ่งรอบ
  สลับได้เฉพาะคู่ขาเดิมของตัวนั้น; คู่ผิดหรือเรียกหลัง begin() คืน false
  ค่านี้กลับทิศ Encoder เท่านั้น ไม่ได้กลับทิศ Motor โดยอัตโนมัติ
  ค่าเป็นจำนวนพัลส์ ไม่ใช่เซนติเมตร; ต้องแปลงตามล้อ/อัตราทดจริง
  สเก็ตช์นี้ใช้ begin() ที่เริ่ม interrupt ภายใน ไม่ต้องสร้าง Encoder เพิ่ม
  หากแอปเป็นผู้ดูแล Encoder เอง ให้เลือก begin(false) และใช้ค่าจากแอปให้สอดคล้อง

8) แบตเตอรี่
    float immediate = robot.getVoltage(); // อ่านแรงดันทันที หน่วย V
    robot.updateBattery();                // อัปเดตค่าที่เก็บไว้
    float cached = robot.getBatteryVoltage();
    robot.bat_control();                  // alias สำหรับอัปเดตค่า ไม่ตัด Motor อัตโนมัติ
    robot.showVoltageUntilButton();       // หยุด Motor และแสดงจนกดปุ่ม (บล็อก)
  ค่าแรงดันขึ้นกับวงจรแบ่งแรงดัน/ADC จริง ยังไม่ได้สอบเทียบเฉพาะบอร์ดของคุณ

9) เสียงและปุ่มกด
  เปิดเครื่องแล้วเล่นเมโลดี C-D-E-G-E-D-C สูง ประมาณ 1 วินาทีพร้อมหน้าต้อนรับ
  เล่นครั้งเดียวหลังเริ่มบอร์ดสำเร็จ ไม่ใช้ delay รอแต่ละโน้ตในลูป UI
  ปรับได้ที่ set_bord:
    BOARD_BOOT_MELODY_ENABLED = true;  // false เพื่อปิดเมโลดี
    BOARD_BOOT_MELODY[][2]             // แต่ละคู่คือ {ความถี่ Hz, ระยะเวลา ms}
    BOARD_BOOT_NOTE_GAP_MS             // ช่องว่างระหว่างโน้ต
  ความถี่ 0 ใช้เป็นช่วงพัก; หน้าต้อนรับจะรอจนทำนองจบก่อนเปิดเมนู
  เสียง RUN ยังคงเป็นบิ๊ปยาว 500 ms แยกจากเมโลดีเปิดเครื่อง
  สำหรับ KLJ-7525-3627 ตั้ง BOARD_RUN_TONE_HZ=2731 และเมโลดีช่วง 2093–3136 Hz
  อ้างอิงความถี่สเปก 2731 Hz ของผู้ผลิต; ยังไม่ได้วัดความดังบนบอร์ดจริง
  รุ่นนี้ระบุแรงดัน 3.6 Vp-p, duty 50% และกระแสสูงสุด 90 mA ที่แรงดันพิกัด
  ต้องใช้วงจรขับที่รองรับ ไม่ควรต่อขดลวดบัซเซอร์รับกระแสจาก GPIO โดยตรง
  tone() ไม่เพิ่มแรงดัน/กระแสหรือปรับ volume; ความดังจริงขึ้นกับวงจรและตำแหน่งติดตั้ง
    robot.playToneAsync(2000, 100);        // 2000 Hz นาน 100 ms ไม่รอเสียงจบ
    robot.playTone(2000, 100);             // รอเสียงจบก่อนทำต่อ
    bool pressed = digitalRead(PiperPico2::BUTTON_PIN) == LOW;
  BUTTON_PIN=2 เป็น INPUT_PULLUP; BUZZER_PIN=11
  เมื่อนำปุ่มไปเริ่มการทำงาน ควรตรวจขอบสัญญาณ/หน่วงเด้งเพื่อไม่ให้สั่งซ้ำขณะกดค้าง

10) ความถี่และสถานะ
    bool accepted = robot.setI2CClock(100000); // 100–400 kHz
    robot.setMotorPWMFrequency(15000);        // GPIO PWM Motor ไม่เปลี่ยน Servo
    bool pwmReady = robot.pwmReady();
    bool oledReady = robot.displayReady();
    bool calibrated = robot.calibrationReady();
  ตั้งค่าเหล่านี้รวมไว้ใน set_bord ได้ ไม่จำเป็นต้องเรียกซ้ำใน loop()
  หากโค้ดอื่นเปลี่ยน Wire clock ค่า clock ของบัสที่ใช้ร่วมกันอาจเปลี่ยนตาม

ขั้นสูง: getPWM() ให้เข้าถึง PCA9685 โดยตรง แต่จะข้ามขีดจำกัด Servo รายช่อง
หากแก้ช่อง Motor ด้วย reference ที่เก็บไว้ ให้เรียก invalidateMotorCache() ภายหลัง

ใน PiperPico2_Basic เรียกฟังก์ชันของ BNO055 ผ่านออบเจ็กต์ uiBno055 ครับ
ฟังก์ชัน	ใช้ทำอะไร
uiBno055.begin(...)	เริ่ม BNO055 ที่ 0x29 บน Wire โดย set_bord() จัดการให้แล้ว
uiBno055.update()	อ่านข้อมูลรอบใหม่ คืน true เมื่ออ่านสำเร็จ
uiBno055.yaw()	อ่านมุม Yaw สะสมเทียบกับตอนเริ่มต้น หน่วยองศา
uiBno055.gyro('z')	อ่านอัตราหมุนรอบแกน Z หน่วยองศาต่อวินาที
uiBno055.calibrationStatus(status)	อ่านระดับคาลิเบรต System, Gyro, Accelerometer และ Magnetometer ค่าระดับ 0–3
uiBno055.loadStoredGyroCalibration()

if (uiGyroReady && uiBno055.update()) {
  const float yaw = uiBno055.yaw();
  const float zRate = uiBno055.gyro('z');

  // ใช้ yaw คุมมุม และใช้ zRate ดูว่าหุ่นกำลังหมุนเร็วแค่ไหน
}

*/


void read_sensor()
  {

    if (!boardReady) {
    static uint32_t lastFailureReport = 0;
    if (millis() - lastFailureReport >= 1000) {
      lastFailureReport = millis();
      Serial.print("Boot failed. Check power/I2C. PWM ready: ");
      Serial.print(robot.pwmReady());
      Serial.print(" OLED ready: "); Serial.println(robot.displayReady());
    }
    delay(20);
    return;
  }
    // ตัวอย่างอ่านค่าอย่างเดียวทุก 500 ms
  static uint32_t lastReport = 0;
  const uint32_t now = millis();
  if (now - lastReport >= 500) {
    lastReport = now;
    uint16_t lines[NUM_SENSORS];
    robot.readLines(lines);
    robot.updateBattery();

    Serial.print("Battery(V): ");
    Serial.print(robot.getBatteryVoltage(), 2);
    Serial.print(" | Lines[0..9]: ");
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
      Serial.print(lines[i]);
      Serial.print(' ');
    }
    Serial.print("| Encoders: ");
    Serial.print(robot.enc1Left());  Serial.print(' ');
    Serial.print(robot.enc1Right()); Serial.print(' ');
    Serial.print(robot.enc2Left());  Serial.print(' ');
    Serial.println(robot.enc2Right());
  }
  delay(1);
  }
