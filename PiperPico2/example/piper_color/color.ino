/*
ขั้นตอนการ calibrate (ทีละสี)

เตรียมสภาพแวดล้อม
ใช้แสงสม่ำเสมอ (เช่น ไฟ LED ขาว หรือแสงธรรมชาติที่คงที่)
วางวัตถุสีมาตรฐานให้อยู่ตรงหน้าเซ็นเซอร์ (ห่างประมาณ 1–2 ซม. หรือตามที่ใช้จริง)
ไม่ให้แสงรบกวนจากภายนอก (เช่น แสงแดด, หลอดไฟกระพริบ)

เรียกฟังก์ชัน calibrate แต่ละสี (ทำทีละสีตามลำดับ)C++tcs.calibrateRed();     // วางวัตถุสีแดงมาตรฐาน แล้วเรียก
tcs.calibrateGreen();   // วางสีเขียว
tcs.calibrateBlue();    // วางสีน้ำเงิน
tcs.calibrateYellow();  // วางสีเหลือง
tcs.calibrateWhite();   // วางสีขาว (กระดาษขาวสะอาด หรือวัตถุขาว)
tcs.calibrateBlack();   // วางสีดำ (พื้นผิวด้านดำ หรือปิดเซ็นเซอร์)
แต่ละฟังก์ชันจะ:
อ่านค่า raw 2 ครั้ง (เพื่อความเสถียร)
คำนวณ ratio: ref_ratio_r[i] = r / c, ref_ratio_g[i] = g / c, ref_ratio_b[i] = b / c
ตั้ง flag (เช่น cal_red = true)


จบการ calibrate
เรียกฟังก์ชันสุดท้ายเพื่อยืนยันและบันทึก:C++tcs.calibrateDone();
ตั้ง calibrated = true
เรียก saveColorCalibration() อัตโนมัติ → บันทึกค่า ratio และ flags ลง EEPROM


การบันทึกและโหลดค่า (EEPROM)

บันทึกอัตโนมัติ เมื่อเรียก calibrateDone()
โหลดอัตโนมัติ เมื่อเรียก begin() → ทำให้ครั้งต่อไปเปิดเครื่องใหม่ไม่ต้อง calibrate ซ้ำ (ถ้ามีค่าใน EEPROM แล้ว)
แต่ละเซ็นเซอร์ (tcs0 และ tcs1) ใช้ offset แยกกันใน EEPROM (เช่น 0x0000 และ 0x0040) จึงไม่ทับกัน

ตัวอย่างการ calibrate ใน sketch (สำหรับเซ็นเซอร์ 1 ตัว)
C++void setup() {
  Serial.begin(115200);
  if (!tcs0.begin(TCS34725_ADDRESS, &Wire)) {
    Serial.println("TCS0 ไม่พบ!");
    while(1);
  }

  Serial.println("เริ่ม calibrate...");
  Serial.println("วางสีแดงแล้วกด Enter ใน Serial Monitor");
  while (Serial.available() == 0) delay(100);
  Serial.read(); // ล้าง buffer
  tcs0.calibrateRed();

  Serial.println("วางสีเขียวแล้วกด Enter");
  while (Serial.available() == 0) delay(100);
  Serial.read();
  tcs0.calibrateGreen();

  // ทำต่อสำหรับสีอื่น ๆ ...

  tcs0.calibrateDone();
  Serial.println("Calibrate เสร็จสิ้น! ค่าเก็บใน EEPROM แล้ว");
}
เคล็ดลับการ calibrate ให้แม่นยำ

วัดแต่ละสีหลายครั้ง (ไลบรารีทำ 2 ครั้งแล้ว แต่คุณสามารถอ่านหลายรอบก่อนเรียก calibrate ได้)
ใช้สีมาตรฐานจริง (เช่น กระดาษสีมาตรฐาน หรือวัตถุที่ใช้ในโปรเจกต์จริง)
ทำในสภาพแสงที่ใช้จริงเสมอ (ถ้าใช้ในห้องมืด/สว่างต่างกัน ให้ calibrate ใหม่)
ถ้าค่ากลับมา "UNKNOWN" บ่อย → ลองปรับ threshold ใน getColor() (เช่น c < 30 แทน 20)

ถ้าต้องการตัวอย่าง sketch เต็มสำหรับ calibrate ทั้งสองตัว (พร้อมแจ้งเตือนทีละสี) หรืออยากปรับ logic getColor() ให้เข้มงวด/ผ่อนคลายขึ้น บอกได้เลยครับ จะช่วยปรับให้ 😊
*/