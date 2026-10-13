void robot_setup()
  {
    // ค่าการอ่านเซนเซอร์: กรองมากขึ้นช่วยลดค่าสั่น แต่ตอบสนองช้าลง
    const uint16_t muxSettleUs = 40;        // รอหลังสลับช่อง MUX (4–100 µs)
    const uint8_t adcDiscardReads = 1;      // ทิ้งค่า ADC แรกหลังสลับช่อง (1–4)
    const uint8_t adcAverageReads = 2;      // จำนวนครั้งที่อ่านมาเฉลี่ย (1–8)
    const uint8_t smoothingDivisor = 4;     // กรองข้ามเฟรม: 1=ปิด, 2/4/8=มากขึ้น
    Serial.begin(115200);                   // ความเร็ว Serial Monitor (baud)
    select_motor_driver(MotorDriverType::DRV8874);  // เลือกไดรเวอร์ก่อนเริ่มหุ่นยนต์
    robot_begin();                          // เริ่มอุปกรณ์ โหลดคาลิเบรต และรีเซตมุมไจโร
    if (!motor_driver_ready()) {
      Serial.println(F("DRV8874 is not ready; see motor_driver_status()."));
      for (;;) delay(1000); }
    // นำค่าการอ่านด้านบนไปใช้กับเซนเซอร์หน้าและหลัง
    if (!configure_sensor_reading(muxSettleUs, adcDiscardReads,
                                  adcAverageReads, smoothingDivisor)) {
      Serial.println(F("Invalid sensor reading settings"));
      motor(0, 0);
      for (;;) delay(1000);
    }

    // กำหนดช่วงพัลส์ของแต่ละช่องก่อนเรียก servo(); ปรับตามขีดจำกัดเซอร์โวจริง
    set_servo_pulse_range(18, 500, 2500); // GP18: เซอร์โวช่อง 1, คำสั่ง 0-180°
    set_servo_pulse_range(22, 500, 2500); // GP22: เซอร์โวช่อง 2, คำสั่ง 0-180°
    set_servo_pulse_range(28, 500, 2500); // GP28: เซอร์โวช่อง 3, คำสั่ง 0-180°
    set_servo_pulse_range(0, 500, 2500);  // GP0: ช่องเสริม เมื่อไม่ได้ใช้ UART0 TX
    set_servo_pulse_range(1, 500, 2500);  // GP1: ช่องเสริม เมื่อไม่ได้ใช้ UART0 RX
    set_line_distance_scale(1.75f);     // ชดเชยระยะ: สั่ง 20 ซม. วิ่งจริง 35 ซม. → 35/20
    set_turn_overshoot(20);             // fl/fr วิ่งต่อหลังข้ามเส้น 20 ms ก่อนเบรก
    set_turn_touch_brake_ms(20);        // เวลาเบรกสวนก่อนหมุน fl/fr และ turn_gyro (ms)
    set_turn_timeout(3000);            // เวลาสูงสุดของ turn/turn_gyro (ms)
    set_turn_approach_kp(0.250f);       // Kp ช่วงตามเส้นเข้าหาจุดหมุน
    set_turn_line_search(60, 100);     // turn(): หมุนเร็ว 100 ms แล้วจำกัดสปีดค้นหาเส้นที่ 60

    set_line_ramp_start_speed(5);       // สปีดเริ่ม f_line/b_line เมื่อสปีดที่สั่งตั้งแต่ 40
    set_line_ramp_ms(200);              // เวลาไล่ถึงสปีดที่สั่ง (ms); 0=ไม่ไล่สปีด
    set_line_decel_ramp_cm(100);        // ชะลอก่อนครบระยะ 1.00 ซม.; 100=1.00 ซม., 0=ปิด
    set_line_pid_tuning(0.0125f);       // Kd ของ f_line/b_line; Kp ใช้ค่าที่ส่งในคำสั่ง
    set_gyro_kd(0.00850f, 0.0125f);    // Kd ของ fw_gyro, bw_gyro ตามลำดับ
    set_rotate_fallback(300, 550, 50); // ใช้เฉพาะ rotate_spin/pivot เมื่อ gyro ใช้ไม่ได้; turn_gyro จะหยุดพร้อมแจ้งข้อผิดพลาด
    set_turn_approach(25, 25, 15);     // สปีดเข้าหาหลังจบด้วยเซนเซอร์/ระยะทาง, กำลังเบรกสวน
    set_turn_center_brake(25, 10, 20, 60);  // tcl/tcr: เบรกสวนหลังเดินหน้า 25/10 ms, หลังถอย 20/60 ms

    // กำลังล้อขณะหมุน: (ซ้าย, ขวา) เป็นร้อยละของสปีดที่สั่ง; ค่าลบคือถอย
    set_turn_motor(TurnMode::fl, -8, 100);   // fl: ซ้ายถอยเบา ขวาเดินหน้า
    set_turn_motor(TurnMode::fr, 100, -8);   // fr: ซ้ายเดินหน้า ขวาถอยเบา
    set_turn_motor(TurnMode::cl, -100, 100); // cl: หมุนซ้ายอยู่กับที่
    set_turn_motor(TurnMode::cr, 100, -100); // cr: หมุนขวาอยู่กับที่
    set_turn_motor(TurnMode::l, -100, 100);  // l: หมุนซ้ายอยู่กับที่
    set_turn_motor(TurnMode::r, 100, -100);  // r: หมุนขวาอยู่กับที่
  }

bool gyro_mini() {
  float yaw = 0.0f;
  float rateX = 0.0f;
  float rateY = 0.0f;
  float rateZ = 0.0f;
  if (!gyro_read_raw(yaw, rateX, rateY, rateZ)) {
    Serial.println(F("gyro_mini: sensor read failed"));
    return false;
  }
  Serial.print(F("gyro_mini raw yaw before reset: "));
  Serial.println(yaw, 2);
  Serial.print(F("gyro_mini rate x/y/z (deg/s): "));
  Serial.print(rateX, 2);
  Serial.print('/');
  Serial.print(rateY, 2);
  Serial.print('/');
  Serial.println(rateZ, 2);

  if (!gyro_reset() || !gyro_read_raw(yaw, rateX, rateY, rateZ)) {
    Serial.println(F("gyro_mini: reset or verification failed"));
    return false;
  }
  Serial.print(F("gyro_mini raw yaw after reset: "));
  Serial.println(yaw, 2);
  return true;
}

bool gyro_set()
  {
    if (!gyro_start_ready() && !gyro_recover()) {
    Serial.println(F("Gyro not ready; mission stopped."));
    motor(1, 1);
    return false;
  }
  if (!gyro_mini()) {
    motor(1, 1);
    return false;
  }
  if (!gyro_start_ready()) {
    Serial.println(F("Gyro not ready after gyro_mini; mission stopped."));
    motor(1, 1);
    return false;
  }
  return true;
  }
void stopFor(uint32_t milliseconds) {
  motor(1, 1);  // Hold both wheels with the PH/EN brake.
  delay(milliseconds);
}
