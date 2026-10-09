void robot_setup()
  {
      // Sensor reading settings used by f_line()/b_line(). Tune with sensor_diag
    // on uniform white and black before increasing smoothing further.
    const uint16_t muxSettleUs = 40;        // 4..100 us; library default is 40.
    const uint8_t adcDiscardReads = 1;      // 1..4; discard after switching ADC.
    const uint8_t adcAverageReads = 2;      // 1..8; library default is 2.
    const uint8_t smoothingDivisor = 4;     // 1=off, 2=light, 4/8=more lag.
    Serial.begin(115200);
    // DRV8874 is the library default; this line is optional.
    select_motor_driver(MotorDriverType::DRV8874);
    robot_begin();
    if (!motor_driver_ready()) {
      Serial.println(F("DRV8874 is not ready; see motor_driver_status()."));
      for (;;) delay(1000); }
    if (!configure_sensor_reading(muxSettleUs, adcDiscardReads,
                                  adcAverageReads, smoothingDivisor)) {
      Serial.println(F("Invalid sensor reading settings"));
      motor(0, 0);
      for (;;) delay(1000);
    }
    set_line_distance_scale(1.75f);  // 35 ÷ 20
    set_turn_overshoot(10);            // วิ่งเลยเส้นขอบต่ออีก 10 ms ก่อนเบรกและเริ่มหมุน
    set_turn_touch_brake_ms(10);       // ระยะเวลาเบรกสวนก่อนหมุนของ fl/fr และ turn_gyro (ms)
    set_turn_timeout(3000);           // เวลาป้องกัน turn() ค้าง; fl/fr นับจากช่วงการทำงานปัจจุบัน โหมดอื่นนับจากเริ่มคำสั่ง
    set_turn_approach_kp(0.250f);      // ค่า P ขณะวิ่งตามเส้นเข้าหาจุดเริ่มเลี้ยว    
    set_turn_line_search(70, 100);      // หมุนด้วยความเร็วที่สั่งใน 60 ms แรก จากนั้นลดเหลือไม่เกิน 40 เพื่อค้นหาเส้นหยุด


    set_line_ramp_start_speed(10);  // ความเร็วพื้นฐานตอนเริ่ม
    set_line_ramp_ms(200);          // เวลาไล่ถึงความเร็วที่สั่ง

    set_line_decel_ramp_cm(100);  // ชะลอในช่วง 2 ซม. สุดท้าย
    set_line_pid_tuning(0.0125f);  
    set_gyro_kd(0.00850f, 0.0125f);  // fw_gyro, bw_gyro
    set_rotate_pid(0.950f, 0.0025f);  // rotate_spin/FW_pivot/BW_pivot: Kp (angle), Kd (yaw rate)
    set_rotate_spin_slowdown(100, 20);  // rotate_spin: ลดเพดานความเร็วลงถึง 20 ภายใน 200 ms ก่อนถึงมุมเป้าหมาย
    set_turn_approach(25, 25, 30);    // ความเร็วเข้าหาหลังจบด้วย sensor, หลังจบด้วย distance และกำลังเบรกสวน ตามลำดับ
    set_turn_center_brake(25, 10, 20, 60);  // tcl/tcr: เดินหน้า กำลัง/เวลา(ms), ถอยหลัง กำลัง/เวลา(ms)
    set_turn_motor(TurnMode::fl, -8, 100);
    set_turn_motor(TurnMode::fr, 100, -8);
    set_turn_motor(TurnMode::cl, -100, 100);
    set_turn_motor(TurnMode::cr, 100, -100);
    set_turn_motor(TurnMode::l, -100, 100);
    set_turn_motor(TurnMode::r, 100, -100);
   

  }

void gyro_set()
  {
    if (!gyro_start_ready() && !gyro_recover()) {
    Serial.println(F("Gyro not ready; mission stopped."));
    motor(1, 1);
    return;
  }
  gyro_mini();
  if (!gyro_start_ready()) {
    Serial.println(F("Gyro not ready after gyro_mini; mission stopped."));
    motor(1, 1);
    return;
  }
  }
void stopFor(uint32_t milliseconds) {
  motor(1, 1);  // Hold both wheels with the PH/EN brake.
  delay(milliseconds);
}
