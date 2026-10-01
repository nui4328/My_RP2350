void robot_setup()
  {
      // Sensor reading settings used by f_line()/b_line(). Tune with sensor_diag
    // on uniform white and black before increasing smoothing further.
    const uint16_t muxSettleUs = 60;        // 4..100 us; library default is 4.
    const uint8_t adcDiscardReads = 1;      // 1..4; discard after switching ADC.
    const uint8_t adcAverageReads = 4;      // 1..8; library default is 2.
    const uint8_t smoothingDivisor = 4;     // 1=off, 2=light, 4/8=more lag.
    Serial.begin(115200);
    // TB6612FNG remains the default unless this line is present.
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
    set_turn_overshoot(100);            // วิ่งเลยเส้นขอบต่ออีก 20 ms ก่อนเบรกและเริ่มหมุน
    set_turn_timeout(3000);           // เวลาป้องกัน turn() ค้าง; fl/fr นับจากช่วงการทำงานปัจจุบัน โหมดอื่นนับจากเริ่มคำสั่ง
    set_turn_approach_kp(0.150f);      // ค่า P ขณะวิ่งตามเส้นเข้าหาจุดเริ่มเลี้ยว
    set_turn_touch_brake_ms(20);       // ระยะเวลาเบรกสวนก่อนหมุน 20 ms
    set_turn_line_search(100, 60);     // หมุนด้วยความเร็วที่สั่งใน 60 ms แรก จากนั้นจำกัดความเร็วค้นเส้นไว้ไม่เกิน 70    
    set_line_ramp_ms(100);
    

    set_line_pid_tuning(1.0f, 0.0f);    
    set_turn_approach(20, 30, 10);    // ความเร็วเข้าหาหลังจบด้วย sensor, หลังจบด้วย distance และกำลังเบรกสวน ตามลำดับ
    set_turn_motor(TurnMode::fl, -5, 100);
    set_turn_motor(TurnMode::fr, 100, -5);
    set_turn_motor(TurnMode::cl, -100, 100);
    set_turn_motor(TurnMode::cr, 100, -100);
    set_turn_motor(TurnMode::l, -100, 100);
    set_turn_motor(TurnMode::r, 100, -100);

    

  }


void stopFor(uint32_t milliseconds) {
  motor(1, 1);  // Hold both wheels with the PH/EN brake.
  delay(milliseconds);
}
