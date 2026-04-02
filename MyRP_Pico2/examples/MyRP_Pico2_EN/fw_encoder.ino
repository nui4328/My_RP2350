void fw_gyro_encoder(int spl_base, int spr_base, float kp, float target_distance_cm, int offset)
{
    if (target_distance_cm <= 0) {
        Motor(0, 0);
        return;
    }

    encoder.resetEncoders();
    my.resetAngles();

    float yaw_offset = my.gyro('z');
    float _integral = 0;
    float _prevErr = 0;
    unsigned long prevT = millis();

    long initial_ticks = (encoder.Poss_L() + encoder.Poss_R()) / 2;
    unsigned long start_time = millis();

    float current_speed = 0.0f;
    float max_speed = min(abs(spl_base), abs(spr_base)) * 1.0f;

    // พารามิเตอร์ Deceleration (ใช้เฉพาะความเร็วสูง)
    const float DECEL_DISTANCE_CM = 15.0f;
    const float MIN_SPEED         = 15.0f;

    // ตรวจสอบว่าเป็นการเดินหน้า หรือ เดินถอยหลัง
    bool is_forward = (spl_base >= 0 && spr_base >= 0);

    while (true)
    {
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        prevT = now;

        // ── คำนวณระยะทางที่เดินไปแล้ว (รองรับทั้งหน้าและหลัง) ─────
        long current_ticks = (encoder.Poss_L() + encoder.Poss_R()) / 2;
        long delta_ticks = current_ticks - initial_ticks;
        
        // เดินถอยหลังต้องกลับเครื่องหมายระยะทาง
        if (!is_forward) {
            delta_ticks = -delta_ticks;
        }

        float traveled_cm = delta_ticks * CM_PER_TICK;
        float remaining_cm = target_distance_cm - traveled_cm;

        if (remaining_cm <= 0.5f) {        // ใช้ 0.5f เพื่อป้องกัน noise
            break;
        }

        // ── คำนวณความเร็วเป้าหมาย ─────────────────────────────
        float target_speed;

        if (spl_base <= 20 && spl_base >= -20) {   // ความเร็วต่ำ (รวมถอยหลัง)
            target_speed = max_speed;              // วิ่งคงที่ ไม่ ramp ไม่ decel
            current_speed = max_speed;
        } 
        else {
            // ความเร็วสูง → มี ramp + deceleration
            if (remaining_cm > DECEL_DISTANCE_CM) {
                if (current_speed < max_speed) {
                    target_speed = current_speed + ACCELERATION_CM_S2 * dt;
                    if (target_speed > max_speed) target_speed = max_speed;
                } else {
                    target_speed = max_speed;
                }
            } 
            else {
                // Deceleration ก่อนถึงจุดหยุด
                target_speed = MIN_SPEED + (max_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);
                if (target_speed < MIN_SPEED) target_speed = MIN_SPEED;
            }
            current_speed = target_speed;
        }

        // ── PID Gyro แก้ทิศทาง ─────────────────────────────────
        float yaw = my.gyro('z') - yaw_offset;
        float err = yaw;

        _integral += err * dt;
        float deriv = (err - _prevErr) / dt;
        _prevErr = err;

        float corr = kp * err + 0.0001f * _integral + 0.08f * deriv;

        // สำคัญ: ใส่เครื่องหมายความเร็วตามทิศทาง
        int left_pwm  = (int)(target_speed * (spl_base >= 0 ? 1 : -1) - corr);
        int right_pwm = (int)(target_speed * (spr_base >= 0 ? 1 : -1) + corr);

        left_pwm  = constrain(left_pwm,  -100, 100);
        right_pwm = constrain(right_pwm, -100, 100);

        Motor(left_pwm, right_pwm);

        if (now - start_time > 30000UL) break;

        delayMicroseconds(20);
    }

    // ── Soft Stop (ปรับตามทิศทาง) ─────────────────────────────
    if (offset > 0) {
        if (is_forward) {
            Motor(-5, -5); delay(offset);   // เดินหน้า → เบรกถอยหลัง
        } else {
            Motor(5, 5); delay(offset);     // เดินถอย → เบรกเดินหน้า
        }
        Motor(-1, -1); delay(80);
    } 
    else {
        Motor(0, 0);   // ไม่มี offset
    }
    
    Motor(0, 0);
    delay(5);
}