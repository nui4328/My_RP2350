#include "MoveGyroPID.h"
// ============================================================
// ARM UP / DOWN CONTROL
// ใช้ตัวแปร Global เดิมจาก MoveGyroPID.h
// ============================================================


// ============================================================
// ค่าตั้งระบบแขน
// ============================================================

#define ARM_CTRL_MAX_HEIGHT_CM          27.0f

#define ARM_CTRL_HOME_ADC_THRESHOLD     1500
#define ARM_CTRL_HOMING_DOWN_SPEED      35
#define ARM_CTRL_HOMING_BACKUP_SPEED    18
#define ARM_CTRL_BACKUP_PULSES          20

#define ARM_CTRL_TIMEOUT_MS             15000UL
#define ARM_CTRL_MOTOR_UPDATE_MS        6UL

// ระยะคลาดเคลื่อนที่ยอมรับได้
#define ARM_CTRL_TOLERANCE_CM           0.10f

// ชดเชยการหยุดตามทิศทาง
// ขาขึ้นแขนเลยเป้าหมายประมาณ +0.30 cm จึงหยุดก่อน
#define ARM_CTRL_UP_TARGET_OFFSET_CM    -1.28f

// ขาลงแขนเลยเป้าหมายประมาณ -0.25 cm จึงหยุดก่อนในทิศลง
#define ARM_CTRL_DOWN_TARGET_OFFSET_CM   3.55f

// ระยะเริ่มลดความเร็ว
#define ARM_CTRL_SLOW_DISTANCE_1_CM      0.30f
#define ARM_CTRL_SLOW_DISTANCE_2_CM      0.80f
#define ARM_CTRL_SLOW_DISTANCE_3_CM      1.60f

// ความเร็วขาขึ้น
#define ARM_CTRL_UP_FAST                 40
#define ARM_CTRL_UP_MEDIUM               34
#define ARM_CTRL_UP_SLOW                 29
#define ARM_CTRL_UP_MIN                  25

// ความเร็วขาลง
#define ARM_CTRL_DOWN_FAST               28
#define ARM_CTRL_DOWN_MEDIUM             25
#define ARM_CTRL_DOWN_SLOW               18
#define ARM_CTRL_DOWN_MIN                15

// คำสั่งตำแหน่งเหมือนครั้งก่อน ไม่เคลื่อนซ้ำ
#define ARM_CTRL_SAME_COMMAND_CM          0.01f


// ============================================================
// ตัวแปรเพิ่มเติมเฉพาะไฟล์นี้
// ชื่อไม่ซ้ำกับ MoveGyroPID.h
// ============================================================

// ตำแหน่งจริงที่ผู้ใช้สั่ง ก่อนชดเชย
static float arm_requested_target_cm = 0.0f;

// ตำแหน่งเป้าหมายหลังชดเชย
static float arm_compensated_target_cm = 0.0f;


// ============================================================
// Function Prototype
// ============================================================

void arm_updown(float target_cm);
void arm_updown_nonblocking(float target_cm);
void arm_update_nonblocking();

long armCmToPulses(float cm);
float armPulsesToCm(long pulses);

void armStopMotor();
void armResetControl();


// ============================================================
// แปลงเซนติเมตรเป็นพัลส์
// ============================================================

long armCmToPulses(float cm)
{
    return lroundf(cm * PULSES_PER_CM);
}


// ============================================================
// แปลงพัลส์เป็นเซนติเมตร
// ============================================================

float armPulsesToCm(long pulses)
{
    if (PULSES_PER_CM <= 0.0f)
    {
        return 0.0f;
    }

    return (float)pulses / PULSES_PER_CM;
}


// ============================================================
// หยุดมอเตอร์แขน
// ============================================================

void armStopMotor()
{
    robot.motor('D', 0);
}


// ============================================================
// รีเซ็ตสถานะควบคุม
// ============================================================

void armResetControl()
{
    armStopMotor();

    arm_moving = false;
    arm_homing_mode = false;
    arm_direction = 0;

    // enum เดิมมีเฉพาะ HOMING_DOWN และ HOMING_BACKUP
    homing_state = HOMING_DOWN;
}


// ============================================================
// ฟังก์ชัน Blocking
// ============================================================

void arm_updown(float target_cm)
{
    target_cm = constrain(
        target_cm,
        0.0f,
        ARM_CTRL_MAX_HEIGHT_CM
    );

    last_PULSES_up = (float)encoder2.Poss_R();

    arm_updown_nonblocking(target_cm);

    unsigned long start_time = millis();

    // ตอนกลับ Home
    if (target_cm <= 0.01f)
    {
        arm_shoulder('L', arm_shoulderL_begin);  // ค่าเดิมคือ65
       arm_shoulder('R', arm_shoulderR_begin);
    }

    while (arm_moving)
    {
        arm_update_nonblocking();

        if (millis() - start_time >= ARM_CTRL_TIMEOUT_MS)
        {
            Serial.println("ARM TIMEOUT!");

            armResetControl();
            return;
        }

        delay(5);
    }
}


// ============================================================
// เริ่มเคลื่อนที่แบบ Non-blocking
// ============================================================

void arm_updown_nonblocking(float target_cm)
{
    target_cm = constrain(
        target_cm,
        0.0f,
        ARM_CTRL_MAX_HEIGHT_CM
    );

    // แขนกำลังทำงาน
    if (arm_moving)
    {
        Serial.println("ARM BUSY!");
        return;
    }

    // --------------------------------------------------------
    // คำสั่งเหมือนครั้งก่อน ไม่ต้องเคลื่อน
    //
    // arm_updown(5);
    // sw();
    // arm_updown(5);
    //
    // ครั้งที่สองไม่เคลื่อน
    // --------------------------------------------------------

    if (
        last_arm_command > -900.0f &&
        fabs(target_cm - last_arm_command) <=
        ARM_CTRL_SAME_COMMAND_CM
    )
    {
        Serial.printf(
            "Arm same command %.2f cm - NO MOVE\n",
            target_cm
        );

        return;
    }

    // ========================================================
    // HOMING
    // ========================================================

    if (target_cm <= 0.01f)
    {
        last_arm_command = 0.0f;

        arm_requested_target_cm = 0.0f;
        arm_compensated_target_cm = 0.0f;

        arm_homing_mode = true;
        arm_moving = true;
        arm_direction = -1;

        homing_state = HOMING_DOWN;

        arm_last_time = millis();
        arm_last_move_time = millis();
        arm_last_pulses_stuck = encoder2.Poss_R();

        robot.motor(
            'D',
            -ARM_CTRL_HOMING_DOWN_SPEED
        );

        Serial.println(
            "=== ARM HOMING STARTED ==="
        );

        return;
    }

    // ========================================================
    // อ่านตำแหน่งปัจจุบัน
    // ========================================================

    long current_pulses =
        encoder2.Poss_R();

    current_position_cm =
        armPulsesToCm(current_pulses);

    // เป้าหมายจริงที่ผู้ใช้สั่ง
    long raw_target_pulses =
        armCmToPulses(target_cm);

    long raw_error_pulses =
        raw_target_pulses -
        current_pulses;

    long tolerance_pulses =
        armCmToPulses(
            ARM_CTRL_TOLERANCE_CM
        );

    if (tolerance_pulses < 2)
    {
        tolerance_pulses = 2;
    }

    // อยู่ใกล้ตำแหน่งจริงที่สั่งแล้ว
    if (
        labs(raw_error_pulses) <=
        tolerance_pulses
    )
    {
        armStopMotor();

        last_arm_command = target_cm;

        Serial.printf(
            "Arm already near target | "
            "Target %.2f cm | Current %.2f cm\n",
            target_cm,
            current_position_cm
        );

        return;
    }

    // ========================================================
    // ตรวจสอบทิศทาง
    // ========================================================

    bool going_up =
        raw_error_pulses > 0;

    // ========================================================
    // ชดเชยเป้าหมายตามทิศทาง
    // ========================================================

    float compensated_target_cm =
        target_cm;

    if (going_up)
    {
        compensated_target_cm +=
            ARM_CTRL_UP_TARGET_OFFSET_CM;
    }
    else
    {
        compensated_target_cm +=
            ARM_CTRL_DOWN_TARGET_OFFSET_CM;
    }

    compensated_target_cm = constrain(
        compensated_target_cm,
        0.0f,
        ARM_CTRL_MAX_HEIGHT_CM
    );

    long compensated_target_pulses =
        armCmToPulses(
            compensated_target_cm
        );

    long compensated_error_pulses =
        compensated_target_pulses -
        current_pulses;

    // หากชดเชยแล้วเป้าหมายอยู่ใกล้ตำแหน่งปัจจุบันเกินไป
    if (
        labs(compensated_error_pulses) <=
        tolerance_pulses
    )
    {
        armStopMotor();

        last_arm_command = target_cm;

        Serial.printf(
            "Arm compensated target near current | "
            "Command %.2f cm | Current %.2f cm\n",
            target_cm,
            current_position_cm
        );

        return;
    }

    // ========================================================
    // เริ่มเคลื่อนที่
    // ========================================================

    arm_requested_target_cm =
        target_cm;

    arm_compensated_target_cm =
        compensated_target_cm;

    arm_goal_pulses =
        compensated_target_pulses;

    arm_direction =
        going_up ? 1 : -1;

    arm_moving = true;
    arm_homing_mode = false;

    // จำคำสั่งล่าสุด
    last_arm_command =
        target_cm;

    arm_last_time =
        millis();

    arm_last_move_time =
        millis();

    arm_last_pulses_stuck =
        current_pulses;

    int start_power;

    if (going_up)
    {
        start_power =
            ARM_CTRL_UP_FAST;
    }
    else
    {
        start_power =
            ARM_CTRL_DOWN_FAST;
    }

    robot.motor(
        'D',
        arm_direction * start_power
    );

    Serial.printf(
        "Arm Command %.2f cm | "
        "Compensated %.2f cm | "
        "Current %.2f cm | "
        "TargetPulse %ld | "
        "CurrentPulse %ld | "
        "ErrorPulse %ld | "
        "Dir %s\n",

        target_cm,
        compensated_target_cm,
        current_position_cm,
        compensated_target_pulses,
        current_pulses,
        compensated_error_pulses,
        going_up ? "UP" : "DOWN"
    );
}


// ============================================================
// อัปเดตการเคลื่อนที่
// ============================================================

void arm_update_nonblocking()
{
    if (!arm_moving)
    {
        return;
    }

    unsigned long now =
        millis();

    int adc_value =
        robot.adcRead(6);

    long current_pulses =
        encoder2.Poss_R();

    current_position_cm =
        armPulsesToCm(current_pulses);

    // ========================================================
    // HOMING
    // ========================================================

    if (arm_homing_mode)
    {
        switch (homing_state)
        {
            // ------------------------------------------------
            // เลื่อนลงหา Home
            // ------------------------------------------------

            case HOMING_DOWN:
            {
                if (
                    adc_value >=
                    ARM_CTRL_HOME_ADC_THRESHOLD
                )
                {
                    armStopMotor();

                    delay(20);

                    encoder2.resetEncoders();

                    homing_backup_start =
                        (unsigned long)
                        encoder2.Poss_R();

                    arm_last_time =
                        millis();

                    arm_last_move_time =
                        millis();

                    arm_last_pulses_stuck =
                        encoder2.Poss_R();

                    homing_state =
                        HOMING_BACKUP;

                    arm_direction = 1;

                    robot.motor(
                        'D',
                        ARM_CTRL_HOMING_BACKUP_SPEED
                    );

                    Serial.println(
                        "HOME SENSOR DETECTED - BACKUP"
                    );
                }
                else if (
                    now - arm_last_time >= 8UL
                )
                {
                    robot.motor(
                        'D',
                        -ARM_CTRL_HOMING_DOWN_SPEED
                    );

                    arm_last_time = now;
                }

                break;
            }

            // ------------------------------------------------
            // ยกขึ้นออกจากเซนเซอร์ Home
            // ------------------------------------------------

            case HOMING_BACKUP:
            {
                current_pulses =
                    encoder2.Poss_R();

                long backup_pulses =
                    current_pulses -
                    (long)homing_backup_start;

                if (
                    backup_pulses >=
                    ARM_CTRL_BACKUP_PULSES
                )
                {
                    armStopMotor();

                    delay(20);

                    encoder2.resetEncoders();

                    current_position_cm =
                        0.0f;

                    arm_moving = false;
                    arm_homing_mode = false;
                    arm_direction = 0;

                    homing_state =
                        HOMING_DOWN;

                    last_arm_command =
                        0.0f;

                    arm_requested_target_cm =
                        0.0f;

                    arm_compensated_target_cm =
                        0.0f;

                    Serial.println(
                        "=== HOMING COMPLETED - Arm at 0 cm ==="
                    );
                }
                else if (
                    now - arm_last_time >= 10UL
                )
                {
                    robot.motor(
                        'D',
                        ARM_CTRL_HOMING_BACKUP_SPEED
                    );

                    arm_last_time = now;
                }

                break;
            }

            default:
            {
                Serial.println(
                    "ARM HOMING STATE ERROR!"
                );

                armResetControl();
                return;
            }
        }

        return;
    }

    // ========================================================
    // ตรวจสอบมอเตอร์ติดขัด
    // ========================================================

    long moved_pulses =
        labs(
            current_pulses -
            arm_last_pulses_stuck
        );

    if (
        moved_pulses >=
        ARM_MIN_PULSES
    )
    {
        arm_last_move_time =
            now;

        arm_last_pulses_stuck =
            current_pulses;
    }
    else if (
        now - arm_last_move_time >=
        ARM_STUCK_TIME_MS
    )
    {
        Serial.printf(
            "ARM STUCK! Position %.2f cm | Pulse %ld\n",
            current_position_cm,
            current_pulses
        );

        armResetControl();
        return;
    }

    // ========================================================
    // คำนวณ Error จากเป้าหมายหลังชดเชย
    // ========================================================

    long error_pulses =
        arm_goal_pulses -
        current_pulses;

    long remain_pulses =
        labs(error_pulses);

    long tolerance_pulses =
        armCmToPulses(
            ARM_CTRL_TOLERANCE_CM
        );

    if (tolerance_pulses < 2)
    {
        tolerance_pulses = 2;
    }

    // ========================================================
    // ถึงเป้าหมายหลังชดเชย
    // ========================================================

    if (
        remain_pulses <=
        tolerance_pulses
    )
    {
        armStopMotor();

        // รอแรงเฉื่อยของกลไก
        delay(25);

        long final_pulses =
            encoder2.Poss_R();

        current_position_cm =
            armPulsesToCm(
                final_pulses
            );

        // Error เทียบกับตำแหน่งที่ผู้ใช้สั่งจริง
        float final_error_cm =
            current_position_cm -
            arm_requested_target_cm;

        arm_moving = false;
        arm_homing_mode = false;
        arm_direction = 0;

        Serial.printf(
            "Arm ARRIVED | "
            "Command %.2f cm | "
            "Compensated %.2f cm | "
            "Actual %.2f cm | "
            "Error %.2f cm | "
            "Pulse %ld\n",

            arm_requested_target_cm,
            arm_compensated_target_cm,
            current_position_cm,
            final_error_cm,
            final_pulses
        );

        return;
    }

    // ========================================================
    // ตรวจสอบว่าเลยเป้าหมายหลังชดเชยหรือไม่
    // ========================================================

    int required_direction =
        (error_pulses > 0)
            ? 1
            : -1;

    if (
        required_direction !=
        arm_direction
    )
    {
        armStopMotor();

        delay(10);

        arm_direction =
            required_direction;

        arm_last_time =
            millis();

        arm_last_move_time =
            millis();

        arm_last_pulses_stuck =
            encoder2.Poss_R();

        Serial.println(
            "ARM OVERSHOOT - CORRECTING"
        );
    }

    // ========================================================
    // ควบคุมความเร็วตามระยะที่เหลือ
    // ========================================================

    float remain_cm =
        armPulsesToCm(
            remain_pulses
        );

    int power = 0;

    // --------------------------------------------------------
    // ขาขึ้น
    // --------------------------------------------------------

    if (arm_direction > 0)
    {
        if (
            remain_cm <=
            ARM_CTRL_SLOW_DISTANCE_1_CM
        )
        {
            power =
                ARM_CTRL_UP_MIN;
        }
        else if (
            remain_cm <=
            ARM_CTRL_SLOW_DISTANCE_2_CM
        )
        {
            power =
                ARM_CTRL_UP_SLOW;
        }
        else if (
            remain_cm <=
            ARM_CTRL_SLOW_DISTANCE_3_CM
        )
        {
            power =
                ARM_CTRL_UP_MEDIUM;
        }
        else
        {
            power =
                ARM_CTRL_UP_FAST;
        }
    }

    // --------------------------------------------------------
    // ขาลง
    // --------------------------------------------------------

    else
    {
        if (
            remain_cm <=
            ARM_CTRL_SLOW_DISTANCE_1_CM
        )
        {
            power =
                ARM_CTRL_DOWN_MIN;
        }
        else if (
            remain_cm <=
            ARM_CTRL_SLOW_DISTANCE_2_CM
        )
        {
            power =
                ARM_CTRL_DOWN_SLOW;
        }
        else if (
            remain_cm <=
            ARM_CTRL_SLOW_DISTANCE_3_CM
        )
        {
            power =
                ARM_CTRL_DOWN_MEDIUM;
        }
        else
        {
            power =
                ARM_CTRL_DOWN_FAST;
        }
    }

    // ========================================================
    // ส่งคำสั่งมอเตอร์
    // ========================================================

    if (
        now - arm_last_time >=
        ARM_CTRL_MOTOR_UPDATE_MS
    )
    {
        robot.motor(
            'D',
            arm_direction * power
        );

        arm_last_time = now;
    }
}

// =============================
// MOVE FUNCTION
// =============================
void move(
    int sl,             // ความเร็วมอเตอร์ซ้าย
    int sr,             // ความเร็วมอเตอร์ขวา
    float kp,           // Kp ควบคุมทิศทาง
    float dist,         // ระยะทางเป้าหมาย หน่วย cm
    int offset,         // เวลา Brake pulse หน่วย ms, 0 = ไม่เบรก
    float arm_target,   // ตำแหน่งแขนเป้าหมาย
    float deg           // มุมเป้าหมายจริง
)
{
    extern PiperPico2 robot;
    extern EncoderLibrarys encoder1;
    extern EncoderLibraryss encoder2;

    //---------------- ตรวจสอบระยะทาง ----------------

    if (dist <= 0.1f)
    {
        robot.motor('A', 0);
        robot.motor('C', 0);
        return;
    }

    //---------------- ปรับระยะตามสถานะ ----------------

    /*
       offset == 0:
       ลดระยะลง 5 cm ตามพฤติกรรมโค้ดเดิม
    */
    if (offset == 0)
    {
        dist -= 5.0f;
    }

    /*
       กรณีมีการถอยหรือเดินหน้าชดเชยก่อนหน้านี้
    */
    if (_set_b || _set_f)
    {
        dist += 5.0f;
    }

    if (dist <= 0.1f)
    {
        robot.motor('A', 0);
        robot.motor('C', 0);
        return;
    }

    //---------------- ปรับแขนไหล่ ----------------

    if (arm_target < 3.0f)
    {
        arm_shoulder('L', 76);
        arm_shoulder('R', 62);
    }

    //---------------- มุมเป้าหมาย ----------------

    /*
       ใช้มุมเป้าหมายที่สั่งจริง ไม่ใช่มุมที่หมุนได้จริง

       ตัวอย่าง:
       turnGyro(70, -90);

       แม้หมุนจริงได้ -87°
       degree_turnGyro ยังเป็นประมาณ -90°
       รถจะค่อย ๆ แก้ทิศไปยัง -90° ระหว่างวิ่ง
    */
    const float targetAngle =
        constrainAngle180(deg);

    //---------------- ความเร็วมอเตอร์ฐาน ----------------

    const int leftBase =
        constrain(sl, -100, 100);

    const int rightBase =
        constrain(sr, -100, 100);

    if (leftBase == 0 && rightBase == 0)
    {
        robot.motor('A', 0);
        robot.motor('C', 0);
        return;
    }

    const bool movingBackward =
        leftBase < 0 &&
        rightBase < 0;

    //---------------- Encoder ----------------

    encoder1.resetEncoders();

    const long startPulses =
        encoder1.Poss_L();

    const float pulsesPerCmUsed =
        movingBackward
        ? pulsesPerCm_bw
        : pulsesPerCm_fw;

    if (pulsesPerCmUsed <= 0.0f)
    {
        Serial.println("MOVE ERROR: pulsesPerCmUsed <= 0");

        robot.motor('A', 0);
        robot.motor('C', 0);
        return;
    }

    //---------------- เริ่มขยับแขน ----------------

    arm_updown_nonblocking(arm_target);

    //---------------- PID variables ----------------

    float integral = 0.0f;
    float lastError = 0.0f;
    float dFilter = 0.0f;

    unsigned long lastTime = micros();
    unsigned long startTime = millis();

    //---------------- ค่าควบคุม ----------------

    const float KD_FILTER = 0.75f;

    const int MIN_SPEED = 32;

    const float averageBaseSpeed =
        (
            fabs((float)leftBase) +
            fabs((float)rightBase)
        ) * 0.5f;

    // ระยะเร่งความเร็ว
    const float ACC_DISTANCE = 15.0f;

    // ระยะเริ่มชะลอ
    const float SLOW_DISTANCE =
        20.0f +
        averageBaseSpeed * 0.40f;

    // ระยะช่วงเข้าจอด
    const float FINAL_APPROACH_DISTANCE = 9.0f;

    // ระยะที่ถือว่าถึงจุดเป้าหมาย
    const float STOP_DISTANCE = 4.0f;

    // จำกัดแรงแก้มุม
    const float MAX_CORRECTION = 30.0f;

    /*
       Timeout ตามระยะทาง

       ตัวอย่าง 100 cm:
       4000 + 100 × 100 = 14000 ms
    */
    unsigned long timeout =
        4000UL +
        (unsigned long)(dist * 100.0f);

    timeout =
        constrain(timeout, 6000UL, 30000UL);

    //---------------- สถานะ ----------------

    bool wasDecelerating = false;
    bool reachedTarget = false;
    bool timeoutOccurred = false;

    float traveled = 0.0f;
    float remain = dist;

    //---------------- Debug เริ่มต้น ----------------

    imu.update();
    //---------------- Main loop ----------------
    while (true)
    {
        //---------------- อัปเดตแขน ----------------

        arm_update_nonblocking();

        //---------------- คำนวณระยะทาง ----------------

        const long currentPulses =
            encoder1.Poss_L();

        const long pulseDifference =
            labs(currentPulses - startPulses);

        traveled =
            pulseDifference /
            pulsesPerCmUsed;

        remain =
            dist - traveled;

        //---------------- ตรวจถึงเป้าหมาย ----------------

        if (remain <= STOP_DISTANCE)
        {
            reachedTarget = true;
            break;
        }

        //---------------- อ่านมุม IMU ----------------

        imu.update();

        const float currentAngle =
            imu.yaw();

        //---------------- Speed profile ----------------

        int baseLeft = leftBase;
        int baseRight = rightBase;

        /*
           ช่วงเร่งความเร็ว
        */
        if (traveled < ACC_DISTANCE)
        {
            float progress =
                traveled / ACC_DISTANCE;

            progress =
                constrain(progress, 0.0f, 1.0f);

            float leftMagnitude =
                MIN_SPEED +
                (
                    fabs((float)leftBase) -
                    MIN_SPEED
                ) * progress;

            float rightMagnitude =
                MIN_SPEED +
                (
                    fabs((float)rightBase) -
                    MIN_SPEED
                ) * progress;

            leftMagnitude =
                max(leftMagnitude, (float)MIN_SPEED);

            rightMagnitude =
                max(rightMagnitude, (float)MIN_SPEED);

            baseLeft =
                (int)roundf(leftMagnitude);

            baseRight =
                (int)roundf(rightMagnitude);

            if (leftBase < 0)
                baseLeft = -baseLeft;

            if (rightBase < 0)
                baseRight = -baseRight;
        }
        /*
           ช่วงชะลอความเร็ว
        */
        else if (remain < SLOW_DISTANCE)
        {
            float progress =
                remain / SLOW_DISTANCE;

            progress =
                constrain(progress, 0.0f, 1.0f);

            /*
               progress = 1 อยู่ต้นช่วงชะลอ
               progress = 0 ใกล้จุดหยุด
            */
            const float slowFactor =
                0.35f +
                0.65f * progress;

            float leftMagnitude =
                MIN_SPEED +
                (
                    fabs((float)leftBase) -
                    MIN_SPEED
                ) * slowFactor;

            float rightMagnitude =
                MIN_SPEED +
                (
                    fabs((float)rightBase) -
                    MIN_SPEED
                ) * slowFactor;

            leftMagnitude =
                max(leftMagnitude, (float)MIN_SPEED);

            rightMagnitude =
                max(rightMagnitude, (float)MIN_SPEED);

            baseLeft =
                (int)roundf(leftMagnitude);

            baseRight =
                (int)roundf(rightMagnitude);

            if (leftBase < 0)
                baseLeft = -baseLeft;

            if (rightBase < 0)
                baseRight = -baseRight;
        }

        /*
           ช่วง 4–9 cm สุดท้าย ลดความเร็วเพิ่ม
        */
        if (remain <= FINAL_APPROACH_DISTANCE)
        {
            const float finalFactor = 0.48f;

            int finalLeft =
                (int)roundf(
                    fabs((float)leftBase) *
                    finalFactor
                );

            int finalRight =
                (int)roundf(
                    fabs((float)rightBase) *
                    finalFactor
                );

            /*
               ป้องกันแรงต่ำเกินจนมอเตอร์ไม่หมุน
            */
            finalLeft =
                max(finalLeft, 18);

            finalRight =
                max(finalRight, 18);

            baseLeft =
                leftBase >= 0
                ? finalLeft
                : -finalLeft;

            baseRight =
                rightBase >= 0
                ? finalRight
                : -finalRight;
        }

        //---------------- Heading error ----------------

        float headingError =
            constrainAngle180(
                targetAngle - currentAngle
            );

        /*
           ถอยหลังต้องกลับทิศการชดเชย
        */
        const float controlError =
            movingBackward
            ? -headingError
            : headingError;

        //---------------- Delta time ----------------

        const unsigned long nowMicros =
            micros();

        float dt =
            (nowMicros - lastTime) *
            0.000001f;

        lastTime =
            nowMicros;

        dt =
            constrain(dt, 0.003f, 0.05f);

        //---------------- ตรวจช่วงชะลอ ----------------

        const bool isDecelerating =
            traveled >= ACC_DISTANCE &&
            remain < SLOW_DISTANCE;

        if (isDecelerating &&
            !wasDecelerating)
        {
            integral = 0.0f;
        }

        wasDecelerating =
            isDecelerating;

        //---------------- Integral ----------------

        /*
           ยังไม่ได้ใช้ Ki ใน correction
           แต่เก็บ Integral ไว้รองรับการเพิ่ม Ki ภายหลัง
        */
        if (fabs(controlError) < 15.0f)
        {
            integral +=
                controlError * dt;

            integral =
                constrain(
                    integral,
                    -40.0f,
                    40.0f
                );
        }
        else
        {
            integral = 0.0f;
        }

        //---------------- Derivative ----------------

        const float derivative =
            (controlError - lastError) /
            dt;

        dFilter =
            KD_FILTER * dFilter +
            (1.0f - KD_FILTER) *
            derivative;

        lastError =
            controlError;

        //---------------- PID correction ----------------

        float correction =
            kp * controlError +
            move_kd * dFilter;

        /*
           ลดแรงแก้มุมเมื่อความเร็วฐานต่ำ
        */
        const float currentAverageSpeed =
            (
                fabs((float)baseLeft) +
                fabs((float)baseRight)
            ) * 0.5f;

        const float speedFactor =
            constrain(
                currentAverageSpeed / 40.0f,
                0.33f,
                1.0f
            );

        correction *=
            speedFactor;

        correction =
            constrain(
                correction,
                -MAX_CORRECTION,
                MAX_CORRECTION
            );

        //---------------- คำนวณมอเตอร์ ----------------

        int leftMotor;
        int rightMotor;

        if (movingBackward)
        {
            leftMotor =
                baseLeft -
                (int)roundf(correction);

            rightMotor =
                baseRight +
                (int)roundf(correction);
        }
        else
        {
            leftMotor =
                baseLeft +
                (int)roundf(correction);

            rightMotor =
                baseRight -
                (int)roundf(correction);
        }

        leftMotor =
            constrain(leftMotor, -100, 100);

        rightMotor =
            constrain(rightMotor, -100, 100);

        //---------------- อ่าน Line Sensor ----------------

        /*
           อ่านเพียงครั้งเดียวต่อรอบ
           ลดเวลาการอ่าน ADC และป้องกันค่าต่างกัน
           ภายในเงื่อนไขเดียวกัน
        */
        bool lineCorrectionActive = false;

        int lineLeftMotor = leftMotor;
        int lineRightMotor = rightMotor;

        if (leftBase > 0 && rightBase > 0)
        {
            // เดินหน้า ใช้ Sensor 8 และ 1

            const int sensorLeft =
                robot.adcRead(8);

            const int sensorRight =
                robot.adcRead(1);

            const int thresholdLeft =
                robot.adcMD(8);

            const int thresholdRight =
                robot.adcMD(1);

            const bool leftOnLine =
                sensorLeft < thresholdLeft;

            const bool rightOnLine =
                sensorRight < thresholdRight;

            if (leftOnLine && !rightOnLine)
            {
                lineLeftMotor =
                    (int)roundf(leftMotor * 0.70f);

                lineRightMotor =
                    (int)roundf(rightMotor * 1.25f);

                lineCorrectionActive = true;
            }
            else if (!leftOnLine && rightOnLine)
            {
                lineLeftMotor =
                    (int)roundf(leftMotor * 1.25f);

                lineRightMotor =
                    (int)roundf(rightMotor * 0.70f);

                lineCorrectionActive = true;
            }
        }
        else if (leftBase < 0 && rightBase < 0)
        {
            // ถอยหลัง ใช้ Sensor 9 และ 0

            const int sensorLeft =
                robot.adcRead(9);

            const int sensorRight =
                robot.adcRead(0);

            const int thresholdLeft =
                robot.adcMD(9);

            const int thresholdRight =
                robot.adcMD(0);

            const bool leftOnLine =
                sensorLeft < thresholdLeft;

            const bool rightOnLine =
                sensorRight < thresholdRight;

            if (leftOnLine && !rightOnLine)
            {
                lineLeftMotor =
                    (int)roundf(leftMotor * 0.70f);

                lineRightMotor =
                    (int)roundf(rightMotor * 1.25f);

                lineCorrectionActive = true;
            }
            else if (!leftOnLine && rightOnLine)
            {
                lineLeftMotor =
                    (int)roundf(leftMotor * 1.25f);

                lineRightMotor =
                    (int)roundf(rightMotor * 0.70f);

                lineCorrectionActive = true;
            }
        }

        lineLeftMotor =
            constrain(lineLeftMotor, -100, 100);

        lineRightMotor =
            constrain(lineRightMotor, -100, 100);

        //---------------- ส่งคำสั่งมอเตอร์ ----------------

        if (lineCorrectionActive)
        {
            /*
               ไม่รีเซต Yaw ตรงนี้

               หลังออกจากเส้น ระบบ PID จะนำรถ
               กลับไปยัง targetAngle เดิมโดยอัตโนมัติ
            */
            robot.motor('A', lineLeftMotor);
            robot.motor('C', lineRightMotor);
        }
        else
        {
            robot.motor('A', leftMotor);
            robot.motor('C', rightMotor);
        }

        //---------------- Safety timeout ----------------

        if (millis() - startTime > timeout)
        {
            timeoutOccurred = true;

            Serial.println("MOVE TIMEOUT");
            break;
        }

        delay(4);
    }

    //---------------- Brake ----------------

    /*
       offset > 0 จึงใช้ Brake pulse

       offset คือระยะเวลาเบรก หน่วย ms
    */
    if (offset > 0 &&
        break_move == 1)
    {
        const float BRAKE_FACTOR = 0.12f;

        int brakeLeft =
            (int)roundf(
                -leftBase *
                BRAKE_FACTOR
            );

        int brakeRight =
            (int)roundf(
                -rightBase *
                BRAKE_FACTOR
            );

        brakeLeft =
            constrain(brakeLeft, -15, 15);

        brakeRight =
            constrain(brakeRight, -15, 15);

        robot.motor('A', brakeLeft);
        robot.motor('C', brakeRight);

        delay(
            constrain(
                offset,
                1,
                20
            )
        );
    }

    //---------------- หยุดมอเตอร์ ----------------

    robot.motor('A', 0);
    robot.motor('C', 0);
    robot.motor('D', 0);

    //---------------- อัปเดตแขนครั้งสุดท้าย ----------------

    arm_update_nonblocking();

    //---------------- Reset states ----------------

    arm_moving = false;
    arm_homing_mode = false;

    _set_f = false;
    _set_b = false;
    _offset = false;

    break_move = 1;

    //---------------- แสดงผลลัพธ์ ----------------

    imu.update();

    const float finalAngle =
        imu.yaw();

    const float finalHeadingError =
        constrainAngle180(
            targetAngle - finalAngle
        );

    // Serial.print("MOVE FINISHED");

    // Serial.print(" | Target angle = ");
    // Serial.print(targetAngle, 2);

    // Serial.print(" | Final angle = ");
    // Serial.print(finalAngle, 2);

    // Serial.print(" | Heading error = ");
    // Serial.print(finalHeadingError, 2);

    // Serial.print(" | Traveled = ");
    // Serial.print(traveled, 1);

    // Serial.print(" cm | Target distance = ");
    // Serial.print(dist, 1);

    // Serial.print(" cm | Remaining = ");
    // Serial.print(remain, 1);

    // Serial.print(" cm | Status = ");

    // if (reachedTarget)
    // {
    //     Serial.println("ARRIVED");
    // }
    // else if (timeoutOccurred)
    // {
    //     Serial.println("TIMEOUT");
    // }
    // else
    // {
    //     Serial.println("STOPPED");
    // }
}


void move_bridge(int sl,             // ความเร็วมอเตอร์ซ้าย
    int sr,             // ความเร็วมอเตอร์ขวา
    float kp,           // Kp ควบคุมทิศทาง
    float dist,         // ระยะทางเป้าหมาย หน่วย cm
    int offset,         // เวลา Brake pulse หน่วย ms, 0 = ไม่เบรก
    float arm_target,   // ตำแหน่งแขนเป้าหมาย
    float deg           // มุมเป้าหมายจริง
)
{
    extern PiperPico2 robot;
    extern EncoderLibrarys encoder1;
    extern EncoderLibraryss encoder2;

    //---------------- ตรวจสอบระยะทาง ----------------

    if (dist <= 0.1f)
    {
        robot.motor('A', 0);
        robot.motor('C', 0);
        return;
    }

    //---------------- ปรับระยะตามสถานะ ----------------

    /*
       offset == 0:
       ลดระยะลง 5 cm ตามพฤติกรรมโค้ดเดิม
    */
    if (offset == 0)
    {
        dist -= 5.0f;
    }

    /*
       กรณีมีการถอยหรือเดินหน้าชดเชยก่อนหน้านี้
    */
    if (_set_b || _set_f)
    {
        dist += 5.0f;
    }

    if (dist <= 0.1f)
    {
        robot.motor('A', 0);
        robot.motor('C', 0);
        return;
    }

    //---------------- ปรับแขนไหล่ ----------------

    if (arm_target < 3.0f)
    {
        arm_shoulder('L', 76);
        arm_shoulder('R', 62);
    }

    //---------------- มุมเป้าหมาย ----------------

    /*
       ใช้มุมเป้าหมายที่สั่งจริง ไม่ใช่มุมที่หมุนได้จริง

       ตัวอย่าง:
       turnGyro(70, -90);

       แม้หมุนจริงได้ -87°
       degree_turnGyro ยังเป็นประมาณ -90°
       รถจะค่อย ๆ แก้ทิศไปยัง -90° ระหว่างวิ่ง
    */
    const float targetAngle =
        constrainAngle180(deg);

    //---------------- ความเร็วมอเตอร์ฐาน ----------------

    const int leftBase =
        constrain(sl, -100, 100);

    const int rightBase =
        constrain(sr, -100, 100);

    if (leftBase == 0 && rightBase == 0)
    {
        robot.motor('A', 0);
        robot.motor('C', 0);
        return;
    }

    const bool movingBackward =
        leftBase < 0 &&
        rightBase < 0;

    //---------------- Encoder ----------------

    encoder1.resetEncoders();

    const long startPulses =
        encoder1.Poss_L();

    const float pulsesPerCmUsed =
        movingBackward
        ? pulsesPerCm_bw
        : pulsesPerCm_fw;

    if (pulsesPerCmUsed <= 0.0f)
    {
        Serial.println("MOVE ERROR: pulsesPerCmUsed <= 0");

        robot.motor('A', 0);
        robot.motor('C', 0);
        return;
    }

    //---------------- เริ่มขยับแขน ----------------

    arm_updown_nonblocking(arm_target);

    //---------------- PID variables ----------------

    float integral = 0.0f;
    float lastError = 0.0f;
    float dFilter = 0.0f;

    unsigned long lastTime = micros();
    unsigned long startTime = millis();

    //---------------- ค่าควบคุม ----------------

    const float KD_FILTER = 0.75f;

    const int MIN_SPEED = 32;

    const float averageBaseSpeed =
        (
            fabs((float)leftBase) +
            fabs((float)rightBase)
        ) * 0.5f;

    // ระยะเร่งความเร็ว
    const float ACC_DISTANCE = 15.0f;

    // ระยะเริ่มชะลอ
    const float SLOW_DISTANCE =
        20.0f +
        averageBaseSpeed * 0.40f;

    // ระยะช่วงเข้าจอด
    const float FINAL_APPROACH_DISTANCE = 9.0f;

    // ระยะที่ถือว่าถึงจุดเป้าหมาย
    const float STOP_DISTANCE = 4.0f;

    // จำกัดแรงแก้มุม
    const float MAX_CORRECTION = 30.0f;

    /*
       Timeout ตามระยะทาง

       ตัวอย่าง 100 cm:
       4000 + 100 × 100 = 14000 ms
    */
    unsigned long timeout =
        4000UL +
        (unsigned long)(dist * 100.0f);

    timeout =
        constrain(timeout, 6000UL, 30000UL);

    //---------------- สถานะ ----------------

    bool wasDecelerating = false;
    bool reachedTarget = false;
    bool timeoutOccurred = false;

    float traveled = 0.0f;
    float remain = dist;

    //---------------- Debug เริ่มต้น ----------------

    imu.update();

    Serial.print("MOVE START");

    Serial.print(" | Target angle = ");
    Serial.print(targetAngle, 2);

    Serial.print(" | Current angle = ");
    Serial.print(imu.yaw(), 2);

    Serial.print(" | Distance = ");
    Serial.print(dist, 1);

    Serial.print(" | Offset = ");
    Serial.print(offset);

    Serial.print(" | Arm target = ");
    Serial.println(arm_target, 2);

    //---------------- Main loop ----------------

    while (true)
    {
        //---------------- อัปเดตแขน ----------------

        arm_update_nonblocking();

        //---------------- คำนวณระยะทาง ----------------

        const long currentPulses =
            encoder1.Poss_L();

        const long pulseDifference =
            labs(currentPulses - startPulses);

        traveled =
            pulseDifference /
            pulsesPerCmUsed;

        remain =
            dist - traveled;

        //---------------- ตรวจถึงเป้าหมาย ----------------

        if (remain <= STOP_DISTANCE)
        {
            reachedTarget = true;
            break;
        }

        //---------------- อ่านมุม IMU ----------------

        imu.update();

        const float currentAngle =
            imu.yaw();

        //---------------- Speed profile ----------------

        int baseLeft = leftBase;
        int baseRight = rightBase;

        /*
           ช่วงเร่งความเร็ว
        */
        if (traveled < ACC_DISTANCE)
        {
            float progress =
                traveled / ACC_DISTANCE;

            progress =
                constrain(progress, 0.0f, 1.0f);

            float leftMagnitude =
                MIN_SPEED +
                (
                    fabs((float)leftBase) -
                    MIN_SPEED
                ) * progress;

            float rightMagnitude =
                MIN_SPEED +
                (
                    fabs((float)rightBase) -
                    MIN_SPEED
                ) * progress;

            leftMagnitude =
                max(leftMagnitude, (float)MIN_SPEED);

            rightMagnitude =
                max(rightMagnitude, (float)MIN_SPEED);

            baseLeft =
                (int)roundf(leftMagnitude);

            baseRight =
                (int)roundf(rightMagnitude);

            if (leftBase < 0)
                baseLeft = -baseLeft;

            if (rightBase < 0)
                baseRight = -baseRight;
        }
        /*
           ช่วงชะลอความเร็ว
        */
        else if (remain < SLOW_DISTANCE)
        {
            float progress =
                remain / SLOW_DISTANCE;

            progress =
                constrain(progress, 0.0f, 1.0f);

            /*
               progress = 1 อยู่ต้นช่วงชะลอ
               progress = 0 ใกล้จุดหยุด
            */
            const float slowFactor =
                0.35f +
                0.65f * progress;

            float leftMagnitude =
                MIN_SPEED +
                (
                    fabs((float)leftBase) -
                    MIN_SPEED
                ) * slowFactor;

            float rightMagnitude =
                MIN_SPEED +
                (
                    fabs((float)rightBase) -
                    MIN_SPEED
                ) * slowFactor;

            leftMagnitude =
                max(leftMagnitude, (float)MIN_SPEED);

            rightMagnitude =
                max(rightMagnitude, (float)MIN_SPEED);

            baseLeft =
                (int)roundf(leftMagnitude);

            baseRight =
                (int)roundf(rightMagnitude);

            if (leftBase < 0)
                baseLeft = -baseLeft;

            if (rightBase < 0)
                baseRight = -baseRight;
        }

        /*
           ช่วง 4–9 cm สุดท้าย ลดความเร็วเพิ่ม
        */
        if (remain <= FINAL_APPROACH_DISTANCE)
        {
            const float finalFactor = 0.48f;

            int finalLeft =
                (int)roundf(
                    fabs((float)leftBase) *
                    finalFactor
                );

            int finalRight =
                (int)roundf(
                    fabs((float)rightBase) *
                    finalFactor
                );

            /*
               ป้องกันแรงต่ำเกินจนมอเตอร์ไม่หมุน
            */
            finalLeft =
                max(finalLeft, 18);

            finalRight =
                max(finalRight, 18);

            baseLeft =
                leftBase >= 0
                ? finalLeft
                : -finalLeft;

            baseRight =
                rightBase >= 0
                ? finalRight
                : -finalRight;
        }

        //---------------- Heading error ----------------

        float headingError =
            constrainAngle180(
                targetAngle - currentAngle
            );

        /*
           ถอยหลังต้องกลับทิศการชดเชย
        */
        const float controlError =
            movingBackward
            ? -headingError
            : headingError;

        //---------------- Delta time ----------------

        const unsigned long nowMicros =
            micros();

        float dt =
            (nowMicros - lastTime) *
            0.000001f;

        lastTime =
            nowMicros;

        dt =
            constrain(dt, 0.003f, 0.05f);

        //---------------- ตรวจช่วงชะลอ ----------------

        const bool isDecelerating =
            traveled >= ACC_DISTANCE &&
            remain < SLOW_DISTANCE;

        if (isDecelerating &&
            !wasDecelerating)
        {
            integral = 0.0f;
        }

        wasDecelerating =
            isDecelerating;

        //---------------- Integral ----------------

        /*
           ยังไม่ได้ใช้ Ki ใน correction
           แต่เก็บ Integral ไว้รองรับการเพิ่ม Ki ภายหลัง
        */
        if (fabs(controlError) < 15.0f)
        {
            integral +=
                controlError * dt;

            integral =
                constrain(
                    integral,
                    -40.0f,
                    40.0f
                );
        }
        else
        {
            integral = 0.0f;
        }

        //---------------- Derivative ----------------

        const float derivative =
            (controlError - lastError) /
            dt;

        dFilter =
            KD_FILTER * dFilter +
            (1.0f - KD_FILTER) *
            derivative;

        lastError =
            controlError;

        //---------------- PID correction ----------------

        float correction =
            kp * controlError +
            move_kd * dFilter;

        /*
           ลดแรงแก้มุมเมื่อความเร็วฐานต่ำ
        */
        const float currentAverageSpeed =
            (
                fabs((float)baseLeft) +
                fabs((float)baseRight)
            ) * 0.5f;

        const float speedFactor =
            constrain(
                currentAverageSpeed / 40.0f,
                0.33f,
                1.0f
            );

        correction *=
            speedFactor;

        correction =
            constrain(
                correction,
                -MAX_CORRECTION,
                MAX_CORRECTION
            );

        //---------------- คำนวณมอเตอร์ ----------------

        int leftMotor;
        int rightMotor;

        if (movingBackward)
        {
            leftMotor =
                baseLeft -
                (int)roundf(correction);

            rightMotor =
                baseRight +
                (int)roundf(correction);
        }
        else
        {
            leftMotor =
                baseLeft +
                (int)roundf(correction);

            rightMotor =
                baseRight -
                (int)roundf(correction);
        }

        leftMotor =
            constrain(leftMotor, -100, 100);

        rightMotor =
            constrain(rightMotor, -100, 100);

        //---------------- อ่าน Line Sensor ----------------

        /*
           อ่านเพียงครั้งเดียวต่อรอบ
           ลดเวลาการอ่าน ADC และป้องกันค่าต่างกัน
           ภายในเงื่อนไขเดียวกัน
        */
        bool lineCorrectionActive = false;

        int lineLeftMotor = leftMotor;
        int lineRightMotor = rightMotor;

        if (leftBase > 0 && rightBase > 0)
        {
            // เดินหน้า ใช้ Sensor 8 และ 1

            const int sensorLeft =
                robot.adcRead(8);

            const int sensorRight =
                robot.adcRead(1);

            const int thresholdLeft =
                robot.adcMD(8);

            const int thresholdRight =
                robot.adcMD(1);

            const bool leftOnLine =
                sensorLeft < thresholdLeft;

            const bool rightOnLine =
                sensorRight < thresholdRight;

            if (leftOnLine && !rightOnLine)
            {
                lineLeftMotor =
                    (int)roundf(leftMotor * 0.70f);

                lineRightMotor =
                    (int)roundf(rightMotor * 1.25f);

                lineCorrectionActive = true;
            }
            else if (!leftOnLine && rightOnLine)
            {
                lineLeftMotor =
                    (int)roundf(leftMotor * 1.25f);

                lineRightMotor =
                    (int)roundf(rightMotor * 0.70f);

                lineCorrectionActive = true;
            }
        }
        else if (leftBase < 0 && rightBase < 0)
        {
            // ถอยหลัง ใช้ Sensor 9 และ 0

            const int sensorLeft =
                robot.adcRead(9);

            const int sensorRight =
                robot.adcRead(0);

            const int thresholdLeft =
                robot.adcMD(9);

            const int thresholdRight =
                robot.adcMD(0);

            const bool leftOnLine =
                sensorLeft < thresholdLeft;

            const bool rightOnLine =
                sensorRight < thresholdRight;

            if (leftOnLine && !rightOnLine)
            {
                lineLeftMotor =
                    (int)roundf(leftMotor * 0.70f);

                lineRightMotor =
                    (int)roundf(rightMotor * 1.25f);

                lineCorrectionActive = true;
            }
            else if (!leftOnLine && rightOnLine)
            {
                lineLeftMotor =
                    (int)roundf(leftMotor * 1.25f);

                lineRightMotor =
                    (int)roundf(rightMotor * 0.70f);

                lineCorrectionActive = true;
            }
        }

        lineLeftMotor =
            constrain(lineLeftMotor, -100, 100);

        lineRightMotor =
            constrain(lineRightMotor, -100, 100);

        //---------------- ส่งคำสั่งมอเตอร์ ----------------

        if (lineCorrectionActive)
        {
            /*
               ไม่รีเซต Yaw ตรงนี้

               หลังออกจากเส้น ระบบ PID จะนำรถ
               กลับไปยัง targetAngle เดิมโดยอัตโนมัติ
            */
            robot.motor('A', lineLeftMotor);
            robot.motor('C', lineRightMotor);
        }
        else
        {
            robot.motor('A', leftMotor);
            robot.motor('C', rightMotor);
        }

        //---------------- Safety timeout ----------------

        if (millis() - startTime > timeout)
        {
            timeoutOccurred = true;

            Serial.println("MOVE TIMEOUT");
            break;
        }

        delay(4);
    }

    //---------------- Brake ----------------

    /*
       offset > 0 จึงใช้ Brake pulse

       offset คือระยะเวลาเบรก หน่วย ms
    */
    if (offset > 0 &&
        break_move == 1)
    {
        const float BRAKE_FACTOR = 0.12f;

        int brakeLeft =
            (int)roundf(
                -leftBase *
                BRAKE_FACTOR
            );

        int brakeRight =
            (int)roundf(
                -rightBase *
                BRAKE_FACTOR
            );

        brakeLeft =
            constrain(brakeLeft, -25, 25);

        brakeRight =
            constrain(brakeRight, -25, 25);

        robot.motor('A', brakeLeft);
        robot.motor('C', brakeRight);

        delay(
            constrain(
                offset,
                1,
                100
            )
        );
    }

    //---------------- หยุดมอเตอร์ ----------------

    robot.motor('A', 0);
    robot.motor('C', 0);
    robot.motor('D', 0);

    //---------------- อัปเดตแขนครั้งสุดท้าย ----------------

    arm_update_nonblocking();

    //---------------- Reset states ----------------

    arm_moving = false;
    arm_homing_mode = false;

    _set_f = false;
    _set_b = false;
    _offset = false;

    break_move = 1;

    //---------------- แสดงผลลัพธ์ ----------------

    imu.update();

    const float finalAngle =
        imu.yaw();

    const float finalHeadingError =
        constrainAngle180(
            targetAngle - finalAngle
        );

    Serial.print("MOVE FINISHED");

    Serial.print(" | Target angle = ");
    Serial.print(targetAngle, 2);

    Serial.print(" | Final angle = ");
    Serial.print(finalAngle, 2);

    Serial.print(" | Heading error = ");
    Serial.print(finalHeadingError, 2);

    Serial.print(" | Traveled = ");
    Serial.print(traveled, 1);

    Serial.print(" cm | Target distance = ");
    Serial.print(dist, 1);

    Serial.print(" cm | Remaining = ");
    Serial.print(remain, 1);

    Serial.print(" cm | Status = ");

    if (reachedTarget)
    {
        Serial.println("ARRIVED");
    }
    else if (timeoutOccurred)
    {
        Serial.println("TIMEOUT");
    }
    else
    {
        Serial.println("STOPPED");
    }
}


void move_chopsticks(
    int sl,             // ความเร็วมอเตอร์ซ้าย
    int sr,             // ความเร็วมอเตอร์ขวา
    float kp,           // Kp ควบคุมทิศทาง
    float dist,         // ระยะทางเป้าหมาย หน่วย cm
    int offset,         // เวลา Brake pulse หน่วย ms, 0 = ไม่เบรก
    float arm_target,   // ตำแหน่งแขนเป้าหมาย
    float deg           // มุมเป้าหมายจริง
)
{
    extern PiperPico2 robot;
    extern EncoderLibrarys encoder1;
    extern EncoderLibraryss encoder2;

    //---------------- ตรวจสอบระยะทาง ----------------

    if (dist <= 0.1f)
    {
        robot.motor('A', 0);
        robot.motor('C', 0);
        return;
    }

    //---------------- ปรับระยะตามสถานะ ----------------

    /*
       offset == 0:
       ลดระยะลง 5 cm ตามพฤติกรรมโค้ดเดิม
    */
    if (offset == 0)
    {
        dist -= 5.0f;
    }

    /*
       กรณีมีการถอยหรือเดินหน้าชดเชยก่อนหน้านี้
    */
    if (_set_b || _set_f)
    {
        dist += 5.0f;
    }

    if (dist <= 0.1f)
    {
        robot.motor('A', 0);
        robot.motor('C', 0);
        return;
    }

    //---------------- ปรับแขนไหล่ ----------------

    if (arm_target < 3.0f)
    {
        arm_shoulder('L', 76);
        arm_shoulder('R', 62);
    }

    //---------------- มุมเป้าหมาย ----------------

    /*
       ใช้มุมเป้าหมายที่สั่งจริง ไม่ใช่มุมที่หมุนได้จริง

       ตัวอย่าง:
       turnGyro(70, -90);

       แม้หมุนจริงได้ -87°
       degree_turnGyro ยังเป็นประมาณ -90°
       รถจะค่อย ๆ แก้ทิศไปยัง -90° ระหว่างวิ่ง
    */
    const float targetAngle =
        constrainAngle180(deg);

    //---------------- ความเร็วมอเตอร์ฐาน ----------------

    const int leftBase =
        constrain(sl, -100, 100);

    const int rightBase =
        constrain(sr, -100, 100);

    if (leftBase == 0 && rightBase == 0)
    {
        robot.motor('A', 0);
        robot.motor('C', 0);
        return;
    }

    const bool movingBackward =
        leftBase < 0 &&
        rightBase < 0;

    //---------------- Encoder ----------------

    encoder1.resetEncoders();

    const long startPulses =
        encoder1.Poss_L();

    const float pulsesPerCmUsed =
        movingBackward
        ? pulsesPerCm_bw
        : pulsesPerCm_fw;

    if (pulsesPerCmUsed <= 0.0f)
    {
        Serial.println("MOVE ERROR: pulsesPerCmUsed <= 0");

        robot.motor('A', 0);
        robot.motor('C', 0);
        return;
    }

    //---------------- เริ่มขยับแขน ----------------

    arm_updown_nonblocking(arm_target);

    //---------------- PID variables ----------------

    float integral = 0.0f;
    float lastError = 0.0f;
    float dFilter = 0.0f;

    unsigned long lastTime = micros();
    unsigned long startTime = millis();

    //---------------- ค่าควบคุม ----------------

    const float KD_FILTER = 0.75f;

    const int MIN_SPEED = 32;

    const float averageBaseSpeed =
        (
            fabs((float)leftBase) +
            fabs((float)rightBase)
        ) * 0.5f;

    // ระยะเร่งความเร็ว
    const float ACC_DISTANCE = 15.0f;

    // ระยะเริ่มชะลอ
    const float SLOW_DISTANCE =
        20.0f +
        averageBaseSpeed * 0.40f;

    // ระยะช่วงเข้าจอด
    const float FINAL_APPROACH_DISTANCE = 9.0f;

    // ระยะที่ถือว่าถึงจุดเป้าหมาย
    const float STOP_DISTANCE = 4.0f;

    // จำกัดแรงแก้มุม
    const float MAX_CORRECTION = 30.0f;

    /*
       Timeout ตามระยะทาง

       ตัวอย่าง 100 cm:
       4000 + 100 × 100 = 14000 ms
    */
    unsigned long timeout =
        4000UL +
        (unsigned long)(dist * 100.0f);

    timeout =
        constrain(timeout, 6000UL, 30000UL);

    //---------------- สถานะ ----------------

    bool wasDecelerating = false;
    bool reachedTarget = false;
    bool timeoutOccurred = false;

    float traveled = 0.0f;
    float remain = dist;

    //---------------- Debug เริ่มต้น ----------------

    imu.update();
    //---------------- Main loop ----------------
    while (true)
    {
        //---------------- อัปเดตแขน ----------------

        arm_update_nonblocking();

        //---------------- คำนวณระยะทาง ----------------

        const long currentPulses =
            encoder1.Poss_L();

        const long pulseDifference =
            labs(currentPulses - startPulses);

        traveled =
            pulseDifference /
            pulsesPerCmUsed;

        remain =
            dist - traveled;

        //---------------- ตรวจถึงเป้าหมาย ----------------

        if (remain <= STOP_DISTANCE)
        {
            reachedTarget = true;
            break;
        }

        //---------------- อ่านมุม IMU ----------------

        imu.update();

        const float currentAngle =
            imu.yaw();

        //---------------- Speed profile ----------------

        int baseLeft = leftBase;
        int baseRight = rightBase;

        /*
           ช่วงเร่งความเร็ว
        */
        if (traveled < ACC_DISTANCE)
        {
            float progress =
                traveled / ACC_DISTANCE;

            progress =
                constrain(progress, 0.0f, 1.0f);

            float leftMagnitude =
                MIN_SPEED +
                (
                    fabs((float)leftBase) -
                    MIN_SPEED
                ) * progress;

            float rightMagnitude =
                MIN_SPEED +
                (
                    fabs((float)rightBase) -
                    MIN_SPEED
                ) * progress;

            leftMagnitude =
                max(leftMagnitude, (float)MIN_SPEED);

            rightMagnitude =
                max(rightMagnitude, (float)MIN_SPEED);

            baseLeft =
                (int)roundf(leftMagnitude);

            baseRight =
                (int)roundf(rightMagnitude);

            if (leftBase < 0)
                baseLeft = -baseLeft;

            if (rightBase < 0)
                baseRight = -baseRight;
        }
        /*
           ช่วงชะลอความเร็ว
        */
        else if (remain < SLOW_DISTANCE)
        {
            float progress =
                remain / SLOW_DISTANCE;

            progress =
                constrain(progress, 0.0f, 1.0f);

            /*
               progress = 1 อยู่ต้นช่วงชะลอ
               progress = 0 ใกล้จุดหยุด
            */
            const float slowFactor =
                0.35f +
                0.65f * progress;

            float leftMagnitude =
                MIN_SPEED +
                (
                    fabs((float)leftBase) -
                    MIN_SPEED
                ) * slowFactor;

            float rightMagnitude =
                MIN_SPEED +
                (
                    fabs((float)rightBase) -
                    MIN_SPEED
                ) * slowFactor;

            leftMagnitude =
                max(leftMagnitude, (float)MIN_SPEED);

            rightMagnitude =
                max(rightMagnitude, (float)MIN_SPEED);

            baseLeft =
                (int)roundf(leftMagnitude);

            baseRight =
                (int)roundf(rightMagnitude);

            if (leftBase < 0)
                baseLeft = -baseLeft;

            if (rightBase < 0)
                baseRight = -baseRight;
        }

        /*
           ช่วง 4–9 cm สุดท้าย ลดความเร็วเพิ่ม
        */
        if (remain <= FINAL_APPROACH_DISTANCE)
        {
            const float finalFactor = 0.48f;

            int finalLeft =
                (int)roundf(
                    fabs((float)leftBase) *
                    finalFactor
                );

            int finalRight =
                (int)roundf(
                    fabs((float)rightBase) *
                    finalFactor
                );

            /*
               ป้องกันแรงต่ำเกินจนมอเตอร์ไม่หมุน
            */
            finalLeft =
                max(finalLeft, 18);

            finalRight =
                max(finalRight, 18);

            baseLeft =
                leftBase >= 0
                ? finalLeft
                : -finalLeft;

            baseRight =
                rightBase >= 0
                ? finalRight
                : -finalRight;
        }

        //---------------- Heading error ----------------

        float headingError =
            constrainAngle180(
                targetAngle - currentAngle
            );

        /*
           ถอยหลังต้องกลับทิศการชดเชย
        */
        const float controlError =
            movingBackward
            ? -headingError
            : headingError;

        //---------------- Delta time ----------------

        const unsigned long nowMicros =
            micros();

        float dt =
            (nowMicros - lastTime) *
            0.000001f;

        lastTime =
            nowMicros;

        dt =
            constrain(dt, 0.003f, 0.05f);

        //---------------- ตรวจช่วงชะลอ ----------------

        const bool isDecelerating =
            traveled >= ACC_DISTANCE &&
            remain < SLOW_DISTANCE;

        if (isDecelerating &&
            !wasDecelerating)
        {
            integral = 0.0f;
        }

        wasDecelerating =
            isDecelerating;

        //---------------- Integral ----------------

        /*
           ยังไม่ได้ใช้ Ki ใน correction
           แต่เก็บ Integral ไว้รองรับการเพิ่ม Ki ภายหลัง
        */
        if (fabs(controlError) < 15.0f)
        {
            integral +=
                controlError * dt;

            integral =
                constrain(
                    integral,
                    -40.0f,
                    40.0f
                );
        }
        else
        {
            integral = 0.0f;
        }

        //---------------- Derivative ----------------

        const float derivative =
            (controlError - lastError) /
            dt;

        dFilter =
            KD_FILTER * dFilter +
            (1.0f - KD_FILTER) *
            derivative;

        lastError =
            controlError;

        //---------------- PID correction ----------------

        float correction =
            kp * controlError +
            move_kd * dFilter;

        /*
           ลดแรงแก้มุมเมื่อความเร็วฐานต่ำ
        */
        const float currentAverageSpeed =
            (
                fabs((float)baseLeft) +
                fabs((float)baseRight)
            ) * 0.5f;

        const float speedFactor =
            constrain(
                currentAverageSpeed / 40.0f,
                0.33f,
                1.0f
            );

        correction *=
            speedFactor;

        correction =
            constrain(
                correction,
                -MAX_CORRECTION,
                MAX_CORRECTION
            );

        //---------------- คำนวณมอเตอร์ ----------------

        int leftMotor;
        int rightMotor;

        if (movingBackward)
        {
            leftMotor =
                baseLeft -
                (int)roundf(correction);

            rightMotor =
                baseRight +
                (int)roundf(correction);
        }
        else
        {
            leftMotor =
                baseLeft +
                (int)roundf(correction);

            rightMotor =
                baseRight -
                (int)roundf(correction);
        }

        leftMotor =
            constrain(leftMotor, -100, 100);

        rightMotor =
            constrain(rightMotor, -100, 100);

        //---------------- อ่าน Line Sensor ----------------

        /*
           อ่านเพียงครั้งเดียวต่อรอบ
           ลดเวลาการอ่าน ADC และป้องกันค่าต่างกัน
           ภายในเงื่อนไขเดียวกัน
        */
        bool lineCorrectionActive = false;

        int lineLeftMotor = leftMotor;
        int lineRightMotor = rightMotor;

        if (leftBase > 0 && rightBase > 0)
        {
            // เดินหน้า ใช้ Sensor 8 และ 1

            const int sensorLeft =
                robot.adcRead(8);

            const int sensorRight =
                robot.adcRead(1);

            const int thresholdLeft =
                robot.adcMD(8);

            const int thresholdRight =
                robot.adcMD(1);

            const bool leftOnLine =
                sensorLeft < thresholdLeft;

            const bool rightOnLine =
                sensorRight < thresholdRight;

            if (leftOnLine && !rightOnLine)
            {
                lineLeftMotor =
                    (int)roundf(leftMotor * 0.70f);

                lineRightMotor =
                    (int)roundf(rightMotor * 1.25f);

                lineCorrectionActive = true;
            }
            else if (!leftOnLine && rightOnLine)
            {
                lineLeftMotor =
                    (int)roundf(leftMotor * 1.25f);

                lineRightMotor =
                    (int)roundf(rightMotor * 0.70f);

                lineCorrectionActive = true;
            }
        }
        else if (leftBase < 0 && rightBase < 0)
        {
            // ถอยหลัง ใช้ Sensor 9 และ 0

            const int sensorLeft =
                robot.adcRead(9);

            const int sensorRight =
                robot.adcRead(0);

            const int thresholdLeft =
                robot.adcMD(9);

            const int thresholdRight =
                robot.adcMD(0);

            const bool leftOnLine =
                sensorLeft < thresholdLeft;

            const bool rightOnLine =
                sensorRight < thresholdRight;

            if (leftOnLine && !rightOnLine)
            {
                lineLeftMotor =
                    (int)roundf(leftMotor * 0.70f);

                lineRightMotor =
                    (int)roundf(rightMotor * 1.25f);

                lineCorrectionActive = true;
            }
            else if (!leftOnLine && rightOnLine)
            {
                lineLeftMotor =
                    (int)roundf(leftMotor * 1.25f);

                lineRightMotor =
                    (int)roundf(rightMotor * 0.70f);

                lineCorrectionActive = true;
            }
        }

        lineLeftMotor =
            constrain(lineLeftMotor, -100, 100);

        lineRightMotor =
            constrain(lineRightMotor, -100, 100);

        //---------------- ส่งคำสั่งมอเตอร์ ----------------

        if (lineCorrectionActive)
        {
            /*
               ไม่รีเซต Yaw ตรงนี้

               หลังออกจากเส้น ระบบ PID จะนำรถ
               กลับไปยัง targetAngle เดิมโดยอัตโนมัติ
            */
            robot.motor('A', leftMotor);
            robot.motor('C', rightMotor);
        }
        else
        {
            robot.motor('A', leftMotor);
            robot.motor('C', rightMotor);
        }

        //---------------- Safety timeout ----------------

        if (millis() - startTime > timeout)
        {
            timeoutOccurred = true;

            Serial.println("MOVE TIMEOUT");
            break;
        }

        delay(4);
    }

    //---------------- Brake ----------------

    /*
       offset > 0 จึงใช้ Brake pulse

       offset คือระยะเวลาเบรก หน่วย ms
    */
    if (offset > 0 &&
        break_move == 1)
    {
        const float BRAKE_FACTOR = 0.12f;

        int brakeLeft =
            (int)roundf(
                -leftBase *
                BRAKE_FACTOR
            );

        int brakeRight =
            (int)roundf(
                -rightBase *
                BRAKE_FACTOR
            );

        brakeLeft =
            constrain(brakeLeft, -15, 15);

        brakeRight =
            constrain(brakeRight, -15, 15);

        robot.motor('A', brakeLeft);
        robot.motor('C', brakeRight);

        delay(
            constrain(
                offset,
                1,
                20
            )
        );
    }

    //---------------- หยุดมอเตอร์ ----------------

    robot.motor('A', 0);
    robot.motor('C', 0);
    robot.motor('D', 0);

    //---------------- อัปเดตแขนครั้งสุดท้าย ----------------

    arm_update_nonblocking();

    //---------------- Reset states ----------------

    arm_moving = false;
    arm_homing_mode = false;

    _set_f = false;
    _set_b = false;
    _offset = false;

    break_move = 1;

    //---------------- แสดงผลลัพธ์ ----------------

    imu.update();

    const float finalAngle =
        imu.yaw();

    const float finalHeadingError =
        constrainAngle180(
            targetAngle - finalAngle
        );

    // Serial.print("MOVE FINISHED");

    // Serial.print(" | Target angle = ");
    // Serial.print(targetAngle, 2);

    // Serial.print(" | Final angle = ");
    // Serial.print(finalAngle, 2);

    // Serial.print(" | Heading error = ");
    // Serial.print(finalHeadingError, 2);

    // Serial.print(" | Traveled = ");
    // Serial.print(traveled, 1);

    // Serial.print(" cm | Target distance = ");
    // Serial.print(dist, 1);

    // Serial.print(" cm | Remaining = ");
    // Serial.print(remain, 1);

    // Serial.print(" cm | Status = ");

    // if (reachedTarget)
    // {
    //     Serial.println("ARRIVED");
    // }
    // else if (timeoutOccurred)
    // {
    //     Serial.println("TIMEOUT");
    // }
    // else
    // {
    //     Serial.println("STOPPED");
    // }
}

void move_to_can(int sl, int sr, float kp, float dist, float arm_target)
{
    extern PiperPico2 robot;
    extern my_BMI160 gyro;
    extern EncoderLibrarys encoder1;
    extern EncoderLibraryss encoder2;

    if (dist <= 0.1f) return;

    if (_set_b == true) dist += 5;

    encoder1.resetEncoders();

    gyro.resetAngles();
    float targetAngle = 0.0f;
    for (int i = 0; i < 3; i++) {
        targetAngle += gyro.gyro('z');
        delay(2);
    }
    targetAngle /= 3.0f;

    arm_updown_nonblocking(arm_target);

    float integral = 0.0f;
    float lastError = 0.0f;
    unsigned long lastTime = millis();

    int leftBase  = constrain(sl, -100, 100);
    int rightBase = constrain(sr, -100, 100);

    long startPulses = encoder1.Poss_L();
    bool movingBackward = (leftBase < 0 && rightBase < 0);
    float pulsesPerCm_used = movingBackward ? pulsesPerCm_bw : pulsesPerCm_fw;

    const int MIN_SPEED = 20;
    float maxSpeed = (abs(leftBase) + abs(rightBase)) / 2.0f;
    const float ACC_DIST = 20.0f;
    const float SLOW_DIST = 20.0f + maxSpeed * 0.40f;

    bool wasDecelerating = false;

    while (true)
    {
        float currentAngle = gyro.gyro('z');
        long currentPulses = encoder1.Poss_L();
        long pulses = labs(currentPulses - startPulses);

        float traveled = pulses / pulsesPerCm_used;
        float remain = dist - traveled;

        arm_update_nonblocking();

        if (remain <= 9.0f)
        {
            if (remain > 4.0f) {
                robot.motor('A', (int)(leftBase * 0.88f));
                robot.motor('C', (int)(rightBase * 0.88f));
                delay(12);
            } else {
                if(break_move == 1) {
                    float brakeFactor = 0.38f;
                    robot.motor('A', (int)(-leftBase * brakeFactor));
                    robot.motor('C', (int)(-rightBase * brakeFactor));
                    delay(10);
                }
                robot.motor('A', 0);
                robot.motor('C', 0);
                break;
            }
        }

        int baseLeft  = leftBase;
        int baseRight = rightBase;

        if (traveled < ACC_DIST) {
            float p = traveled / ACC_DIST;
            baseLeft  = (int)(MIN_SPEED + (abs(leftBase)  - MIN_SPEED) * p) * (leftBase  >= 0 ? 1 : -1);
            baseRight = (int)(MIN_SPEED + (abs(rightBase) - MIN_SPEED) * p) * (rightBase >= 0 ? 1 : -1);
        }
        else if (remain < SLOW_DIST) {
            float p = remain / SLOW_DIST;
            p = constrain(p, 0.0f, 1.0f);
            float slowFactor = 0.78f + 0.22f * p;
            baseLeft  = (int)(MIN_SPEED + (abs(leftBase)  - MIN_SPEED) * slowFactor * p) * (leftBase  >= 0 ? 1 : -1);
            baseRight = (int)(MIN_SPEED + (abs(rightBase) - MIN_SPEED) * slowFactor * p) * (rightBase >= 0 ? 1 : -1);
        }

        float error = constrainAngle180(targetAngle - currentAngle);
        if (movingBackward) error = -error;

        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;
        if (dt < 0.002f) dt = 0.02f;

        bool isDecelerating = (traveled >= ACC_DIST && remain < SLOW_DIST);
        if (isDecelerating && !wasDecelerating) integral = 0.0f;
        wasDecelerating = isDecelerating;

        integral += error * dt;
        integral = constrain(integral, -65.0f, 65.0f);

        float derivative = (error - lastError) / dt;
        lastError = error;

        float correction = kp * error  + move_kd * derivative;
        float speedFactor = constrain((abs(baseLeft) + abs(baseRight)) / 80.0f, 0.33f, 1.0f);
        correction *= speedFactor;
        correction = constrain(correction, -40.0f, 40.0f);

        int left  = baseLeft  + (movingBackward ? -correction : correction);
        int right = baseRight - (movingBackward ? -correction : correction);

        left  = constrain(left,  -100, 100);
        right = constrain(right, -100, 100);
        robot.motor('A', left); robot.motor('C', right); 
      

        delay(4);
    }

    robot.motor('A', 0);
    robot.motor('C', 0);
    robot.motor('D', 0);
    arm_moving = false;
    arm_homing_mode = false;
    _set_f = false;
    _set_b = false;
    break_move = 1;

   // Serial.printf("🏁 move() FINISHED → Arm Position = %.2f cm\n", current_position_cm);

}

void move_out_stand(int sl, int sr, float kp, float dist)
{
    extern PiperPico2 robot;
    extern my_BMI160 gyro;
    extern EncoderLibrarys encoder1;
    extern EncoderLibraryss encoder2;

    if (dist <= 0.1f) return;

    if (_set_b == true) dist += 5;

    encoder1.resetEncoders();

    gyro.resetAngles();
    float targetAngle = 0.0f;
    for (int i = 0; i < 3; i++) {
        targetAngle += gyro.gyro('z');
        delay(2);
    }
    targetAngle /= 3.0f;


    float integral = 0.0f;
    float lastError = 0.0f;
    unsigned long lastTime = millis();

    int leftBase  = constrain(sl, -100, 100);
    int rightBase = constrain(sr, -100, 100);

    long startPulses = encoder1.Poss_L();
    bool movingBackward = (leftBase < 0 && rightBase < 0);
    float pulsesPerCm_used = movingBackward ? pulsesPerCm_bw : pulsesPerCm_fw;

    const int MIN_SPEED = 20;
    float maxSpeed = (abs(leftBase) + abs(rightBase)) / 2.0f;
    const float ACC_DIST = 20.0f;
    const float SLOW_DIST = 20.0f + maxSpeed * 0.40f;

    bool wasDecelerating = false;

    while (true)
    {
        float currentAngle = gyro.gyro('z');
        long currentPulses = encoder1.Poss_L();
        long pulses = labs(currentPulses - startPulses);

        float traveled = pulses / pulsesPerCm_used;
        float remain = dist - traveled;

        if (remain <= 9.0f)
        {
            if (remain > 4.0f) {
                robot.motor('A', (int)(leftBase * 0.88f));
                robot.motor('C', (int)(rightBase * 0.88f));
                delay(12);
            } else {
                if(break_move == 1) {
                    float brakeFactor = 0.38f;
                    robot.motor('A', (int)(-leftBase * brakeFactor));
                    robot.motor('C', (int)(-rightBase * brakeFactor));
                    delay(10);
                }
                robot.motor('A', 0);
                robot.motor('C', 0);
                break;
            }
        }

        int baseLeft  = leftBase;
        int baseRight = rightBase;

        if (traveled < ACC_DIST) {
            float p = traveled / ACC_DIST;
            baseLeft  = (int)(MIN_SPEED + (abs(leftBase)  - MIN_SPEED) * p) * (leftBase  >= 0 ? 1 : -1);
            baseRight = (int)(MIN_SPEED + (abs(rightBase) - MIN_SPEED) * p) * (rightBase >= 0 ? 1 : -1);
        }
        else if (remain < SLOW_DIST) {
            float p = remain / SLOW_DIST;
            p = constrain(p, 0.0f, 1.0f);
            float slowFactor = 0.78f + 0.22f * p;
            baseLeft  = (int)(MIN_SPEED + (abs(leftBase)  - MIN_SPEED) * slowFactor * p) * (leftBase  >= 0 ? 1 : -1);
            baseRight = (int)(MIN_SPEED + (abs(rightBase) - MIN_SPEED) * slowFactor * p) * (rightBase >= 0 ? 1 : -1);
        }

        float error = constrainAngle180(targetAngle - currentAngle);
        if (movingBackward) error = -error;

        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;
        if (dt < 0.002f) dt = 0.02f;

        bool isDecelerating = (traveled >= ACC_DIST && remain < SLOW_DIST);
        if (isDecelerating && !wasDecelerating) integral = 0.0f;
        wasDecelerating = isDecelerating;

        integral += error * dt;
        integral = constrain(integral, -65.0f, 65.0f);

        float derivative = (error - lastError) / dt;
        lastError = error;

        float correction = kp * error  + move_kd * derivative;
        float speedFactor = constrain((abs(baseLeft) + abs(baseRight)) / 80.0f, 0.33f, 1.0f);
        correction *= speedFactor;
        correction = constrain(correction, -40.0f, 40.0f);

        int left  = baseLeft  + (movingBackward ? -correction : correction);
        int right = baseRight - (movingBackward ? -correction : correction);

        left  = constrain(left,  -100, 100);
        right = constrain(right, -100, 100);
        robot.motor('A', left); robot.motor('C', right); 
      

        delay(4);
    }

    robot.motor('A', 0);
    robot.motor('C', 0);
    robot.motor('D', 0);
    arm_moving = false;
    arm_homing_mode = false;
    _set_f = false;
    _set_b = false;
    break_move = 1;

   // Serial.printf("🏁 move() FINISHED → Arm Position = %.2f cm\n", current_position_cm);

}

void move_untrasonic(int sl, int sr, float kp, float untra)
{
    extern PiperPico2 robot;
    extern my_BMI160 gyro;

    //if (untra <= 2.0f) return;  // ระยะปลอดภัยขั้นต่ำ

    gyro.resetAngles();

    // คำนวณมุมเริ่มต้น
    float targetAngle = 0.0f;
    for (int i = 0; i < 3; i++) {
        targetAngle += gyro.gyro('z');
        delay(2);
    }
    targetAngle /= 3.0f;

    float integral = 0.0f;
    float lastError = 0.0f;
    unsigned long lastTime = millis();

    int leftBase  = constrain(sl, -100, 100);
    int rightBase = constrain(sr, -100, 100);

    bool movingBackward = (leftBase < 0 && rightBase < 0);

    float currentDistance = 999.0f;
    const float SLOW_DISTANCE = 5.0f;   // ชะลอเมื่อเหลือ 5 ซม.

    while (true)
    {
        // อ่าน Ultrasonic
        currentDistance = untrasonic();

        // อ่าน Gyro
        float currentAngle = gyro.gyro('z');

        // === เงื่อนไขหยุดหลัก ===
        if (currentDistance <= untra )
        {                              
            break;
        }

        // === Speed Control ===
        int baseLeft  = leftBase;
        int baseRight = rightBase;

        // === ชะลอเมื่อเหลือ 5 ซม. ===
        if (currentDistance < untra + SLOW_DISTANCE)
        {
            float remain = currentDistance - untra;
            float p = remain / SLOW_DISTANCE;           // p จะลดลงเมื่อใกล้เป้าหมาย
            p = constrain(p, 0.0f, 1.0f);
            
            float slowFactor = 0.45f + 0.55f * p;       // ชะลอลงเหลือประมาณ 45% ของความเร็ว
            
            baseLeft  = (int)(abs(leftBase)  * slowFactor) * (leftBase  >= 0 ? 1 : -1);
            baseRight = (int)(abs(rightBase) * slowFactor) * (rightBase >= 0 ? 1 : -1);
        }

        // === PID Heading Control ===
        float error = constrainAngle180(targetAngle - currentAngle);
        if (movingBackward) error = -error;

        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;
        if (dt < 0.002f) dt = 0.025f;

        integral += error * dt;
        integral = constrain(integral, -65.0f, 65.0f);

        float derivative = (error - lastError) / dt;
        lastError = error;

        float correction = kp * error + move_kd * derivative;
        float speedFactor = constrain((abs(baseLeft) + abs(baseRight)) / 80.0f, 0.35f, 1.0f);
        correction *= speedFactor;
        correction = constrain(correction, -32.0f, 32.0f);

        int left  = baseLeft  + (movingBackward ? -correction : correction);
        int right = baseRight - (movingBackward ? -correction : correction);

        left  = constrain(left,  -100, 100);
        right = constrain(right, -100, 100);

        // ส่งคำสั่งมอเตอร์
        robot.motor('A', left);
        robot.motor('C', right);

        delay(6);
    }

    // จบการทำงาน
    robot.motor('A',-10 );
    robot.motor('C',-10 );
    delay(10);
    robot.motor('A', -1);
    robot.motor('C', -1);
    robot.motor('D', 0);

    arm_moving = false;
    arm_homing_mode = false;
    _set_f = false;
    _set_b = false;
    break_move = 1;

    Serial.printf("🏁 move_untrasonic() FINISHED → Distance = %.1f cm (Target = %.1f cm)\n", 
                  currentDistance, untra);
}

void move_untrasonic(    
    int sl,         // ความเร็วมอเตอร์ซ้าย
    int sr,         // ความเร็วมอเตอร์ขวา
    float kp,       // Kp รักษาทิศทาง
    float untra,     // ระยะหยุดจาก Ultrasonic
    float deg=degree_turnGyro     // มุมเป้าหมายที่ต้องการรักษา
)
{
    extern PiperPico2 robot;

    //---------------- Target heading ----------------

    /*
       ใช้มุมที่ผู้ใช้กำหนดโดยตรง

       ตัวอย่าง:
       deg = 0      รักษามุม 0°
       deg = -90    รักษามุม -90°
       deg = 90     รักษามุม 90°
       deg = -180   รักษาทิศ 180°
    */
    const float targetAngle =
        constrainAngle180(deg);
    //---------------- Base motor speed ----------------
    const int leftBase =
        constrain(sl, -100, 100);
    const int rightBase =
        constrain(sr, -100, 100);
    /*
       เดินหน้า:
       sl และ sr เป็นค่าบวก
       ถอยหลัง:
       sl และ sr เป็นค่าลบ
    */
    const bool movingBackward =
        leftBase < 0 &&
        rightBase < 0;
    //---------------- PID variables ----------------
    float integral = 0.0f;
    float lastError = 0.0f;
    float dFilter = 0.0f;
    unsigned long lastTime = micros();
    unsigned long startTime = millis();
    //---------------- Configuration ----------------
    // ตัวกรองค่า Derivative
    const float KD_FILTER = 0.75f;
    // เริ่มชะลอก่อนถึงระยะเป้าหมาย
    const float SLOW_DISTANCE = 5.0f;
    // ความเร็วต่ำสุดเมื่อใกล้ระยะเป้าหมาย
    const float MIN_SLOW_FACTOR = 0.45f;
    // ช่วงค่าที่ถือว่า Ultrasonic อ่านได้ถูกต้อง
    const float MIN_VALID_DISTANCE = 1.0f;
    const float MAX_VALID_DISTANCE = 400.0f;
    // จำนวนครั้งที่ต้องพบระยะถึงเป้าหมายต่อเนื่อง
    const uint8_t DISTANCE_STABLE_COUNT = 2;
    // ป้องกันฟังก์ชันทำงานไม่สิ้นสุด
    const unsigned long TIMEOUT = 10000UL;
    uint8_t distanceStableCount = 0;
    float currentDistance = 999.0f;
    //---------------- Initial IMU reading ----------------
    imu.update();
    float currentAngle = imu.yaw();
    float initialError =
        constrainAngle180(
            targetAngle - currentAngle
        );
    //---------------- Main loop ----------------
    while (true)
    {
        //---------------- Read ultrasonic ----------------
        currentDistance = untrasonic();
        const bool validDistance =
            currentDistance >= MIN_VALID_DISTANCE &&
            currentDistance <= MAX_VALID_DISTANCE;
        //---------------- Distance stop detection ----------------
        /*
           ให้พบระยะถึงเป้าหมายต่อเนื่องมากกว่า 1 ครั้ง
           ป้องกัน Ultrasonic กระโดดเพียงครั้งเดียวแล้วหยุด
        */
        if (validDistance &&
            currentDistance <= untra)
        {
            distanceStableCount++;

            if (distanceStableCount >= DISTANCE_STABLE_COUNT)
            {
                break;
            }
        }
        else
        {
            distanceStableCount = 0;
        }

        //---------------- Read IMU ----------------
        imu.update();

        currentAngle = imu.yaw();
        //---------------- Speed profile ----------------
        int baseLeft = leftBase;
        int baseRight = rightBase;
        /*
           ชะลอความเร็วเมื่อเข้าใกล้ระยะหยุด

           ตัวอย่าง:
           Target = 15 cm
           เริ่มชะลอที่ 20 cm
        */
        if (validDistance &&
            currentDistance < untra + SLOW_DISTANCE)
        {
            float remain =
                currentDistance - untra;

            float progress =
                remain / SLOW_DISTANCE;

            progress =
                constrain(progress, 0.0f, 1.0f);

            const float slowFactor =
                MIN_SLOW_FACTOR +
                (1.0f - MIN_SLOW_FACTOR) * progress;

            baseLeft =
                (int)roundf(
                    fabs((float)leftBase) *
                    slowFactor
                );

            baseRight =
                (int)roundf(
                    fabs((float)rightBase) *
                    slowFactor
                );

            if (leftBase < 0)
            {
                baseLeft = -baseLeft;
            }

            if (rightBase < 0)
            {
                baseRight = -baseRight;
            }
        }
        //---------------- Heading error ----------------

        /*
           ตัวอย่าง:
           Target  = -180°
           Current = -178°
           Error   = -2°

           รถจะค่อย ๆ แก้ทิศไปยัง -180°
        */
        float error =
            constrainAngle180(
                targetAngle - currentAngle
            );

        /*
           ตอนถอยหลัง ทิศทางการชดเชยมอเตอร์ต้องกลับด้าน
        */
        float controlError = error;

        if (movingBackward)
        {
            controlError = -controlError;
        }

        //---------------- Delta time ----------------

        const unsigned long nowMicros =
            micros();

        float dt =
            (nowMicros - lastTime) *
            0.000001f;

        lastTime = nowMicros;

        dt =
            constrain(dt, 0.003f, 0.05f);

        //---------------- Integral ----------------
        /*
           Integral ยังไม่ได้ถูกนำไปใช้ใน correction
           แต่เก็บโครงสร้างไว้สำหรับเพิ่ม Ki ในอนาคต
        */
        if (fabs(controlError) < 15.0f)
        {
            integral += controlError * dt;

            integral =
                constrain(
                    integral,
                    -30.0f,
                    30.0f
                );
        }
        else
        {
            integral = 0.0f;
        }

        //---------------- Derivative ----------------

        const float derivative =
            (controlError - lastError) / dt;

        dFilter =
            KD_FILTER * dFilter +
            (1.0f - KD_FILTER) * derivative;

        lastError = controlError;

        //---------------- Heading correction ----------------

        /*
           ใช้ P และ D:

           correction =
               kp × error +
               move_kd × derivative
        */
        float correction =
            kp * controlError +
            move_kd * dFilter;

        //---------------- Scale correction ----------------

        /*
           ลดแรงแก้ทิศเมื่อความเร็วฐานต่ำลง
           เพื่อป้องกันกระตุกตอนเข้าใกล้วัตถุ
        */
        const float averageSpeed =
            (
                fabs((float)baseLeft) +
                fabs((float)baseRight)
            ) * 0.5f;

        const float speedFactor =
            constrain(
                averageSpeed / 40.0f,
                0.35f,
                1.0f
            );

        correction *= speedFactor;

        /*
           จำกัดแรงแก้สูงสุด
           ป้องกันมอเตอร์ข้างหนึ่งย้อนทิศแรงเกินไป
        */
        correction =
            constrain(
                correction,
                -32.0f,
                32.0f
            );

        //---------------- Motor output ----------------

        int leftMotor;
        int rightMotor;

        if (movingBackward)
        {
            /*
               ถอยหลังต้องกลับทิศการชดเชย
            */
            leftMotor =
                baseLeft -
                (int)roundf(correction);

            rightMotor =
                baseRight +
                (int)roundf(correction);
        }
        else
        {
            /*
               เดินหน้า
            */
            leftMotor =
                baseLeft +
                (int)roundf(correction);

            rightMotor =
                baseRight -
                (int)roundf(correction);
        }

        leftMotor =
            constrain(leftMotor, -100, 100);

        rightMotor =
            constrain(rightMotor, -100, 100);

        //---------------- Send motor commands ----------------

        robot.motor('A', leftMotor);
        robot.motor('C', rightMotor);

        //---------------- Safety timeout ----------------

        if (millis() - startTime > TIMEOUT)
        {
            Serial.println("MOVE ULTRASONIC TIMEOUT");
            break;
        }

        delay(6);
    }

    //---------------- Brake ----------------

    /*
       เบรกสวนทางกับทิศการเคลื่อนที่

       เดินหน้า  -> เบรกค่าลบ
       ถอยหลัง   -> เบรกค่าบวก
    */
    int brakeLeft = 0;
    int brakeRight = 0;

    if (leftBase > 0)
    {
        brakeLeft = -10;
    }
    else if (leftBase < 0)
    {
        brakeLeft = 10;
    }

    if (rightBase > 0)
    {
        brakeRight = -10;
    }
    else if (rightBase < 0)
    {
        brakeRight = 10;
    }

    robot.motor('A', brakeLeft);
    robot.motor('C', brakeRight);

    delay(15);

    //---------------- Final stop ----------------

    robot.motor('A', 0);
    robot.motor('C', 0);
    robot.motor('D', 0);

    //---------------- Reset states ----------------

    arm_moving = false;
    arm_homing_mode = false;

    _set_f = false;
    _set_b = false;

    break_move = 1;

    //---------------- Final result ----------------

    imu.update();

    const float finalAngle =
        imu.yaw();

    const float finalHeadingError =
        constrainAngle180(
            targetAngle - finalAngle
        );

    Serial.print("MOVE ULTRASONIC FINISHED");

    Serial.print(" | Target angle = ");
    Serial.print(targetAngle, 2);

    Serial.print(" | Final angle = ");
    Serial.print(finalAngle, 2);

    Serial.print(" | Heading error = ");
    Serial.print(finalHeadingError, 2);

    Serial.print(" | Distance = ");
    Serial.print(currentDistance, 1);

    Serial.print(" | Target distance = ");
    Serial.println(untra, 1);
}


void move_untrasonic_to_stand(int sl, int sr, float kp)
{
    extern PiperPico2 robot;
    extern my_BMI160 gyro;

    //if (untra <= 2.0f) return;  // ระยะปลอดภัยขั้นต่ำ

    gyro.resetAngles();

    // คำนวณมุมเริ่มต้น
    float targetAngle = 0.0f;
    for (int i = 0; i < 3; i++) {
        targetAngle += gyro.gyro('z');
        delay(2);
    }
    targetAngle /= 3.0f;

    float integral = 0.0f;
    float lastError = 0.0f;
    unsigned long lastTime = millis();

    int leftBase  = constrain(sl, -100, 100);
    int rightBase = constrain(sr, -100, 100);

    bool movingBackward = (leftBase < 0 && rightBase < 0);

    float currentDistance = 999.0f;
    const float SLOW_DISTANCE = 5.0f;   // ชะลอเมื่อเหลือ 5 ซม.
    float untra = 7;

    while (true)
    {
        // อ่าน Ultrasonic
        currentDistance = untrasonic();

        // อ่าน Gyro
        float currentAngle = gyro.gyro('z');

        // === เงื่อนไขหยุดหลัก ===
        if (currentDistance <= 3)
        {                         
            break;
        }
        Serial.println(currentDistance);
        // === Speed Control ===
        int baseLeft  = leftBase;
        int baseRight = rightBase;

        // === ชะลอเมื่อเหลือ 5 ซม. ===
        if (currentDistance < untra + SLOW_DISTANCE)
        {
            float remain = currentDistance - untra;
            float p = remain / SLOW_DISTANCE;           // p จะลดลงเมื่อใกล้เป้าหมาย
            p = constrain(p, 0.0f, 1.0f);
            
            float slowFactor = 0.85f + 0.55f * p;       // ชะลอลงเหลือประมาณ 45% ของความเร็ว
            
            baseLeft  = (int)(abs(leftBase)  * slowFactor) * (leftBase  >= 0 ? 1 : -1);
            baseRight = (int)(abs(rightBase) * slowFactor) * (rightBase >= 0 ? 1 : -1);
        }

        // === PID Heading Control ===
        float error = constrainAngle180(targetAngle - currentAngle);
        if (movingBackward) error = -error;

        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;
        if (dt < 0.002f) dt = 0.025f;

        integral += error * dt;
        integral = constrain(integral, -65.0f, 65.0f);

        float derivative = (error - lastError) / dt;
        lastError = error;

        float correction = kp * error + move_kd * derivative;
        float speedFactor = constrain((abs(baseLeft) + abs(baseRight)) / 80.0f, 0.35f, 1.0f);
        correction *= speedFactor;
        correction = constrain(correction, -32.0f, 32.0f);

        int left  = baseLeft  + (movingBackward ? -correction : correction);
        int right = baseRight - (movingBackward ? -correction : correction);

        left  = constrain(left,  -100, 100);
        right = constrain(right, -100, 100);

        // ส่งคำสั่งมอเตอร์
        robot.motor('A', left);
        robot.motor('C', right);

        delay(6);
    }

    // จบการทำงาน
    robot.motor('A',-10 );
    robot.motor('C',-10 );
    delay(10);
    robot.motor('A', -1);
    robot.motor('C', -1);
    robot.motor('D', 0);

    arm_moving = false;
    arm_homing_mode = false;
    _set_f = false;
    _set_b = false;
    break_move = 1;

}
