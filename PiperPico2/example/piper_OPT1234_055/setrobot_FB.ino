
void setrobot_fw(int _time)
{
    extern PiperPico2 robot;

    _set_f = true;

    if (_time < 0)
    {
        _time = 0;
    }

    //---------------- Heading target ----------------

    /*
       ใช้มุมเป้าหมายล่าสุดจาก turnGyro()

       ตัวอย่าง:
       turnGyro(70, -90);

       แม้หมุนจริงได้ -87°
       degree_turnGyro ยังคงเป็นประมาณ -90°
    */
    const float targetAngle =
        constrainAngle180(degree_turnGyro);

    //---------------- PID configuration ----------------

    const float KP = 1.25f;
    const float KI = 0.00001f;
    const float KD = 0.002f;

    const float KD_FILTER = 0.75f;
    const float MAX_CORRECTION = 30.0f;

    //---------------- Speed configuration ----------------

    const int HEADING_SPEED = 20;
    const int FOLLOW_SPEED = 19;

    //---------------- Timeout configuration ----------------

    const unsigned long FIND_LINE_TIMEOUT = 4000UL;
    const unsigned long ALIGN_TIMEOUT = 2500UL;
    const unsigned long RELEASE_LINE_TIMEOUT = 3000UL;
    const unsigned long COUNT_LINE_TIMEOUT = 5000UL;

    //---------------- PID variables ----------------

    float integral = 0.0f;
    float lastError = 0.0f;
    float dFilter = 0.0f;

    unsigned long lastTime = micros();

    //---------------- Debug ----------------

    imu.update();

    Serial.print("SETROBOT FW START");

    Serial.print(" | Target = ");
    Serial.print(targetAngle, 2);

    Serial.print(" | Current = ");
    Serial.print(imu.yaw(), 2);

    Serial.print(" | Count = ");
    Serial.println(_time);

    //==================================================
    // ขั้นที่ 1 วิ่งรักษามุมจนพบเส้น
    //==================================================

    unsigned long stageStart = millis();

    while (true)
    {
        //---------------- Read IMU ----------------

        imu.update();

        const float currentAngle =
            imu.yaw();

        const float angleError =
            constrainAngle180(
                targetAngle - currentAngle
            );

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

        //---------------- Integral ----------------

        if (fabs(angleError) < 20.0f)
        {
            integral +=
                angleError * dt;

            integral =
                constrain(
                    integral,
                    -60.0f,
                    60.0f
                );
        }
        else
        {
            integral = 0.0f;
        }

        //---------------- Derivative ----------------

        const float derivative =
            (angleError - lastError) /
            dt;

        dFilter =
            KD_FILTER * dFilter +
            (1.0f - KD_FILTER) *
            derivative;

        lastError =
            angleError;

        //---------------- PID output ----------------

        float correction =
            KP * angleError +
            KI * integral +
            KD * dFilter;

        correction =
            constrain(
                correction,
                -MAX_CORRECTION,
                MAX_CORRECTION
            );

        //---------------- Motor calculation ----------------

        /*
           เดินหน้า:

           error บวก:
           เพิ่มมอเตอร์ซ้าย ลดมอเตอร์ขวา

           error ลบ:
           ลดมอเตอร์ซ้าย เพิ่มมอเตอร์ขวา
        */
        int leftSpeed =
            HEADING_SPEED +
            (int)roundf(correction);

        int rightSpeed =
            HEADING_SPEED -
            (int)roundf(correction);

        leftSpeed =
            constrain(leftSpeed, -100, 100);

        rightSpeed =
            constrain(rightSpeed, -100, 100);

        //---------------- Read sensors ----------------

        const int sensorRight =
            robot.adcRead(1);

        const int sensorLeft =
            robot.adcRead(8);

        const int mdRight =
            robot.adcMD(1);

        const int mdLeft =
            robot.adcMD(8);

        const int deepRight =
            (
                robot.adcMD(1) +
                robot.adcMin(1)
            ) / 2;

        const int deepLeft =
            (
                robot.adcMD(8) +
                robot.adcMin(8)
            ) / 2;

        //---------------- Line detection ----------------

        /*
           Sensor ขวาพบเส้นก่อน
        */
        if (sensorRight < deepRight &&
            sensorLeft > mdLeft)
        {
            unsigned long alignStart =
                millis();

            while (true)
            {
                const int leftValue =
                    robot.adcRead(8);

                if (leftValue <=
                    robot.adcMD(8))
                {
                    break;
                }

                robot.motor('A', -5);
                robot.motor('C', 25);

                if (millis() - alignStart >
                    ALIGN_TIMEOUT)
                {
                    Serial.println(
                        "SETROBOT FW ALIGN LEFT TIMEOUT"
                    );
                    break;
                }

                delay(3);
            }

            robot.motor('A', 10);
            robot.motor('C', -25);

            delay(15);
            break;
        }

        /*
           Sensor ซ้ายพบเส้นก่อน
        */
        if (sensorRight > mdRight &&
            sensorLeft < deepLeft)
        {
            unsigned long alignStart =
                millis();

            while (true)
            {
                const int rightValue =
                    robot.adcRead(1);

                if (rightValue <=
                    robot.adcMD(1))
                {
                    break;
                }

                robot.motor('A', 25);
                robot.motor('C', -5);

                if (millis() - alignStart >
                    ALIGN_TIMEOUT)
                {
                    Serial.println(
                        "SETROBOT FW ALIGN RIGHT TIMEOUT"
                    );
                    break;
                }

                delay(3);
            }

            robot.motor('A', -25);
            robot.motor('C', 5);

            delay(15);
            break;
        }

        /*
           Sensor ทั้งสองพบเส้น
        */
        if (sensorRight < mdRight &&
            sensorLeft < mdLeft)
        {
            robot.motor('A', -20);
            robot.motor('C', -20);

            delay(30);
            break;
        }

        //---------------- Normal heading movement ----------------

        robot.motor('A', leftSpeed);
        robot.motor('C', rightSpeed);

        //---------------- Timeout ----------------

        if (millis() - stageStart >
            FIND_LINE_TIMEOUT)
        {
            Serial.println(
                "SETROBOT FW FIND LINE TIMEOUT"
            );

            break;
        }

        delay(5);
    }

    //==================================================
    // ขั้นที่ 2 ถอยออกจากเส้น
    //==================================================

    robot.motor('A', -2);
    robot.motor('C', -2);

    delay(200);

    robot.motor('A', -18);
    robot.motor('C', -18);

    delay(20);

    stageStart = millis();

    while (true)
    {
        robot.motor('A', -17);
        robot.motor('C', -17);

        const int sensorRight =
            robot.adcRead(1);

        const int sensorLeft =
            robot.adcRead(8);

        const int releaseRight =
            (
                robot.adcMD(1) +
                robot.adcMin(1)
            ) / 2;

        const bool rightReleased =
            sensorRight > releaseRight;

        const bool leftReleased =
            sensorLeft > robot.adcMD(8);

        if (rightReleased &&
            leftReleased)
        {
            delay(20);

            robot.motor('A', 5);
            robot.motor('C', 5);

            delay(10);

            robot.motor('A', 0);
            robot.motor('C', 0);

            delay(10);
            break;
        }

        if (millis() - stageStart >
            RELEASE_LINE_TIMEOUT)
        {
            Serial.println(
                "SETROBOT FW RELEASE TIMEOUT"
            );

            robot.motor('A', 0);
            robot.motor('C', 0);
            break;
        }

        delay(3);
    }

    delay(200);

    //==================================================
    // ขั้นที่ 3 นับเส้นตามจำนวน _time
    //==================================================

    for (int i = 0; i < _time; i++)
    {
        stageStart = millis();

        while (true)
        {
            //---------------- Read sensors ----------------

            const int sensorRight =
                robot.adcRead(1);

            const int sensorLeft =
                robot.adcRead(8);

            const int mdRight =
                robot.adcMD(1);

            const int mdLeft =
                robot.adcMD(8);

            //---------------- Line follow ----------------

            if (sensorRight < mdRight &&
                sensorLeft > mdLeft)
            {
                robot.motor('A', -3);
                robot.motor('C', 20);
            }
            else if (sensorRight > mdRight &&
                     sensorLeft < mdLeft)
            {
                robot.motor('A', 20);
                robot.motor('C', -3);
            }
            else if (sensorRight < mdRight &&
                     sensorLeft < mdLeft)
            {
                robot.motor('A', -3);
                robot.motor('C', -3);

                delay(30);
                break;
            }
            else
            {
                /*
                   ไม่พบเส้น ใช้ IMU รักษามุม
                */
                imu.update();

                const float currentAngle =
                    imu.yaw();

                const float angleError =
                    constrainAngle180(
                        targetAngle -
                        currentAngle
                    );

                const unsigned long nowMicros =
                    micros();

                float dt =
                    (nowMicros - lastTime) *
                    0.000001f;

                lastTime =
                    nowMicros;

                dt =
                    constrain(
                        dt,
                        0.003f,
                        0.05f
                    );

                const float derivative =
                    (angleError - lastError) /
                    dt;

                dFilter =
                    KD_FILTER * dFilter +
                    (1.0f - KD_FILTER) *
                    derivative;

                lastError =
                    angleError;

                float correction =
                    KP * angleError +
                    KD * dFilter;

                correction =
                    constrain(
                        correction,
                        -15.0f,
                        15.0f
                    );

                int leftMotor =
                    FOLLOW_SPEED +
                    (int)roundf(correction);

                int rightMotor =
                    FOLLOW_SPEED -
                    (int)roundf(correction);

                leftMotor =
                    constrain(
                        leftMotor,
                        -100,
                        100
                    );

                rightMotor =
                    constrain(
                        rightMotor,
                        -100,
                        100
                    );

                robot.motor('A', leftMotor);
                robot.motor('C', rightMotor);
            }

            if (millis() - stageStart >
                COUNT_LINE_TIMEOUT)
            {
                Serial.print(
                    "SETROBOT FW COUNT TIMEOUT | "
                );

                Serial.println(i + 1);
                break;
            }

            delay(3);
        }

        //---------------- ออกจากเส้นก่อนนับรอบถัดไป ----------------

        if (i < _time - 1)
        {
            robot.motor('A', -17);
            robot.motor('C', -17);

            delay(10);

            stageStart = millis();

            while (true)
            {
                robot.motor('A', -20);
                robot.motor('C', -20);

                const int sensorRight =
                    robot.adcRead(1);

                const int sensorLeft =
                    robot.adcRead(8);

                const int releaseRight =
                    (
                        robot.adcMD(1) +
                        robot.adcMin(1)
                    ) / 2;

                if (sensorRight >
                        releaseRight &&
                    sensorLeft >
                        robot.adcMD(8))
                {
                    delay(20);

                    robot.motor('A', 0);
                    robot.motor('C', 0);

                    delay(10);
                    break;
                }

                if (millis() - stageStart >
                    RELEASE_LINE_TIMEOUT)
                {
                    Serial.println(
                        "SETROBOT FW NEXT LINE TIMEOUT"
                    );

                    robot.motor('A', 0);
                    robot.motor('C', 0);
                    break;
                }

                delay(3);
            }
        }
    }

    //==================================================
    // ขั้นที่ 4 หาเส้นสุดท้าย
    //==================================================

    stageStart = millis();

    while (true)
    {
        const int sensorRight =
            robot.adcRead(1);

        const int sensorLeft =
            robot.adcRead(8);

        const int mdRight =
            robot.adcMD(1);

        const int mdLeft =
            robot.adcMD(8);

        if (sensorRight < mdRight &&
            sensorLeft > mdLeft)
        {
            robot.motor('A', -3);
            robot.motor('C', 18);
        }
        else if (sensorRight > mdRight &&
                 sensorLeft < mdLeft)
        {
            robot.motor('A', 18);
            robot.motor('C', -3);
        }
        else if (sensorRight < mdRight &&
                 sensorLeft < mdLeft)
        {
            robot.motor('A', -3);
            robot.motor('C', -3);

            delay(30);
            break;
        }
        else
        {
            /*
               รักษามุมด้วย IMU
            */
            imu.update();

            const float currentAngle =
                imu.yaw();

            const float angleError =
                constrainAngle180(
                    targetAngle -
                    currentAngle
                );

            const unsigned long nowMicros =
                micros();

            float dt =
                (nowMicros - lastTime) *
                0.000001f;

            lastTime =
                nowMicros;

            dt =
                constrain(
                    dt,
                    0.003f,
                    0.05f
                );

            const float derivative =
                (angleError - lastError) /
                dt;

            dFilter =
                KD_FILTER * dFilter +
                (1.0f - KD_FILTER) *
                derivative;

            lastError =
                angleError;

            float correction =
                KP * angleError +
                KD * dFilter;

            correction =
                constrain(
                    correction,
                    -15.0f,
                    15.0f
                );

            int leftMotor =
                FOLLOW_SPEED +
                (int)roundf(correction);

            int rightMotor =
                FOLLOW_SPEED -
                (int)roundf(correction);

            leftMotor =
                constrain(leftMotor, -100, 100);

            rightMotor =
                constrain(rightMotor, -100, 100);

            robot.motor('A', leftMotor);
            robot.motor('C', rightMotor);
        }

        if (millis() - stageStart >
            COUNT_LINE_TIMEOUT)
        {
            Serial.println(
                "SETROBOT FW FINAL LINE TIMEOUT"
            );
            break;
        }

        delay(3);
    }

    //---------------- Final stop ----------------

    robot.motor('A', -1);
    robot.motor('C', -1);

    delay(60);

    robot.motor('A', 0);
    robot.motor('C', 0);

    //---------------- Final information ----------------

    imu.update();

    const float finalAngle =
        imu.yaw();

    const float finalError =
        constrainAngle180(
            targetAngle - finalAngle
        );

    Serial.print("SETROBOT FW FINISHED");

    Serial.print(" | Target = ");
    Serial.print(targetAngle, 2);

    Serial.print(" | Final = ");
    Serial.print(finalAngle, 2);

    Serial.print(" | Error = ");
    Serial.println(finalError, 2);

    /*
       ไม่ใช้คำสั่งเหล่านี้:
       gyro.resetAngles();
       imu.resetYaw();
       imu.zeroYaw();

       เพื่อให้มุมต่อเนื่องกับ turnGyro(),
       move(), move_to_can() และ move_untrasonic()
    */
}


void setrobot_bw(int _time)
{
    extern PiperPico2 robot;

    _set_b = true;

    if (_time < 0)
    {
        _time = 0;
    }

    //---------------- มุมเป้าหมาย ----------------

    /*
       ใช้มุมเป้าหมายล่าสุดจาก turnGyro()

       ตัวอย่าง:
       turnGyro(70, -90);

       แม้หมุนจริงได้ -87°
       targetAngle ยังคงเป็นประมาณ -90°
    */
    const float targetAngle =
        constrainAngle180(degree_turnGyro);

    //---------------- PID configuration ----------------

    const float HEADING_KP = 0.35f;
    const float HEADING_KI = 0.0f;
    const float HEADING_KD = 0.020f;

    const float KD_FILTER = 0.75f;
    const float MAX_CORRECTION = 30.0f;

    //---------------- Speed configuration ----------------

    const int HEADING_SPEED = 20;
    const int FOLLOW_SPEED = 19;

    //---------------- Timeout configuration ----------------

    const unsigned long LEAVE_LINE_TIMEOUT  = 3000UL;
    const unsigned long FIND_LINE_TIMEOUT   = 4000UL;
    const unsigned long ALIGN_TIMEOUT       = 2500UL;
    const unsigned long RELEASE_TIMEOUT     = 3000UL;
    const unsigned long COUNT_LINE_TIMEOUT  = 5000UL;

    //---------------- PID variables ----------------

    float integral = 0.0f;
    float lastError = 0.0f;
    float dFilter = 0.0f;

    unsigned long lastTime = micros();

    //---------------- Debug ----------------

    imu.update();

    Serial.print("SETROBOT BW START");

    Serial.print(" | Target = ");
    Serial.print(targetAngle, 2);

    Serial.print(" | Current = ");
    Serial.print(imu.yaw(), 2);

    Serial.print(" | Count = ");
    Serial.println(_time);

    //==================================================
    // ขั้นที่ 1 เดินหน้าออกจากเส้นเดิม
    //==================================================

    unsigned long stageStart = millis();

    while (true)
    {
        const int sensorRight =
            robot.adcRead(0);

        const int sensorLeft =
            robot.adcRead(9);

        robot.motor('A', 20);
        robot.motor('C', 20);

        const bool rightReleased =
            sensorRight >= robot.adcMD(0) - 100;

        const bool leftReleased =
            sensorLeft >= robot.adcMD(9) - 100;

        if (rightReleased && leftReleased)
        {
            break;
        }

        if (millis() - stageStart >
            LEAVE_LINE_TIMEOUT)
        {
            Serial.println(
                "SETROBOT BW LEAVE LINE TIMEOUT"
            );
            break;
        }

        delay(3);
    }

    // เบรกการเดินหน้า
    robot.motor('A', -5);
    robot.motor('C', -5);

    delay(25);

    //==================================================
    // ขั้นที่ 2 ถอยหลังรักษามุมจนพบเส้น
    //==================================================

    stageStart = millis();

    lastTime = micros();
    lastError = 0.0f;
    integral = 0.0f;
    dFilter = 0.0f;

    while (true)
    {
        //---------------- อ่าน IMU ----------------

        imu.update();

        const float currentAngle =
            imu.yaw();

        const float headingError =
            constrainAngle180(
                targetAngle - currentAngle
            );

        /*
           เนื่องจากกำลังถอยหลัง
           ต้องกลับทิศ Error สำหรับมอเตอร์
        */
        const float controlError =
            -headingError;

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

        if (fabs(controlError) < 20.0f)
        {
            integral +=
                controlError * dt;

            integral =
                constrain(
                    integral,
                    -60.0f,
                    60.0f
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
            HEADING_KP * controlError +
            HEADING_KI * integral +
            HEADING_KD * dFilter;

        correction =
            constrain(
                correction,
                -MAX_CORRECTION,
                MAX_CORRECTION
            );

        //---------------- Motor calculation ----------------

        /*
           ความเร็วฐานเป็นค่าลบเพราะถอยหลัง
        */
        int leftSpeed =
            -HEADING_SPEED -
            (int)roundf(correction);

        int rightSpeed =
            -HEADING_SPEED +
            (int)roundf(correction);

        leftSpeed =
            constrain(leftSpeed, -100, 100);

        rightSpeed =
            constrain(rightSpeed, -100, 100);

        //---------------- อ่านเซนเซอร์ ----------------

        const int sensorRight =
            robot.adcRead(0);

        const int sensorLeft =
            robot.adcRead(9);

        const int mdRight =
            robot.adcMD(0);

        const int mdLeft =
            robot.adcMD(9);

        const int deepRight =
            (
                robot.adcMD(0) +
                robot.adcMin(0)
            ) / 2;

        const int deepLeft =
            (
                robot.adcMD(9) +
                robot.adcMin(9)
            ) / 2;

        //---------------- Sensor 9 พบเส้นก่อน ----------------

        if (sensorLeft < mdLeft - 100 &&
            sensorRight > mdRight)
        {
            unsigned long alignStart =
                millis();

            while (true)
            {
                const int rightValue =
                    robot.adcRead(0);

                if (rightValue <= deepRight)
                {
                    break;
                }

                robot.motor('A', -30);
                robot.motor('C', 5);

                if (millis() - alignStart >
                    ALIGN_TIMEOUT)
                {
                    Serial.println(
                        "SETROBOT BW ALIGN RIGHT TIMEOUT"
                    );
                    break;
                }

                delay(3);
            }

            robot.motor('A', 30);
            robot.motor('C', -5);

            delay(13);
            break;
        }

        //---------------- Sensor 0 พบเส้นก่อน ----------------

        if (sensorLeft > mdLeft &&
            sensorRight < mdRight - 100)
        {
            unsigned long alignStart =
                millis();

            while (true)
            {
                const int leftValue =
                    robot.adcRead(9);

                if (leftValue <= deepLeft)
                {
                    break;
                }

                robot.motor('A', 5);
                robot.motor('C', -30);

                if (millis() - alignStart >
                    ALIGN_TIMEOUT)
                {
                    Serial.println(
                        "SETROBOT BW ALIGN LEFT TIMEOUT"
                    );
                    break;
                }

                delay(3);
            }

            robot.motor('A', -5);
            robot.motor('C', 30);

            delay(13);
            break;
        }

        //---------------- พบเส้นทั้งสอง ----------------

        if (sensorLeft < deepLeft &&
            sensorRight < deepRight)
        {
            robot.motor('A', 23);
            robot.motor('C', 23);

            delay(20);
            break;
        }

        //---------------- ถอยหลังรักษามุม ----------------

        robot.motor('A', leftSpeed);
        robot.motor('C', rightSpeed);

        //---------------- Timeout ----------------

        if (millis() - stageStart >
            FIND_LINE_TIMEOUT)
        {
            Serial.println(
                "SETROBOT BW FIND LINE TIMEOUT"
            );
            break;
        }

        delay(4);
    }

    //==================================================
    // ขั้นที่ 3 เดินหน้าออกจากเส้น
    //==================================================

    robot.motor('A', 5);
    robot.motor('C', 5);

    delay(100);

    robot.motor('A', 19);
    robot.motor('C', 19);

    delay(60);

    stageStart = millis();

    while (true)
    {
        robot.motor('A', 19);
        robot.motor('C', 19);

        const int sensorRight =
            robot.adcRead(0);

        const int sensorLeft =
            robot.adcRead(9);

        if (sensorRight > robot.adcMD(0) &&
            sensorLeft > robot.adcMD(9))
        {
            delay(10);

            robot.motor('A', -5);
            robot.motor('C', -5);

            delay(10);

            robot.motor('A', 0);
            robot.motor('C', 0);

            delay(10);
            break;
        }

        if (millis() - stageStart >
            RELEASE_TIMEOUT)
        {
            Serial.println(
                "SETROBOT BW RELEASE TIMEOUT"
            );

            robot.motor('A', 0);
            robot.motor('C', 0);
            break;
        }

        delay(3);
    }

    delay(200);

    //==================================================
    // ขั้นที่ 4 นับเส้นตามจำนวน _time
    //==================================================

    for (int i = 0; i < _time; i++)
    {
        stageStart = millis();

        while (true)
        {
            const int sensorRight =
                robot.adcRead(0);

            const int sensorLeft =
                robot.adcRead(9);

            const int mdRight =
                robot.adcMD(0);

            const int mdLeft =
                robot.adcMD(9);

            const int deepRight =
                (
                    robot.adcMD(0) +
                    robot.adcMin(0)
                ) / 2;

            const int deepLeft =
                (
                    robot.adcMD(9) +
                    robot.adcMin(9)
                ) / 2;

            //---------------- Line follow ----------------

            if (sensorLeft < mdLeft - 100 &&
                sensorRight > deepRight)
            {
                robot.motor('A', -19);
                robot.motor('C', 3);
            }
            else if (sensorLeft > mdLeft &&
                     sensorRight < deepRight)
            {
                robot.motor('A', 3);
                robot.motor('C', -19);
            }
            else if (sensorLeft < deepLeft &&
                     sensorRight < deepRight)
            {
                robot.motor('A', 10);
                robot.motor('C', 10);

                delay(20);
                break;
            }
            else
            {
                //---------------- รักษามุมขณะถอย ----------------

                imu.update();

                const float currentAngle =
                    imu.yaw();

                const float headingError =
                    constrainAngle180(
                        targetAngle -
                        currentAngle
                    );

                const float controlError =
                    -headingError;

                const unsigned long nowMicros =
                    micros();

                float dt =
                    (nowMicros - lastTime) *
                    0.000001f;

                lastTime =
                    nowMicros;

                dt =
                    constrain(
                        dt,
                        0.003f,
                        0.05f
                    );

                const float derivative =
                    (controlError - lastError) /
                    dt;

                dFilter =
                    KD_FILTER * dFilter +
                    (1.0f - KD_FILTER) *
                    derivative;

                lastError =
                    controlError;

                float correction =
                    HEADING_KP * controlError +
                    HEADING_KD * dFilter;

                correction =
                    constrain(
                        correction,
                        -15.0f,
                        15.0f
                    );

                int leftMotor =
                    -FOLLOW_SPEED -
                    (int)roundf(correction);

                int rightMotor =
                    -FOLLOW_SPEED +
                    (int)roundf(correction);

                leftMotor =
                    constrain(
                        leftMotor,
                        -100,
                        100
                    );

                rightMotor =
                    constrain(
                        rightMotor,
                        -100,
                        100
                    );

                robot.motor('A', leftMotor);
                robot.motor('C', rightMotor);
            }

            if (millis() - stageStart >
                COUNT_LINE_TIMEOUT)
            {
                Serial.print(
                    "SETROBOT BW COUNT TIMEOUT | "
                );

                Serial.println(i + 1);
                break;
            }

            delay(3);
        }

        //---------------- ออกจากเส้นก่อนรอบถัดไป ----------------

        if (i < _time - 1)
        {
            robot.motor('A', 19);
            robot.motor('C', 19);

            delay(20);

            stageStart = millis();

            while (true)
            {
                robot.motor('A', 19);
                robot.motor('C', 19);

                const int sensorRight =
                    robot.adcRead(0);

                const int sensorLeft =
                    robot.adcRead(9);

                if (sensorRight > robot.adcMD(0) ||
                    sensorLeft > robot.adcMD(9))
                {
                    delay(20);

                    robot.motor('A', 0);
                    robot.motor('C', 0);

                    delay(10);
                    break;
                }

                if (millis() - stageStart >
                    RELEASE_TIMEOUT)
                {
                    Serial.println(
                        "SETROBOT BW NEXT LINE TIMEOUT"
                    );

                    robot.motor('A', 0);
                    robot.motor('C', 0);
                    break;
                }

                delay(3);
            }
        }
    }

    //==================================================
    // ขั้นที่ 5 หาเส้นสุดท้าย
    //==================================================

    stageStart = millis();

    while (true)
    {
        const int sensorRight =
            robot.adcRead(0);

        const int sensorLeft =
            robot.adcRead(9);

        const int mdRight =
            robot.adcMD(0);

        const int mdLeft =
            robot.adcMD(9);

        const int deepRight =
            (
                robot.adcMD(0) +
                robot.adcMin(0)
            ) / 2;

        const int deepLeft =
            (
                robot.adcMD(9) +
                robot.adcMin(9)
            ) / 2;

        if (sensorLeft < mdLeft - 100 &&
            sensorRight > deepRight)
        {
            robot.motor('A', -19);
            robot.motor('C', 2);
        }
        else if (sensorLeft > mdLeft &&
                 sensorRight < deepRight)
        {
            robot.motor('A', 2);
            robot.motor('C', -19);
        }
        else if (sensorLeft < deepLeft &&
                 sensorRight < deepRight)
        {
            robot.motor('A', 5);
            robot.motor('C', 5);

            delay(50);
            break;
        }
        else
        {
            //---------------- รักษามุมขณะถอย ----------------

            imu.update();

            const float currentAngle =
                imu.yaw();

            const float headingError =
                constrainAngle180(
                    targetAngle -
                    currentAngle
                );

            const float controlError =
                -headingError;

            const unsigned long nowMicros =
                micros();

            float dt =
                (nowMicros - lastTime) *
                0.000001f;

            lastTime =
                nowMicros;

            dt =
                constrain(
                    dt,
                    0.003f,
                    0.05f
                );

            const float derivative =
                (controlError - lastError) /
                dt;

            dFilter =
                KD_FILTER * dFilter +
                (1.0f - KD_FILTER) *
                derivative;

            lastError =
                controlError;

            float correction =
                HEADING_KP * controlError +
                HEADING_KD * dFilter;

            correction =
                constrain(
                    correction,
                    -15.0f,
                    15.0f
                );

            int leftMotor =
                -18 -
                (int)roundf(correction);

            int rightMotor =
                -18 +
                (int)roundf(correction);

            leftMotor =
                constrain(leftMotor, -100, 100);

            rightMotor =
                constrain(rightMotor, -100, 100);

            robot.motor('A', leftMotor);
            robot.motor('C', rightMotor);
        }

        if (millis() - stageStart >
            COUNT_LINE_TIMEOUT)
        {
            Serial.println(
                "SETROBOT BW FINAL LINE TIMEOUT"
            );
            break;
        }

        delay(3);
    }

    //---------------- Final stop ----------------

    robot.motor('A', 0);
    robot.motor('C', 0);

    delay(15);
    delay(50);

    ch_line = false;

    //---------------- Final debug ----------------

    imu.update();

    const float finalAngle =
        imu.yaw();

    const float finalError =
        constrainAngle180(
            targetAngle - finalAngle
        );

    Serial.print("SETROBOT BW FINISHED");

    Serial.print(" | Target = ");
    Serial.print(targetAngle, 2);

    Serial.print(" | Final = ");
    Serial.print(finalAngle, 2);

    Serial.print(" | Error = ");
    Serial.println(finalError, 2);

    /*
       ห้ามเรียกคำสั่งเหล่านี้ท้ายฟังก์ชัน:

       gyro.resetAngles();
       imu.resetYaw();
       imu.zeroYaw();

       เพราะมุมต้องต่อเนื่องกับ turnGyro(),
       move(), setrobot_fw() และ move_untrasonic()
    */
}


void setrobot_bw_gyroset_zero(int _time)
{
    extern PiperPico2 robot;

    _set_b = true;

    if (_time < 0)
    {
        _time = 0;
    }

    //---------------- มุมเป้าหมาย ----------------

    /*
       ใช้มุมเป้าหมายล่าสุดจาก turnGyro()

       ตัวอย่าง:
       turnGyro(70, -90);

       แม้หมุนจริงได้ -87°
       targetAngle ยังคงเป็นประมาณ -90°
    */
    const float targetAngle =
        constrainAngle180(degree_turnGyro);

    //---------------- PID configuration ----------------

    const float HEADING_KP = 0.35f;
    const float HEADING_KI = 0.0f;
    const float HEADING_KD = 0.020f;

    const float KD_FILTER = 0.75f;
    const float MAX_CORRECTION = 30.0f;

    //---------------- Speed configuration ----------------

    const int HEADING_SPEED = 20;
    const int FOLLOW_SPEED = 19;

    //---------------- Timeout configuration ----------------

    const unsigned long LEAVE_LINE_TIMEOUT  = 3000UL;
    const unsigned long FIND_LINE_TIMEOUT   = 4000UL;
    const unsigned long ALIGN_TIMEOUT       = 2500UL;
    const unsigned long RELEASE_TIMEOUT     = 3000UL;
    const unsigned long COUNT_LINE_TIMEOUT  = 5000UL;

    //---------------- PID variables ----------------

    float integral = 0.0f;
    float lastError = 0.0f;
    float dFilter = 0.0f;

    unsigned long lastTime = micros();

    //---------------- Debug ----------------

    imu.update();

    Serial.print("SETROBOT BW START");

    Serial.print(" | Target = ");
    Serial.print(targetAngle, 2);

    Serial.print(" | Current = ");
    Serial.print(imu.yaw(), 2);

    Serial.print(" | Count = ");
    Serial.println(_time);

    //==================================================
    // ขั้นที่ 1 เดินหน้าออกจากเส้นเดิม
    //==================================================

    unsigned long stageStart = millis();

    while (true)
    {
        const int sensorRight =
            robot.adcRead(0);

        const int sensorLeft =
            robot.adcRead(9);

        robot.motor('A', 20);
        robot.motor('C', 20);

        const bool rightReleased =
            sensorRight >= robot.adcMD(0) - 100;

        const bool leftReleased =
            sensorLeft >= robot.adcMD(9) - 100;

        if (rightReleased && leftReleased)
        {
            break;
        }

        if (millis() - stageStart >
            LEAVE_LINE_TIMEOUT)
        {
            Serial.println(
                "SETROBOT BW LEAVE LINE TIMEOUT"
            );
            break;
        }

        delay(3);
    }

    // เบรกการเดินหน้า
    robot.motor('A', -5);
    robot.motor('C', -5);

    delay(25);

    //==================================================
    // ขั้นที่ 2 ถอยหลังรักษามุมจนพบเส้น
    //==================================================

    stageStart = millis();

    lastTime = micros();
    lastError = 0.0f;
    integral = 0.0f;
    dFilter = 0.0f;

    while (true)
    {
        //---------------- อ่าน IMU ----------------

        imu.update();

        const float currentAngle =
            imu.yaw();

        const float headingError =
            constrainAngle180(
                targetAngle - currentAngle
            );

        /*
           เนื่องจากกำลังถอยหลัง
           ต้องกลับทิศ Error สำหรับมอเตอร์
        */
        const float controlError =
            -headingError;

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

        if (fabs(controlError) < 20.0f)
        {
            integral +=
                controlError * dt;

            integral =
                constrain(
                    integral,
                    -60.0f,
                    60.0f
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
            HEADING_KP * controlError +
            HEADING_KI * integral +
            HEADING_KD * dFilter;

        correction =
            constrain(
                correction,
                -MAX_CORRECTION,
                MAX_CORRECTION
            );

        //---------------- Motor calculation ----------------

        /*
           ความเร็วฐานเป็นค่าลบเพราะถอยหลัง
        */
        int leftSpeed =
            -HEADING_SPEED -
            (int)roundf(correction);

        int rightSpeed =
            -HEADING_SPEED +
            (int)roundf(correction);

        leftSpeed =
            constrain(leftSpeed, -100, 100);

        rightSpeed =
            constrain(rightSpeed, -100, 100);

        //---------------- อ่านเซนเซอร์ ----------------

        const int sensorRight =
            robot.adcRead(0);

        const int sensorLeft =
            robot.adcRead(9);

        const int mdRight =
            robot.adcMD(0);

        const int mdLeft =
            robot.adcMD(9);

        const int deepRight =
            (
                robot.adcMD(0) +
                robot.adcMin(0)
            ) / 2;

        const int deepLeft =
            (
                robot.adcMD(9) +
                robot.adcMin(9)
            ) / 2;

        //---------------- Sensor 9 พบเส้นก่อน ----------------

        if (sensorLeft < mdLeft - 100 &&
            sensorRight > mdRight)
        {
            unsigned long alignStart =
                millis();

            while (true)
            {
                const int rightValue =
                    robot.adcRead(0);

                if (rightValue <= deepRight)
                {
                    break;
                }

                robot.motor('A', -20);
                robot.motor('C', 5);

                if (millis() - alignStart >
                    ALIGN_TIMEOUT)
                {
                    Serial.println(
                        "SETROBOT BW ALIGN RIGHT TIMEOUT"
                    );
                    break;
                }

                delay(3);
            }

            robot.motor('A', 20);
            robot.motor('C', -5);

            delay(13);
            break;
        }

        //---------------- Sensor 0 พบเส้นก่อน ----------------

        if (sensorLeft > mdLeft &&
            sensorRight < mdRight - 100)
        {
            unsigned long alignStart =
                millis();

            while (true)
            {
                const int leftValue =
                    robot.adcRead(9);

                if (leftValue <= deepLeft)
                {
                    break;
                }

                robot.motor('A', 5);
                robot.motor('C', -20);

                if (millis() - alignStart >
                    ALIGN_TIMEOUT)
                {
                    Serial.println(
                        "SETROBOT BW ALIGN LEFT TIMEOUT"
                    );
                    break;
                }

                delay(3);
            }

            robot.motor('A', -5);
            robot.motor('C', 20);

            delay(13);
            break;
        }

        //---------------- พบเส้นทั้งสอง ----------------

        if (sensorLeft < deepLeft &&
            sensorRight < deepRight)
        {
            robot.motor('A', 20);
            robot.motor('C', 20);

            delay(20);
            break;
        }

        //---------------- ถอยหลังรักษามุม ----------------

        robot.motor('A', leftSpeed);
        robot.motor('C', rightSpeed);

        //---------------- Timeout ----------------

        if (millis() - stageStart >
            FIND_LINE_TIMEOUT)
        {
            Serial.println(
                "SETROBOT BW FIND LINE TIMEOUT"
            );
            break;
        }

        delay(4);
    }

    //==================================================
    // ขั้นที่ 3 เดินหน้าออกจากเส้น
    //==================================================

    robot.motor('A', 5);
    robot.motor('C', 5);

    delay(100);

    robot.motor('A', 19);
    robot.motor('C', 19);

    delay(60);

    stageStart = millis();

    while (true)
    {
        robot.motor('A', 19);
        robot.motor('C', 19);

        const int sensorRight =
            robot.adcRead(0);

        const int sensorLeft =
            robot.adcRead(9);

        if (sensorRight > robot.adcMD(0) &&
            sensorLeft > robot.adcMD(9))
        {
            delay(10);

            robot.motor('A', -5);
            robot.motor('C', -5);

            delay(10);

            robot.motor('A', 0);
            robot.motor('C', 0);

            delay(10);
            break;
        }

        if (millis() - stageStart >
            RELEASE_TIMEOUT)
        {
            Serial.println(
                "SETROBOT BW RELEASE TIMEOUT"
            );

            robot.motor('A', 0);
            robot.motor('C', 0);
            break;
        }

        delay(3);
    }

    delay(200);

    //==================================================
    // ขั้นที่ 4 นับเส้นตามจำนวน _time
    //==================================================

    for (int i = 0; i < _time; i++)
    {
        stageStart = millis();

        while (true)
        {
            const int sensorRight =
                robot.adcRead(0);

            const int sensorLeft =
                robot.adcRead(9);

            const int mdRight =
                robot.adcMD(0);

            const int mdLeft =
                robot.adcMD(9);

            const int deepRight =
                (
                    robot.adcMD(0) +
                    robot.adcMin(0)
                ) / 2;

            const int deepLeft =
                (
                    robot.adcMD(9) +
                    robot.adcMin(9)
                ) / 2;

            //---------------- Line follow ----------------

            if (sensorLeft < mdLeft - 100 &&
                sensorRight > deepRight)
            {
                robot.motor('A', -19);
                robot.motor('C', 3);
            }
            else if (sensorLeft > mdLeft &&
                     sensorRight < deepRight)
            {
                robot.motor('A', 3);
                robot.motor('C', -19);
            }
            else if (sensorLeft < deepLeft &&
                     sensorRight < deepRight)
            {
                robot.motor('A', 10);
                robot.motor('C', 10);

                delay(20);
                break;
            }
            else
            {
                //---------------- รักษามุมขณะถอย ----------------

                imu.update();

                const float currentAngle =
                    imu.yaw();

                const float headingError =
                    constrainAngle180(
                        targetAngle -
                        currentAngle
                    );

                const float controlError =
                    -headingError;

                const unsigned long nowMicros =
                    micros();

                float dt =
                    (nowMicros - lastTime) *
                    0.000001f;

                lastTime =
                    nowMicros;

                dt =
                    constrain(
                        dt,
                        0.003f,
                        0.05f
                    );

                const float derivative =
                    (controlError - lastError) /
                    dt;

                dFilter =
                    KD_FILTER * dFilter +
                    (1.0f - KD_FILTER) *
                    derivative;

                lastError =
                    controlError;

                float correction =
                    HEADING_KP * controlError +
                    HEADING_KD * dFilter;

                correction =
                    constrain(
                        correction,
                        -15.0f,
                        15.0f
                    );

                int leftMotor =
                    -FOLLOW_SPEED -
                    (int)roundf(correction);

                int rightMotor =
                    -FOLLOW_SPEED +
                    (int)roundf(correction);

                leftMotor =
                    constrain(
                        leftMotor,
                        -100,
                        100
                    );

                rightMotor =
                    constrain(
                        rightMotor,
                        -100,
                        100
                    );

                robot.motor('A', leftMotor);
                robot.motor('C', rightMotor);
            }

            if (millis() - stageStart >
                COUNT_LINE_TIMEOUT)
            {
                Serial.print(
                    "SETROBOT BW COUNT TIMEOUT | "
                );

                Serial.println(i + 1);
                break;
            }

            delay(3);
        }

        //---------------- ออกจากเส้นก่อนรอบถัดไป ----------------

        if (i < _time - 1)
        {
            robot.motor('A', 19);
            robot.motor('C', 19);

            delay(20);

            stageStart = millis();

            while (true)
            {
                robot.motor('A', 19);
                robot.motor('C', 19);

                const int sensorRight =
                    robot.adcRead(0);

                const int sensorLeft =
                    robot.adcRead(9);

                if (sensorRight > robot.adcMD(0) ||
                    sensorLeft > robot.adcMD(9))
                {
                    delay(20);

                    robot.motor('A', 0);
                    robot.motor('C', 0);

                    delay(10);
                    break;
                }

                if (millis() - stageStart >
                    RELEASE_TIMEOUT)
                {
                    Serial.println(
                        "SETROBOT BW NEXT LINE TIMEOUT"
                    );

                    robot.motor('A', 0);
                    robot.motor('C', 0);
                    break;
                }

                delay(3);
            }
        }
    }

    //==================================================
    // ขั้นที่ 5 หาเส้นสุดท้าย
    //==================================================

    stageStart = millis();

    while (true)
    {
        const int sensorRight =
            robot.adcRead(0);

        const int sensorLeft =
            robot.adcRead(9);

        const int mdRight =
            robot.adcMD(0);

        const int mdLeft =
            robot.adcMD(9);

        const int deepRight =
            (
                robot.adcMD(0) +
                robot.adcMin(0)
            ) / 2;

        const int deepLeft =
            (
                robot.adcMD(9) +
                robot.adcMin(9)
            ) / 2;

        if (sensorLeft < mdLeft - 100 &&
            sensorRight > deepRight)
        {
            robot.motor('A', -19);
            robot.motor('C', 2);
        }
        else if (sensorLeft > mdLeft &&
                 sensorRight < deepRight)
        {
            robot.motor('A', 2);
            robot.motor('C', -19);
        }
        else if (sensorLeft < deepLeft &&
                 sensorRight < deepRight)
        {
            robot.motor('A', 5);
            robot.motor('C', 5);

            delay(50);
            break;
        }
        else
        {
            //---------------- รักษามุมขณะถอย ----------------

            imu.update();

            const float currentAngle =
                imu.yaw();

            const float headingError =
                constrainAngle180(
                    targetAngle -
                    currentAngle
                );

            const float controlError =
                -headingError;

            const unsigned long nowMicros =
                micros();

            float dt =
                (nowMicros - lastTime) *
                0.000001f;

            lastTime =
                nowMicros;

            dt =
                constrain(
                    dt,
                    0.003f,
                    0.05f
                );

            const float derivative =
                (controlError - lastError) /
                dt;

            dFilter =
                KD_FILTER * dFilter +
                (1.0f - KD_FILTER) *
                derivative;

            lastError =
                controlError;

            float correction =
                HEADING_KP * controlError +
                HEADING_KD * dFilter;

            correction =
                constrain(
                    correction,
                    -15.0f,
                    15.0f
                );

            int leftMotor =
                -18 -
                (int)roundf(correction);

            int rightMotor =
                -18 +
                (int)roundf(correction);

            leftMotor =
                constrain(leftMotor, -100, 100);

            rightMotor =
                constrain(rightMotor, -100, 100);

            robot.motor('A', leftMotor);
            robot.motor('C', rightMotor);
        }

        if (millis() - stageStart >
            COUNT_LINE_TIMEOUT)
        {
            Serial.println(
                "SETROBOT BW FINAL LINE TIMEOUT"
            );
            break;
        }

        delay(3);
    }

    //---------------- Final stop ----------------

    robot.motor('A', 0);
    robot.motor('C', 0);

    delay(15);
    delay(50);

    ch_line = false;

    //---------------- Final debug ----------------

    imu.update();

    const float finalAngle =
        imu.yaw();

    const float finalError =
        constrainAngle180(
            targetAngle - finalAngle
        );

    Serial.print("SETROBOT BW FINISHED");

    Serial.print(" | Target = ");
    Serial.print(targetAngle, 2);

    Serial.print(" | Final = ");
    Serial.print(finalAngle, 2);

    Serial.print(" | Error = ");
    Serial.println(finalError, 2);

    /*
       ห้ามเรียกคำสั่งเหล่านี้ท้ายฟังก์ชัน:

       gyro.resetAngles();
       imu.resetYaw();
       imu.zeroYaw();

       เพราะมุมต้องต่อเนื่องกับ turnGyro(),
       move(), setrobot_fw() และ move_untrasonic()
    */
    delay(200);
    imu.update();
    imu.resetAngles();
    delay(100);
}