#include "MoveGyroPID.h"


#define ARM_UP_STOP_EARLY_START_CM  1.5f
#define ARM_UP_STOP_EARLY_RUN_CM    0.6f
#define ARM_DOWN_STOP_EARLY_CM      0.7f

#define ARM_HOME_ADC_THRESHOLD      1500
#define ARM_HOMING_DOWN_SPEED       35
#define ARM_HOMING_BACKUP_SPEED     18
#define ARM_BACKUP_PULSES           20

void arm_updown(float target_cm)
{
    target_cm = constrain(target_cm, 0.0f, 27.0f);

    last_PULSES_up = encoder2.Poss_R();

    arm_updown_nonblocking(target_cm);

    unsigned long start_time = millis();
    if(target_cm == 0)
        {
            arm_shoulder('L', 70);  // ค่าเดิมคือ65
            arm_shoulder('R', 70); 
        }

    while (arm_moving && (millis() - start_time < 15000UL)) {
        arm_update_nonblocking();
        delay(5);
    }

    if (arm_moving) {
        Serial.println("ARM TIMEOUT!");
        robot.motor('D', 0);
        arm_moving = false;
        arm_homing_mode = false;
    }
}

void arm_updown_nonblocking(float target_cm)
{
    target_cm = constrain(target_cm, 0.0f, 27.0f);

    if (arm_moving) return;

    // ---------- HOMING ----------
    if (fabs(target_cm) < 0.01f) {
        arm_homing_mode = true;
        homing_state = HOMING_DOWN;
        arm_moving = true;

        robot.motor('D', -ARM_HOMING_DOWN_SPEED);

        arm_last_time = millis();
        arm_last_move_time = millis();
        arm_last_pulses_stuck = encoder2.Poss_R();

        Serial.println("=== ARM HOMING STARTED ===");
        return;
    }

    float delta_raw_cm = target_cm - current_position_cm;

    if (fabs(delta_raw_cm) < 0.08f) {
        Serial.println("Arm already near target.");
        return;
    }

    bool going_up = delta_raw_cm > 0.0f;
    float delta_cm = delta_raw_cm;

    // ---------- COMPENSATION ----------
    if (going_up) {
        if (current_position_cm < 2.0f) {
            delta_cm -= ARM_UP_STOP_EARLY_START_CM;
        } else {
            delta_cm -= ARM_UP_STOP_EARLY_RUN_CM;
        }

        if (delta_cm < 0.3f) delta_cm = 0.3f;
    } 
    else {
        delta_cm += ARM_DOWN_STOP_EARLY_CM;

        if (delta_cm > -0.3f) delta_cm = -0.3f;
    }

    long start_pulses = encoder2.Poss_R();
    long move_pulses = (long)(fabs(delta_cm) * PULSES_PER_CM + 0.5f);

    if (move_pulses < 5) {
        Serial.println("Move pulse too small.");
        return;
    }

    arm_goal_pulses = start_pulses + (going_up ? move_pulses : -move_pulses);
    arm_direction = going_up ? 1 : -1;
    arm_moving = true;
    arm_homing_mode = false;

    int start_speed = going_up ? 42 : 28;
    robot.motor('D', arm_direction * start_speed);

    arm_last_time = millis();
    arm_last_move_time = millis();
    arm_last_pulses_stuck = start_pulses;

    Serial.printf(
        "Arm Target %.2f cm | Current %.2f cm | RawDelta %.2f | MoveDelta %.2f | Dir %s\n",
        target_cm,
        current_position_cm,
        delta_raw_cm,
        delta_cm,
        going_up ? "UP" : "DOWN"
    );
}

void arm_update_nonblocking()
{
    if (!arm_moving) return;

    int adc_value = robot.adcRead(6);
    long current_pulses = encoder2.Poss_R();

    current_position_cm = (float)current_pulses / PULSES_PER_CM;

    // ---------- HOMING ----------
    if (arm_homing_mode) {
        switch (homing_state) {
            case HOMING_DOWN:
                if (adc_value >= ARM_HOME_ADC_THRESHOLD) {
                    robot.motor('D', 0);

                    encoder2.resetEncoders();
                    arm_last_time = millis();

                    robot.motor('D', ARM_HOMING_BACKUP_SPEED);

                    homing_backup_start = encoder2.Poss_R();
                    homing_state = HOMING_BACKUP;
                }
                else if (millis() - arm_last_time > 8) {
                    robot.motor('D', -ARM_HOMING_DOWN_SPEED);
                    arm_last_time = millis();
                }
                break;

            case HOMING_BACKUP:
                current_pulses = encoder2.Poss_R();

                if (current_pulses >= ARM_BACKUP_PULSES) {
                    robot.motor('D', 0);

                    encoder2.resetEncoders();
                    current_position_cm = 0.0f;

                    arm_moving = false;
                    arm_homing_mode = false;

                    Serial.println("=== HOMING COMPLETED - Arm at 0 cm ===");
                }
                else if (millis() - arm_last_time > 10) {
                    robot.motor('D', ARM_HOMING_BACKUP_SPEED);
                    arm_last_time = millis();
                }
                break;
        }

        return;
    }

    // ---------- STUCK CHECK ----------
    long delta_pulses = labs(current_pulses - arm_last_pulses_stuck);

    if (delta_pulses >= ARM_MIN_PULSES) {
        arm_last_move_time = millis();
        arm_last_pulses_stuck = current_pulses;
    }
    else if (millis() - arm_last_move_time > ARM_STUCK_TIME_MS) {
        Serial.println("ARM STUCK!");
        robot.motor('D', 0);
        arm_moving = false;
        return;
    }

    // ---------- REMAIN ----------
    long remain_pulses;

    if (arm_direction > 0) {
        remain_pulses = arm_goal_pulses - current_pulses;
    } else {
        remain_pulses = current_pulses - arm_goal_pulses;
    }

    // ---------- REACHED ----------
    if (remain_pulses <= 5) {
        int brake = (arm_direction > 0) ? 20 : 32;

        robot.motor('D', -arm_direction * brake);
        delay(20);
        robot.motor('D', 0);

        current_position_cm = (float)encoder2.Poss_R() / PULSES_PER_CM;

        arm_moving = false;

        Serial.printf("Arm ARRIVED at %.2f cm\n", current_position_cm);
        return;
    }

    // ---------- SPEED CONTROL ----------
    int power;

    if (arm_direction > 0) {
        power = 40;

        if (remain_pulses < 55) {
            power = 18;
        }
        else if (remain_pulses < 120) {
            power = 24;
        }
        else if (remain_pulses < 180) {
            power = 30;
        }
    }
    else {
        power = 28;

        if (remain_pulses < 55) {
            power = 20;
        }
        else if (remain_pulses < 120) {
            power = 23;
        }
        else if (remain_pulses < 180) {
            power = 25;
        }
    }

    if (millis() - arm_last_time > 6) {
        robot.motor('D', arm_direction * power);
        arm_last_time = millis();
    }
}



// =============================
// MOVE FUNCTION
// =============================
void move(int sl, int sr, float kp, float dist, int offset, float arm_target)
{
    extern PiperPico2 robot;
    extern my_BMI160 gyro;
    extern EncoderLibrarys encoder1;
    extern EncoderLibraryss encoder2;

    if (dist <= 0.1f) return;
    if(offset == 0)
        {
            dist -= 5;
        }
    if (_set_b == true || _set_f == true ) 
        {
            dist += 6;                        
        }
    if(arm_target < 3)
        {
            arm_shoulder('L', 76);  // ค่าเดิมคือ65
            arm_shoulder('R', 62);
        }
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

    const int MIN_SPEED = 32;
    float maxSpeed = (abs(leftBase) + abs(rightBase)) / 2.0f;
    const float ACC_DIST = 15.0f;
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
        
        if(offset > 0)
            {
                _offset = true;
                if (remain <= 9.0f)
                {
                    if (remain > 4.0f) {
                        robot.motor('A', (int)(leftBase * 0.48f));
                        robot.motor('C', (int)(rightBase * 0.48f));
                        delay(12);
                    } else {
                        if(break_move == 1) {
                            float brakeFactor = 0.13f;
                            robot.motor('A', (int)(-leftBase * brakeFactor));
                            robot.motor('C', (int)(-rightBase * brakeFactor));
                            delay(offset);
                        }
                        robot.motor('A', 0);
                        robot.motor('C', 0);
                        break;
                    }
                }
            }
        else    
            {
                _offset = false;
                if (remain <= 9.0f)
                {
                    if (remain > 4.0f) {
                        robot.motor('A', (int)(leftBase * 0.48f));
                        robot.motor('C', (int)(rightBase * 0.48f));
                        delay(12);
                    } else {
                        if(break_move == 1) {
                           robot.motor('A', 0);
                           robot.motor('C', 0);
                        }
                        
                        break;
                    }
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

        //float derivative = (error - lastError) / dt;
        float derivative = error - lastError;
        lastError = error;

        float correction = kp * error  + move_kd * derivative;
        float speedFactor = constrain((abs(baseLeft) + abs(baseRight)) / 80.0f, 0.33f, 1.0f);
        correction *= speedFactor;
        correction = constrain(correction, -30.0f, 30.0f);

        int left  = baseLeft  + (movingBackward ? -correction : correction);
        int right = baseRight - (movingBackward ? -correction : correction);

        left  = constrain(left,  -100, 100);
        right = constrain(right, -100, 100);

        // Line Sensor
        if(sl > 0) {
            if (robot.adcRead(8) < robot.adcMD(8) && robot.adcRead(1) > robot.adcMD(1)) {
                robot.motor('A', left*0.7); robot.motor('C', right*1.4);
                gyro.resetAngles();
            } else if (robot.adcRead(8) > robot.adcMD(8) && robot.adcRead(1) < robot.adcMD(1)) {
                robot.motor('A', left*1.4); robot.motor('C', right*0.7);
                gyro.resetAngles();
            } else {
                robot.motor('A', left); robot.motor('C', right);     
            } 
        } else {
            if (robot.adcRead(9) < robot.adcMD(9) && robot.adcRead(0) > robot.adcMD(0)) {
                robot.motor('A', left*0.7); robot.motor('C', right*1.4);
                gyro.resetAngles();
            } else if (robot.adcRead(9) > robot.adcMD(9) && robot.adcRead(0) < robot.adcMD(0)) {
                robot.motor('A', left*1.4); robot.motor('C', right*0.7);
                gyro.resetAngles();
            } else {
                robot.motor('A', left); robot.motor('C', right);     
            } 
        }

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

    //Serial.printf("🏁 move() FINISHED → Arm Position = %.2f cm\n", current_position_cm);
}
void move_bridge(int sl, int sr, float kp, float dist, int offset, float arm_target)
{
    extern PiperPico2 robot;
    extern my_BMI160 gyro;
    extern EncoderLibrarys encoder1;
    extern EncoderLibraryss encoder2;

    if (dist <= 0.1f) return;
    if(offset == 0)
        {
            dist -= 5;
        }
    if (_set_b == true || _set_f == true ) 
        {
            dist += 6;                        
        }

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

    const int MIN_SPEED = 32;
    float maxSpeed = (abs(leftBase) + abs(rightBase)) / 2.0f;
    const float ACC_DIST = 15.0f;
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
        
        if(offset > 0)
            {
                _offset = true;
                if (remain <= 9.0f)
                {
                    if (remain > 4.0f) {
                        robot.motor('A', (int)(leftBase * 0.48f));
                        robot.motor('C', (int)(rightBase * 0.48f));
                        delay(12);
                    } else {
                        if(break_move == 1) {
                            float brakeFactor = 0.38f;
                            robot.motor('A', (int)(-leftBase * brakeFactor));
                            robot.motor('C', (int)(-rightBase * brakeFactor));
                            delay(offset);
                        }
                        robot.motor('A', 0);
                        robot.motor('C', 0);
                        break;
                    }
                }
            }
        else    
            {
                _offset = false;
                if (remain <= 9.0f)
                {
                    if (remain > 4.0f) {
                        robot.motor('A', (int)(leftBase * 0.48f));
                        robot.motor('C', (int)(rightBase * 0.48f));
                        delay(12);
                    } else {
                        if(break_move == 1) {
                           robot.motor('A', 0);
                           robot.motor('C', 0);
                        }
                        
                        break;
                    }
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

        //float derivative = (error - lastError) / dt;
        float derivative = error - lastError;
        lastError = error;

        float correction = kp * error  + move_kd * derivative;
        float speedFactor = constrain((abs(baseLeft) + abs(baseRight)) / 80.0f, 0.33f, 1.0f);
        correction *= speedFactor;
        correction = constrain(correction, -30.0f, 30.0f);

        int left  = baseLeft  + (movingBackward ? -correction : correction);
        int right = baseRight - (movingBackward ? -correction : correction);

        left  = constrain(left,  -100, 100);
        right = constrain(right, -100, 100);

        // Line Sensor
        if(sl > 0) {
            if (robot.adcRead(8) < (robot.adcMD(8)+robot.adcMin(8))/2 && robot.adcRead(1) > robot.adcMD(1)) {
                robot.motor('A', left*0.7); robot.motor('C', right*1.4);
                gyro.resetAngles();
            } else if (robot.adcRead(8) > robot.adcMD(8) && robot.adcRead(1) < (robot.adcMD(1)+robot.adcMin(1))/2) {
                robot.motor('A', left*1.4); robot.motor('C', right*0.7);
                gyro.resetAngles();
            } else {
                robot.motor('A', left); robot.motor('C', right);     
            } 
        } else {
            if (robot.adcRead(9) < (robot.adcMD(9)+robot.adcMin(9))/2 && robot.adcRead(0) > robot.adcMD(0)) {
                robot.motor('A', left*0.7); robot.motor('C', right*1.4);
                gyro.resetAngles();
            } else if (robot.adcRead(9) > robot.adcMD(9) && robot.adcRead(0) < (robot.adcMD(0)+robot.adcMin(0))/2) {
                robot.motor('A', left*1.4); robot.motor('C', right*0.7);
                gyro.resetAngles();
            } else {
                robot.motor('A', left); robot.motor('C', right);     
            } 
        }

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

    //Serial.printf("🏁 move() FINISHED → Arm Position = %.2f cm\n", current_position_cm);
}


void move_chopsticks(int sl, int sr, float kp, float dist, float arm_target)
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

    const int MIN_SPEED = 22;
    float maxSpeed = (abs(leftBase) + abs(rightBase)) / 2.0f;
    const float ACC_DIST = 15.0f;
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
                robot.motor('A', (int)(leftBase * 0.48f));
                robot.motor('C', (int)(rightBase * 0.48f));
                delay(12);
            } else {
                if(break_move == 1) {
                    float brakeFactor = 0.38f;
                    robot.motor('A', (int)(-leftBase * brakeFactor));
                    robot.motor('C', (int)(-rightBase * brakeFactor));
                    delay(14);
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

        float correction = kp * error + move_ki * integral + move_kd * derivative;
        float speedFactor = constrain((abs(baseLeft) + abs(baseRight)) / 80.0f, 0.33f, 1.0f);
        correction *= speedFactor;
        correction = constrain(correction, -30.0f, 30.0f);

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
    robot.motor('A',-5);
    robot.motor('C',-5 );
    delay(10);
    robot.motor('A', 0);
    robot.motor('C', 0);
    robot.motor('D', 0);
    delay(100);

    arm_moving = false;
    arm_homing_mode = false;
    _set_f = false;
    _set_b = false;
    break_move = 1;

    Serial.printf("🏁 move_untrasonic() FINISHED → Distance = %.1f cm (Target = %.1f cm)\n", 
                  currentDistance, untra);
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
            
            float slowFactor = 0.65f + 0.55f * p;       // ชะลอลงเหลือประมาณ 45% ของความเร็ว
            
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
