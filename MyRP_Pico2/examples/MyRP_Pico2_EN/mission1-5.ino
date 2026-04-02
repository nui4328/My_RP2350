// ====== ค่าคงที่ ======
const float WHEEL_DIAMETER_CM = 4.6;
//const float WHEEL_CIRCUMFERENCE_CM = 3.141592653589793 * WHEEL_DIAMETER_CM;
//const int   TICKS_PER_REVOLUTION = 25;
//const float CM_PER_TICK = WHEEL_CIRCUMFERENCE_CM / TICKS_PER_REVOLUTION;

const float TRACK_WIDTH_CM = 13.5;

// TURN_GAIN สำหรับ speed สูง → หมุนเป๊ะ ±1°
// TURN_GAIN แยกทิศทาง
const float TURN_GAIN_FORWARD  = 1.16; // หมุนไป
const float TURN_GAIN_BACKWARD = 1.14; // หมุนกลับ

void turn_encoder(int spl, int spr, int degree, int offset)
{
    if (degree <= 0) { Motor(0,0); return; }

    encoder.resetEncoders();

    float distance_cm = 3.141592653589793 * TRACK_WIDTH_CM * (degree / 360.0);

    // ====== เลือก TURN_GAIN ตามทิศทาง ======
    float gain = (spl < 0 || spr > 0) ? TURN_GAIN_FORWARD : TURN_GAIN_BACKWARD;

    long targetTicks = (long)((distance_cm / CM_PER_TICK) * gain);

    unsigned long start_time = millis();
    long slowStart = targetTicks * 0.5;

    while(true) {
        long currentTicks = (abs(encoder.Poss_L()) + abs(encoder.Poss_R()))/2;
        if(currentTicks >= targetTicks) break;
        long remain = targetTicks - currentTicks;

        int currentSpeedL, currentSpeedR;

        if(remain < slowStart) {
            float ratio = (float)remain / slowStart;
            if(ratio < 0.40f) ratio = 0.40f;
            currentSpeedL = (int)(spl * ratio);
            currentSpeedR = (int)(spr * ratio);
        } else {
            currentSpeedL = spl;
            currentSpeedR = spr;
        }

        Motor(currentSpeedL, currentSpeedR);

        if(millis() - start_time > 5000UL) break;
        delayMicroseconds(20);
    }

    // เบรกสั้น
    Motor(-max(1,spl/8), -max(1,spr/8));
    delay(offset);
    if(spl <= 0 && spr > 0)
        {
            Motor(1,-1);
        }
    else
        {
            Motor(-1,1);
        }
    
}