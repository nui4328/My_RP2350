const float LTRACK_WIDTH_CM = 14.5;
const float RTRACK_WIDTH_CM = 14.1;

long degreeToTicks_LeftPivot(float degree)
{
    float distance = 2.0 * 3.141592653589793 * LTRACK_WIDTH_CM * (degree / 360.0);
    return (long)(distance / CM_PER_TICK);
}

long degreeToTicks_RightPivot(float degree)
{
    float distance = 2.0 * 3.141592653589793 * RTRACK_WIDTH_CM * (degree / 360.0);
    return (long)(distance / CM_PER_TICK);
}
void turn_left_in(int speed, int degree, int offset)
{
    encoder.resetEncoders();

    long targetTicks = degreeToTicks_LeftPivot(degree);

    int slowSpeed = max(15, speed / 3);

    // เริ่ม slow ใกล้เป้ามากขึ้น
    long slowStart = targetTicks * 0.10;

    while (true)
    {
        long leftTick = abs(encoder.Poss_R());

        if (leftTick >= targetTicks)
            break;

        long remain = targetTicks - leftTick;

        if (remain < slowStart)
            Motor(-1, slowSpeed);
        else
            Motor(-1, speed);
    }

    // brake
    Motor(1, -speed);
    delay(offset);

    Motor(1, 1);
}

void turn_left_out(int speed, int degree, int offset)
{
    encoder.resetEncoders();

    long targetTicks = degreeToTicks_LeftPivot(degree);

    int slowSpeed = max(15, speed / 3);

    // เริ่ม slow ใกล้เป้ามากขึ้น
    long slowStart = targetTicks * 0.10;

    while (true)
    {
        long leftTick = abs(encoder.Poss_R());

        if (leftTick >= targetTicks)
            break;

        long remain = targetTicks - leftTick;

        if (remain < slowStart)
            Motor(1, -slowSpeed);
        else
            Motor(1, -speed);
    }

    // brake
    Motor(1, speed);
    delay(offset);

    Motor(1, 1);
}


void turn_right_in(int speed, int degree, int offset)
{
    encoder.resetEncoders();

    long targetTicks = degreeToTicks_RightPivot(degree);

    int slowSpeed = max(15, speed / 3);

    // เริ่ม slow ใกล้เป้ามากขึ้น
    long slowStart = targetTicks * 0.10;

    while (true)
    {
        long leftTick = abs(encoder.Poss_L());

        if (leftTick >= targetTicks)
            break;

        long remain = targetTicks - leftTick;

        if (remain < slowStart)
            Motor(slowSpeed, -1 );
        else
            Motor(speed, -1 );
    }

    // brake
    Motor(-speed, 1 );
    delay(offset);

    Motor(1, 1);
}

void turn_right_out(int speed, int degree, int offset)
{
    encoder.resetEncoders();

    long targetTicks = degreeToTicks_LeftPivot(degree);

    int slowSpeed = max(15, speed / 3);

    // เริ่ม slow ใกล้เป้ามากขึ้น
    long slowStart = targetTicks * 0.10;

    while (true)
    {
        long leftTick = abs(encoder.Poss_L());

        if (leftTick >= targetTicks)
            break;

        long remain = targetTicks - leftTick;

        if (remain < slowStart)
            Motor(-slowSpeed, 1 );
        else
            Motor(-speed, 1 );
    }

    // brake
    Motor(speed, -1 );
    delay(offset);

    Motor(1, 1);
}

