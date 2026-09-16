// รายการคำสั่งของ My Mini Pro สำหรับคัดลอกไปใช้ในแท็บ MyMiniPro
// ทุกบรรทัดเป็นความคิดเห็น จึงไม่มีคำสั่งทำงานเองในแท็บนี้
// เปลี่ยนหมายเลขช่อง มุม และความเร็วให้เหมาะกับงาน

// ===== เริ่มระบบ =====
// robot.begin();                 // ใช้ครั้งเดียวใน setup()
// robot.begin(false);            // ไม่เล่นเพลงเปิดเครื่อง แต่ยังมีเสียงรีเซ็ต

// ===== เซ็นเซอร์ Front / Rear: ช่อง 0..15 =====
// robot.readSensorFront(0);      // อ่านค่า raw; คำสั่งนี้สแกนชุด Front ใหม่
// robot.minSensorFront(0);       // ค่าต่ำสุดที่คาลิเบรตไว้
// robot.maxSensorFront(0);       // ค่าสูงสุดที่คาลิเบรตไว้
// robot.readSensorRear(0);
// robot.minSensorRear(0);
// robot.maxSensorRear(0);
// robot.setSensorSmoothing(50);  // 100 = ไม่กรอง; ค่าน้อย = เรียบขึ้นแต่ช้าลง

// สแกนหนึ่งครั้งแล้วอ่านหลายช่องจากชุดข้อมูลเดียวกัน
// robot.scanFront();
// robot.scanRear();
// robot.scan(MyMiniPro::Direction::Forward);  // สแกน Front
// robot.scan(MyMiniPro::Direction::Reverse);  // สแกน Rear
// robot.normalized(MyMiniPro::Array::Front, 0); // 0..1000 หลัง scanFront()
// robot.normalized(MyMiniPro::Array::Rear, 0);  // 0..1000 หลัง scanRear()
// robot.values(MyMiniPro::Array::Front, 0);     // current, maximum, minimum
// robot.values(MyMiniPro::Array::Rear, 0);
// robot.minValues(MyMiniPro::Array::Front);    // อาร์เรย์อ่านอย่างเดียว 16 ค่า
// robot.maxValues(MyMiniPro::Array::Front);
// robot.minValues(MyMiniPro::Array::Rear);
// robot.maxValues(MyMiniPro::Array::Rear);
// robot.printLiveReadings(Serial); // สแกนและพิมพ์ Front/Rear/ADC ใต้ท้อง

// ===== คาลิเบรต Front / Rear และ EEPROM =====
// หยุดมอเตอร์และเลื่อนเซ็นเซอร์ผ่านดำ/ขาวตลอดช่วงคาลิเบรต
// robot.startCalibration(MyMiniPro::Array::Front); // เรียกครั้งเดียวเพื่อเริ่ม
// robot.startCalibration(MyMiniPro::Array::Rear);
// robot.serviceCalibration();    // เรียกต่อเนื่องใน loop() จนเสร็จ
// robot.calibrationActive();
// robot.calibrationArray();
// robot.hasCalibration(MyMiniPro::Array::Front);
// robot.hasCalibration(MyMiniPro::Array::Rear);
// robot.saveCalibration();       // เขียน EEPROM; ไม่ควรเรียกทุก loop()
// robot.loadCalibration();       // begin() โหลดให้อยู่แล้ว
// robot.rearCalibrationButtonValue(); // ค่า raw จาก MCP3421

// ===== ADS1115: เซ็นเซอร์ใต้ท้องและช่องเสริม =====
// ค่า raw เป็น signed; INT16_MIN หมายถึงอ่านไม่สำเร็จ
// robot.readAdcL();               // AIN1: เซ็นเซอร์ใต้ท้องซ้าย
// robot.minAdcL();
// robot.maxAdcL();
// robot.readAdcR();               // AIN2: เซ็นเซอร์ใต้ท้องขวา
// robot.minAdcR();
// robot.maxAdcR();
// robot.readAdcLNormalized();     // 0..1000; 65535 = ยังไม่คาลิเบรต/อ่านผิดพลาด
// robot.readAdcRNormalized();
// robot.readAds1115Ain3Raw();     // AIN3: ช่องเสริม
// robot.readAds1115Raw(3);        // เลือก AIN0..AIN3; AIN0 ใช้วัดแบตเตอรี่
// robot.readAds1115Voltage(3);    // แรงดันที่ขา ADC; NAN เมื่ออ่านผิดพลาด
// robot.setAds1115Address(0x48);

// ===== คาลิเบรตเซ็นเซอร์ใต้ท้อง =====
// robot.startUnderbodyCalibration(); // เรียกครั้งเดียวเพื่อเริ่ม
// robot.serviceUnderbodyCalibration(); // เรียกต่อเนื่องใน loop() จนเสร็จ
// robot.underbodyCalibrationActive();
// robot.hasUnderbodyCalibration();
// robot.adcLMinimum();            // ชื่อเต็มของ minAdcL()
// robot.adcLMaximum();
// robot.adcRMinimum();
// robot.adcRMaximum();

// ===== ปุ่ม Start =====
// while (!robot.waitButton()) {} // ใช้รอใน setup(); มีการแสดงค่าเซ็นเซอร์
// while (!robot.waitButton(Serial, 100)) {}
// robot.serviceStartButton();    // อีกทางเลือก: ตรวจปุ่มเป็นรอบ ๆ
// robot.robotStarted();
// robot.stopMotors();
// robot.stopRobot();             // ล้างสถานะเริ่มงาน; ไม่ใช่คำสั่งหยุดมอเตอร์

// ===== มอเตอร์: ซ้าย, ขวา ช่วง -100..100 =====
// เมื่อต้องการทดสอบมอเตอร์ ให้ปรับคำสั่ง stopMotors() ใน loop() แท็บหลักด้วย
// robot.Motor(30, 30);            // เดินหน้า
// robot.Motor(-30, -30);          // ถอยหลัง
// robot.Motor(0, 30);             // ล้อซ้ายหยุด ล้อขวาเดินหน้า
// robot.Motor(30, 0);             // ล้อซ้ายเดินหน้า ล้อขวาหยุด
// robot.Motor(0, 0);
// robot.stopMotors();
// robot.leftMotorCommand();       // คำสั่งล่าสุด ไม่ใช่ RPM ที่วัดได้
// robot.rightMotorCommand();

// ===== เซอร์โว: GPIO 18, 22, 28, 0, 1 =====
// begin() ไม่ขยับเซอร์โว; servo() จะ attach เมื่อเรียกครั้งแรก
// ใช้แหล่งจ่ายเซอร์โวภายนอกและต่อกราวด์ร่วม
// robot.servo(18, 90);            // GPIO, มุม 0..180 องศา
// robot.servo(22, 90);
// robot.servo(28, 90);
// robot.servo(0, 90);
// robot.servo(1, 90);
// robot.setServoPulseLimits(18, 1000, 2000); // microseconds; ตั้งก่อน attach
// robot.detachServo(18);
// robot.servoAttached(18);
// robot.servoAngle(18);           // -1 เมื่อยังไม่ได้ attach

// ===== บัซเซอร์ =====
// robot.playStartupMelody();
// robot.playTone(1500);           // เล่นต่อเนื่องจน stopBuzzer()
// robot.beep(2000, 120);          // ความถี่ Hz, ระยะเวลา ms
// robot.stopBuzzer();
// robot.updateBuzzer();          // เรียกใน loop() เพื่อเดินเพลง/เสียงตามเวลา
// robot.buzzerActive();

// ===== แบตเตอรี่และ LED 8 ดวง =====
// robot.readBatteryVoltage();
// robot.readBatteryVoltage(4.0); // ระบุอัตราส่วนวงจรแบ่งแรงดัน
// robot.setBatteryDividerRatio(4.0);
// robot.setBatteryCalibration(0.9638554, 0.4963855); // gain, offset ของบอร์ดเดิม
// robot.beginBatteryLevelLeds(0x20, false); // address, activeLow; เรียกครั้งเดียว
// robot.updateBatteryLevelLeds();          // อ่านแบตแล้วอัปเดตไฟ
// robot.updateBatteryLevelLeds(12.0);     // ใช้แรงดันที่อ่านไว้แล้ว
// MyMiniPro::batteryLevelLedCount(12.0);  // แปลงแรงดันเป็นจำนวน LED

// ===== ชดเชยแรงดันสำหรับคำสั่งมอเตอร์ =====
// robot.setMotorVoltageCompensation(true, 12.4, 500, 5);
// robot.serviceMotorVoltageCompensation(); // เรียกต่อเนื่องเพื่ออัปเดต gain
// robot.motorVoltageCompensationGain();
// int speed = robot.compensateBaseMotorSpeed(30);
// robot.Motor(speed, speed);
// การชดเชยใช้ ADS1115 ร่วมกับเซ็นเซอร์ใต้ท้อง: ไม่เรียกอ่าน ADC แบบอื่น
// สลับระหว่างรอบเริ่มและรอ conversion ของ service นี้


// ===== F_Line: COPYABLE REFERENCE (four arguments; uncomment after copying) =====
// GLOBAL: place below #include <Pico2_MyMiniPro.h> in the MyMiniPro tab:
// using Sensor = MyMiniProForward::Sensor;
// The main tab already declares Pico2MyMiniPro robot; do not duplicate it.
//
// SETUP: after robot.begin(), before the while (!robot.waitButton(...)) line:
// robot.setMuxSettleMicroseconds(400); // then recalibrate FRONT at this setting
// robot.setFLineSensorDebounceMs(0); // first qualifying scan; does NOT change400us
// Serial.println(robot.fLineSensorDebounceMs()); // line confirmation in ms
// robot.setFLineKd(0.001); // setup; any finite signed KD; NaN/Inf rejected
// Serial.println(robot.fLineKd(), 4); // read the configured KD
// For P-only compatibility, replace0.001 above with0.0f.
// MyMiniPro::ForwardSettings forward;
// forward.maximumRunMs = 1500; // distance mode only
// forward.sensorMaximumRunMs = 0; // default: no sensor timeout
// For a bounded bench test ONLY: forward.sensorMaximumRunMs = 1500;
// forward.centimetersPerSecondPerPercent = 0.0; // replace with measured cm/s / percent
// robot.setForwardSettings(forward);
//
// SETUP: AFTER waitButton, REPLACE the existing movement call with ONE call:
// auto reason = robot.F_Line(20, 20,0.1, Sensor::f15);
// auto reason = robot.F_Line(20, 20,0.1, Sensor::cl); // ADC L calibration, AIN1
// auto reason = robot.F_Line(20, 20,0.1, Sensor::cr); // ADC R calibration, AIN2
// auto reason = robot.F_Line(20, 20,0.1, Sensor::b0); // Rear channel 0
// auto reason = robot.F_Line(20, 20,0.1, Sensor::b15); // Rear channel 15
// auto reason = robot.F_Line(20, 20,0.1, 2); // 2cm; requires measured coefficient
// Serial.println(MyMiniProForward::resultName(reason));
// robot.stopMotors();
//
// LOOP: leave empty or use only robot.updateBuzzer(); for this single-run test.
// Do not copy F_Line directly into loop(): it would repeat movement forever.
// Sensor names (always keep Sensor:: to avoid Arduino's b0 binary constant):
// Sensor::f0, Sensor::f1, Sensor::f2, Sensor::f3, Sensor::f4, Sensor::f5,
// Sensor::f6, Sensor::f7, Sensor::f8, Sensor::f9, Sensor::f10, Sensor::f11,
// Sensor::f12, Sensor::f13, Sensor::f14, Sensor::f15 = Front channels 0..15.
// Sensor::cl, Sensor::cr = calibrated ADC L/R.
// Sensor::b0, Sensor::b1, Sensor::b2, Sensor::b3, Sensor::b4, Sensor::b5,
// Sensor::b6, Sensor::b7, Sensor::b8, Sensor::b9, Sensor::b10, Sensor::b11,
// Sensor::b12, Sensor::b13, Sensor::b14, Sensor::b15 = Rear channels 0..15.
//
// Lift wheels first; keep the front sensors on a calibrated line.
// Default polarity: line LOW. Set forward.frontLineHigh/rearLineHigh/
// centerLineHigh before setForwardSettings() if needed; verify frontMap.
// Sensor exits require CLEAR for20ms, then a qualifying scan (confirmation0ms); an initial line
// cannot finish the command immediately. All calls need front calibration;
// cl/cr additionally need underbody calibration; B needs rear calibration.
// Every return stops both motors. Sensor mode has no default elapsed-time cap.
// Press Start after release to stop. No automatic restart; reset to repeat.
// Result enum order: DistanceReached, SensorDetected, InvalidArgument,
// CalibrationNotReady, DistanceNotCalibrated, LineLost, SensorError, Timeout,
// Stopped, MotorsNotReady. See ForwardLineTest for a serial-controlled test.



// F_Line still takes exactly four parameters: left, right, KP, exit condition.
// Migration: divide old F_Line KP/KD/recovery threshold by50 (5/.05/4 -> .1/.001/.08).
// KP/KD/threshold: any finite signed float, no tuning ceiling; migrate old gains /50.
// Correction = KP*error + KD*filtered((error-previousError)/dtSeconds).
// Error range is-50..+50. KD units: percent-command seconds per position unit.
// D uses20ms filtering; first sample and dt<100us or >50ms use P only and resetD.
// Each call starts fresh. Final wheels stay clamped0..100 before compensation.

// Optional extra line-noise confirmation: setFLineSensorDebounceMs(7) or20.
// These are milliseconds of stable line, not MUX microseconds; narrow pulses
// can be missed with longer confirmation. Default0 accepts one strong scan.
// Serial.println(robot.lastFLineSensorStrength()); // fresh selected strength
// This may differ from normalized() tracking EMA; thresholds: clear<=400,line>=600.

// ===== DISTANCE MODE: copy these blocks into the main MyMiniPro tab =====
// Last argument is CENTIMETERS, never milliseconds/PWM/sensor number.
// GLOBAL: edit the existing measuredDistanceScale if present; do not duplicate it.
// constexpr float measuredDistanceScale = 1.6875; // corrected:2.25*(actual15cm/target20cm)
// This robot's measured correction; repeat3 runs and verify the distance later.
// At40/40,20cm nominal duration=20/(1.6875*40)=0.2963s; keep a longer safety limit.
//
// SETUP: replace the existing distanceSettings block after begin(), before waitButton.
// MyMiniPro::ForwardSettings distanceSettings;
// distanceSettings.centimetersPerSecondPerPercent = measuredDistanceScale;
// distanceSettings.maximumRunMs = 1500; // short-test safety limit
// robot.setForwardSettings(distanceSettings);
// robot.setMuxSettleMicroseconds(400); // use front calibration made at400us
// robot.setFLineKd(0.001);
//
// SETUP (after waitButton): replace the current movement block INCLUDING its
// reverse/braking pulse with this one-shot short test, not an extra loop call:
// const auto distanceResult = robot.F_Line(40, 40,0.3, 2.0); // estimated2cm
// robot.stopMotors();
// Serial.println(MyMiniProForward::resultName(distanceResult));
// LOOP: keep this distance call out of loop() so it does not repeat forever.
// Coefficient0 deliberately returns DistanceNotCalibrated; do not guess a value.
//
// MEASUREMENT: use the already-working sensor-mode run on a straight track at
// the SAME nominal40/40 command, load/surface and compensation settings.
// Measure physical travelD in cm and elapsedt in seconds. Then coefficient=D/(t*40).
// Example arithmetic only:24cm /2s /40 =0.30 cm/s per command-percent.
// Time only F_Line: take millis() immediately before and after its call, before
// Serial printing or the reverse pulse. Measure travel to the F_Line stop point.
// Repeat and average; startup/slip/curves mean this is an estimate, not encoder distance.
// Pico2 begin enables compensation; F_Line services/applies it internally.
// Supply nominal speeds40/40; do NOT pre-compensate them or multiply coefficient by gain.
// Expected success: DistanceReached. Fault/Stopped/Timeout can finish earlier.
// Temporary line loss now recovers; LineLost remains an enum for compatibility.

// ===== GUIDED DISTANCE CALIBRATION (no guessed coefficient) =====
// Open Examples > Pico2 My Mini Pro > ForwardDistanceCalibration.
// Start enables Serial115200 commands only. m=bounded0.5s measurement at40/40;
// Repeat m/c for3 runs; average the3 printed coefficients for MyMiniPro.
// Each uses actual elapsed time. d uses only the latest run's coefficient.
// then enter c <actual centimeters>; d 2 validates a short2cm run; x stops.
// Copy the printed measuredDistanceScale constant into MyMiniPro.
// Pure helper if you already measured the run:
// const float actualCm = 0.0; // replace with actual travel
// const uint32_t elapsedMs = 0; // replace with the captured F_Line elapsed time
// float k = MyMiniProForward::distanceCoefficientFromMeasurement(actualCm, elapsedMs, 40.0);
// distanceSettings.centimetersPerSecondPerPercent = k; // NAN means invalid measurement
// Wheel diameter46mm => circumference144.513mm/rev, but no encoder/RPM is known;
// diameter alone cannot supply k. Measure actual travel/time with compensation enabled.



// ===== F_LINE PHASE TWO: choose the desired call after waitButton =====
// Recovery: KP > threshold snaps; KP <= threshold keeps mild last-error steering.
// robot.setFLineRecoveryKpThreshold(0.4); // KP >0.4 snaps on loss; KP <=0.4 keeps last-error steering
// robot.getFLineRecoveryKpThreshold();
// robot.setFLineApproachSpeed(30, 30); // After numeric distance: left/right 1..100.
// robot.setFLineApproachSpeed(30); // Same speed for both wheels.
// robot.setFLineApproachSpeed(0); // Restore the speeds passed to F_Line (default).
// Turns: exit,action,integer turnSpeed,exact F/B stop sensor,integer brake.
// robot.setFLineFRMotors(100, -30); // signed percentages of turnSpeed
// robot.setFLineFLMotors(-30, 100); // separate left-turn wheel profile
// Retired: setFLineTurnClearanceCm / setFLineMinTurnMs accept only zero.
// Nonzero returns false; no stored value or motion effect. Getters always return zero.
// robot.fLineTurnClearanceCm(); // always zero; retired API
// robot.fLineMinTurnMs(); // always zero; retired API
// robot.F_Line(40, 40,0.1, f5, fr, 90, f11, 40); // brake, f14/f15 each cross then both clear, turn to f11
// robot.F_Line(40, 40,0.1, f15, fl, 50, f5, 40); // brake, f0/f1 each cross then both clear, turn to f5
// robot.F_Line(40, 40,0.1, f0, fr, 50, b5, 40); // exact rear b5 works too
// robot.F_Line(40, 40,0.1, f15, cl, 50, f5, 40); // seek cl, then left spin
// robot.F_Line(40, 40,0.1, f15, cr, 50, b5, 40); // seek cr, then right spin
// robot.F_Line(40, 40,0.1, 20.0, nl, 50, f5, 40); // left spin now
// robot.F_Line(40, 40,0.1, 20.0, nr, 50, b5, 40); // right spin now
// robot.F_Line(40, 40,0.1, cl, cl, 50, f5, 40); // already centered: no approach
// robot.F_Line(40, 40,0.1, cr, cr, 50, b5, 40);
// robot.F_Line(40, 40,0.1, f15, stop, 40);
// robot.F_Line(40, 40,0.1, 20.0, stop, 0); // stop, no active brake
// robot.F_Line(40, 40,0.1, f15, cross, 0); // RETURNS WITH MOTORS RUNNING
// robot.F_Line(40, 40,0.1, b15, cross, 40); // same sensor black->white, brake+stop
// robot.F_Line(40, 40,0.1, cl, cross, 40);
// robot.F_Line(40, 40,0.1, 20.0, cross, f5, 0); // explicit crossing sensor
// robot.F_Line(40, 40,0.1, 20.0, cross, b5, 40);
// Valid pairing table:
// numeric/F exit | fl fr cl cr nl nr stop cross
// cl/cr exit     | cl cr stop cross
// B exit         | stop cross
// Turn stop and numeric cross sensors: F/B only. Numeric cross needs7 arguments.
// fr/fl advance: fl uses f0/f1, fr uses f14/f15. Each must see confirmed black,
// then BOTH must be white together for debounceMs; starting white alone never counts.
// Retouch/ambiguous readings restart shared white confirmation. Fresh raw samples.
// Start profile immediately after pair clears: no extra distance/minimum turn time.
// Selected TURN-stop stays unchanged: NEW stable white then black after turn starts.
// Old black at turn entry cannot finish; other turns retain their existing gate.
// fr/fl pre-brake uses the same brake level, based on incoming following outputs.
// Advance uses the original left/right speeds, without PD; final brake opposes turn outputs.
// Setter defaults: fr(100,0), fl(0,100); each ratio clamps -100..100.
// Wheel command = turnSpeed * ratio /100 (integer truncation), then voltage compensation.
// Example fr(100,-30) at90 -> Motor(90,-27) before compensation. Values are tuning examples.
// F exit: selected OR its inward neighbor, each with independent white->black history.
// f0|f1, f4|f5, f11|f10, f15|f14; only the initial follow gate uses OR.
// Sensor cross uses selected black->white; neighbor-only exit first waits for selected black.
// TURN-stop stays single-sensor and fr/fl end-pair preparation stays AND.
// CROSS0 retains output; issue the next command or robot.stopMotors() yourself.
// Other completions and every fault stop. Action owns braking; no extra manual pulse.
// turnSpeed/brake clamp0..100. Brake counteracts each incoming wheel:
// magnitude=ceil(abs(command)*level/100), duration=5+ceil(25*level/100)ms (max30ms).
// Results: TurnCompleted, StopCompleted, CrossCompleted, InvalidAction,
// InvalidTurnSensor, InvalidCrossSensor, or existing fault/stop results.
// No default sensor-stage timeout. User stop still works during recovery/turn/brake.

// ===== F_Line speed profiles (ForwardSettings) =====
// settings.startupRampMs = 300; // from first translation output;0 disables
// settings.distanceDecelerationCm = 5; // numeric Follow only;0 disables
// settings.distanceEndSpeedPercent = 20; // percent OF target, not absolute PWM
// robot.setForwardSettings(settings); // retain calibration/timeout fields too
// Default target40 -> about8 at gain1/centered line, held over final1cm of5cm window.
// New call resets ramp. Stop/fault still stops immediately; turns/brakes bypass it.
// Distance uses actual ramped output. Tune low-speed ratio for the real motor's deadband.
// Per wheel: requested speed<25 bypasses BOTH profiles;25 qualifies.
// Example24/40 -> left immediate24, right ramps and later holds about8.
// Negative SPL/SPR arguments remain invalid; PD-generated signed outputs keep their sign.
