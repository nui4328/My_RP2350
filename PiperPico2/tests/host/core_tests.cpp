#include "PiperPico2.h"
#include <cassert>
#include <iostream>
static void coldStartChecks(){
 using PWM=Adafruit_PWMServoDriver;
 clockMs=0;
 {
  PiperPico2 robot;
  PWM::readyAt=600;Wire.deviceReadyAt[0x3C]=900;
  const int starts=PWM::beginCalls;
  duty[8]=123;
  assert(robot.begin());
  assert(clockMs>=900 && PWM::beginCalls>starts+1);
  assert(Wire.timeoutMs==25 && Wire.resetOnTimeout);
  assert(duty[8]==0 && duty[9]==0 && duty[7]==0 && duty[6]==0);
 }
 PWM::readyAt=0;Wire.deviceReadyAt[0x3C]=0;
 {
  PiperPico2 robot;
  pinLowUntil[4]=clockMs+700;
  assert(robot.begin());assert(clockMs>=pinLowUntil[4]);
 }
 pinLowUntil[4]=0;
 {
  PiperPico2 robot;
  const uint32_t started=clockMs;
  pinLowUntil[5]=clockMs+10000;
  assert(!robot.begin());
  assert(clockMs-started>=2000 && clockMs-started<3000);
  assert(!robot.pwmReady() && !robot.displayReady());
  assert(duty[8]==0 && duty[6]==0);
 }
 pinLowUntil[5]=0;
 {
  PiperPico2 robot;
  const uint32_t started=clockMs;
  Wire.deviceReadyAt[0x3C]=clockMs+10000;
  assert(!robot.begin());
  assert(clockMs-started>=2000 && clockMs-started<3000);
  assert(robot.pwmReady() && !robot.displayReady());
  assert(duty[8]==0 && duty[6]==0);
 }
 Wire.deviceReadyAt[0x3C]=0;
}

static void encoderCycle(uint8_t originalA,uint8_t originalB){
 gpio[originalA]=LOW; gpio[originalB]=LOW;
 gpio[originalB]=HIGH; if(interrupts[originalB]) interrupts[originalB]();
 gpio[originalA]=HIGH; if(interrupts[originalA]) interrupts[originalA]();
 gpio[originalB]=LOW; gpio[originalA]=LOW;
}
static void encoderPinChecks(){
 const uint8_t pairs[4][2]={{20,19},{21,22},{16,17},{10,18}};
 {
  PiperPico2 normal; assert(normal.begin());
  for(uint8_t i=0;i<4;++i) encoderCycle(pairs[i][0],pairs[i][1]);
  assert(normal.enc1Left()==1 && normal.enc1Right()==1);
  assert(normal.enc2Left()==1 && normal.enc2Right()==1);
 }
 {
  PiperPico2 swapped;
  for(uint8_t i=0;i<4;++i) assert(swapped.setEncoderPins(i,pairs[i][1],pairs[i][0]));
  assert(!swapped.setEncoderPins(4,20,19));
  assert(!swapped.setEncoderPins(255,20,19));
  assert(!swapped.setEncoderPins(0,20,20));
  assert(!swapped.setEncoderPins(0,21,22));
  assert(!swapped.setEncoderPins(0,8,9));
  assert(swapped.begin());
  for(uint8_t i=0;i<4;++i){
   assert(interrupts[pairs[i][0]]==nullptr);
   assert(interrupts[pairs[i][1]]!=nullptr);
   assert(!swapped.setEncoderPins(i,pairs[i][0],pairs[i][1]));
   encoderCycle(pairs[i][0],pairs[i][1]);
  }
  assert(swapped.enc1Left()==-1 && swapped.enc1Right()==-1);
  assert(swapped.enc2Left()==-1 && swapped.enc2Right()==-1);
  swapped.resetEncoders(); assert(swapped.begin());
  encoderCycle(20,19); assert(swapped.enc1Left()==-1);
 }
 for(const auto& pair:pairs){
  assert(interrupts[pair[0]]==nullptr && interrupts[pair[1]]==nullptr);
 }
}

static void servoRangeChecks(){
 using PWM=Adafruit_PWMServoDriver;
 PiperPico2 robot;
 int writes=PWM::writes;
 assert(robot.setServoPulseRangeUs(0,1000,2000));
 assert(robot.setServoPulseRange(1,120,600));
 assert(PWM::writes==writes); // Configuration never moves a servo.
 robot.setservo(0,90); assert(PWM::writes==writes); // No output before initialization.
 assert(!robot.setServoPulseRange(6,100,500));
 assert(!robot.setServoPulseRange(255,100,500));
 assert(!robot.setServoPulseRange(0,0,500));
 assert(!robot.setServoPulseRange(0,500,500));
 assert(!robot.setServoPulseRange(0,600,500));
 assert(!robot.setServoPulseRange(0,100,4096));
 assert(!robot.setServoPulseRangeUs(0,0,2000));
 assert(!robot.setServoPulseRangeUs(0,2000,1000));
 assert(!robot.setServoPulseRangeUs(0,1000,1001)); // Cannot represent the narrow window.
 assert(!robot.setServoPulseRangeUs(0,1000,65535)); // Reject long pulses without overflow.
 assert(!robot.setServoPulseRangeUs(6,1000,2000));
 assert(robot.begin());
 robot.setservo(0,-50); assert(PWM::channelOff[15]==246);
 robot.setservo(0,90); assert(PWM::channelOff[15]==368);
 robot.setservo(0,200); assert(PWM::channelOff[15]==491);
 robot.setservo(1,0); assert(PWM::channelOff[14]==120);
 robot.setservo(1,180); assert(PWM::channelOff[14]==600);
 const int channels[4]={13,12,11,10};
 for(uint8_t i=2;i<6;++i){
  robot.setservo(i,0); assert(PWM::channelOff[channels[i-2]]==90);
  robot.setservo(i,180); assert(PWM::channelOff[channels[i-2]]==760);
 }
 robot.SERVO_MIN=100; robot.SERVO_MAX=700;
 robot.setservo(2,0); assert(PWM::channelOff[13]==100); // Untouched channels follow legacy defaults.
 robot.setservo(0,0); assert(PWM::channelOff[15]==246); // Overrides remain independent.
 assert(robot.setServoPulseRange(2,150,550)); // Runtime reconfiguration only affects the next command.
 assert(PWM::channelOff[13]==100);
 robot.setservo(2,90); assert(PWM::channelOff[13]==350);
 writes=PWM::writes;
 robot.setservo(6,90); robot.setservo(255,90); assert(PWM::writes==writes);
 robot.SERVO_MIN=800; robot.SERVO_MAX=700;
 robot.setservo(3,90); assert(PWM::writes==writes); // Invalid legacy defaults cannot reach hardware.
 assert(robot.begin()); robot.setservo(0,180); assert(PWM::channelOff[15]==491);
 PWM::works=false; assert(!robot.begin()); writes=PWM::writes;
 robot.setservo(0,90); assert(PWM::writes==writes);
 PWM::works=true;
}

static void drv8874Checks(){
 using Driver=PiperPico2::MotorDriver;
 using PWM=Adafruit_PWMServoDriver;
 PiperPico2 robot(Driver::DRV8874);
 assert(robot.setMotorDriver('B',Driver::LEGACY));
 assert(robot.setMotorDriver('C',Driver::DRV8874,true));
 assert(!robot.setMotorDriver('Z',Driver::DRV8874));
 assert(!robot.setMotorDriver('A',static_cast<Driver>(255)));
 robot.motor('A',50); assert(duty[8]==0 && !robot.motorOK());
 assert(robot.begin());
 assert(!robot.setMotorDriver('A',Driver::LEGACY));
 assert(duty[8]==0 && duty[7]==0);
 assert(PWM::channelOn[0]==0 && PWM::channelOn[1]==4096);
 assert(PWM::channelOn[2]==0 && PWM::channelOn[3]==4096);
 assert(PWM::channelOn[6]==0 && PWM::channelOn[7]==4096);
 assert(PWM::channelOn[4]==0 && PWM::channelOn[5]==0); // Legacy B remains neutral.
 int writes=PWM::writes;
 uint32_t start=clockMs;
 robot.motor('A',50);
 assert(duty[8]>0 && PWM::channelOn[0]==4096 && PWM::channelOn[1]==4096);
 assert(PWM::writes==writes+1 && clockMs==start); // No new wake-up when already awake.
 robot.motor('A',70); assert(PWM::writes==writes+1);
 robot.motor('A',-50); assert(PWM::channelOn[0]==0 && PWM::channelOn[1]==4096);
 robot.motor('A',0); assert(duty[8]==0 && PWM::channelOn[1]==4096); // Brake, not sleep.
 robot.motor('C',50); assert(duty[7]>0 && PWM::channelOn[2]==0); // Inverted polarity.
 robot.motor('C',-50); assert(PWM::channelOn[2]==4096);
 robot.motor('B',50); assert(duty[9]>0 && PWM::channelOn[5]==4096 && PWM::channelOn[4]==0);
 robot.motor('D',1000); assert(duty[6]==65535 && PWM::channelOn[6]==4096);
 robot.motor('D',-1000); assert(duty[6]==65535 && PWM::channelOn[6]==0);
 PWM::failChannel=0; robot.motor('A',50);
 assert(!robot.motorOK() && duty[8]==0);
 PWM::failChannel=-1;
 start=clockMs; writes=PWM::writes;
 robot.motor('A',50);
 assert(robot.motorOK() && duty[8]>0 && PWM::writes==writes+2);
 assert(dutyUpdatedAt[8]-start>=1); // Failed state is resynchronized before EN goes HIGH.
 robot.stopMotors();
 robot.invalidateMotorCache();
 PWM::failChannel=1; robot.motor('A',50);
 assert(!robot.motorOK() && duty[8]==0); // nSLEEP I2C failure must not enable EN.
 PWM::failChannel=-1; start=clockMs;
 robot.motor('A',50); assert(robot.motorOK() && dutyUpdatedAt[8]-start>=1);
 robot.stopMotors(); assert(robot.motorOK());
 assert(duty[8]==0 && duty[9]==0 && duty[7]==0 && duty[6]==0);
 assert(PWM::channelOn[1]==4096 && PWM::channelOn[3]==4096 && PWM::channelOn[7]==4096);
 // Re-begin after PCA9685 reinitialization must restore the wake state too.
 PWM::channelOn[1]=0; assert(robot.begin()); assert(PWM::channelOn[1]==4096);
}

static void ownershipChecks(){
 {
  PiperPico2 owner;
  assert(owner.begin());
  {
   PiperPico2 other;
   assert(other.begin(false)); // External ownership opt-out.
   assert(!other.begin()); // Cannot steal existing interrupt ownership.
  }
  gpio[19]=HIGH; interrupts[20](); assert(owner.enc1Left()==1);
 }
 assert(interrupts[20]==nullptr && interrupts[21]==nullptr);
}
int main(){
 coldStartChecks();
 encoderPinChecks();
 servoRangeChecks();
 drv8874Checks();
 ownershipChecks();
 PiperPico2 robot;
 assert(robot.begin());
 assert(robot.pwmReady() && robot.displayReady());
 assert(!robot.calibrationReady()); // Erased EEPROM is rejected.
 assert(robot.adcMin(0)==0 && robot.adcMax(0)==4095);
 const int baseline=Adafruit_PWMServoDriver::writes;
 robot.motor('A',30); assert(Adafruit_PWMServoDriver::writes==baseline+2);
 robot.motor('A',40); assert(Adafruit_PWMServoDriver::writes==baseline+2);
 robot.motor('A',-30); assert(Adafruit_PWMServoDriver::writes==baseline+4);
 Adafruit_PWMServoDriver::failWrites=1;
 robot.motor('A',30); assert(!robot.motorOK() && duty[8]==0);
 robot.motor('A',30); assert(robot.motorOK() && duty[8]>0);
 robot.stopMotors(); assert(duty[8]==0 && duty[9]==0 && duty[7]==0 && duty[6]==0);
 auto writes=Adafruit_PWMServoDriver::writes;
 robot.motor('Z',90); assert(Adafruit_PWMServoDriver::writes==writes);
 robot.getPWM(); robot.motor('A',0); assert(Adafruit_PWMServoDriver::writes==writes+2);
 gpio[19]=LOW; interrupts[20](); assert(robot.enc1Left()==-1);
 gpio[19]=HIGH; interrupts[20](); assert(robot.enc1Left()==0);
 gpio[22]=HIGH; interrupts[21](); assert(robot.enc1Right()==1);
 robot.resetEncoders(); assert(robot.enc1Right()==0);
 assert(!robot.setI2CClock(1000000)); assert(robot.setI2CClock(400000));
 assert(robot.startCalibration(100)); adcValue=100;
 assert(robot.updateCalibration()); clockMs+=100; adcValue=3000;
 assert(!robot.updateCalibration()); assert(robot.calibrationSaved());
 assert(robot.adcMin(0)==100 && robot.adcMax(0)==3000);
 assert(Wire.clock==400000);
 // Failed and short reads must never partially replace live calibration.
 Wire.shortRead=true; assert(!robot.loadCalibration()); Wire.shortRead=false;
 assert(robot.adcMin(0)==100);
 Wire.memory[0]=200; Wire.failAt=Wire.requests+2;
 assert(!robot.loadCalibration()); assert(robot.adcMin(0)==100);
 Wire.failAt=-1; assert(robot.loadCalibration()); assert(robot.adcMin(0)==200);
 Wire.fail=true; assert(!robot.saveCalibration()); Wire.fail=false;
 assert(robot.startCalibration(100)); adcValue=500; robot.updateCalibration();
 clockMs+=100; robot.updateCalibration(); assert(!robot.calibrationSaved());
 assert(robot.adcMin(0)==200); // Flat calibration preserves live values.
 assert(robot.startCalibration()); robot.cancelCalibration(); assert(!robot.isCalibrating());
 assert(!robot.startCalibration(0)); robot.readLines(nullptr);
 assert(robot.adcRead(16)==0); assert(robot.readLine(10)==0);
 robot.updateBattery(); assert(robot.getBatteryVoltage()>0); robot.bat_control();
 // Uint32 clock rollover keeps the asynchronous state machine working.
 clockMs=0xfffffff0u; assert(robot.startCalibration(100)); adcValue=100;
 robot.updateCalibration(); clockMs+=100; adcValue=3000; robot.updateCalibration();
 assert(robot.calibrationSaved());
 Adafruit_PWMServoDriver::works=false;
 assert(!robot.begin()); robot.motor('A',100); assert(duty[8]==0);
 Adafruit_PWMServoDriver::works=true; Adafruit_SSD1306::works=false;
 assert(!robot.begin()); assert(!robot.displayReady()); robot.showVoltageUntilButton();
 std::cout<<"All core behavior checks passed\n";
}
