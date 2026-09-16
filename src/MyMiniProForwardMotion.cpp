#include "Pico2_MyMiniPro.h"
#include "MyMiniProForwardTracker.h"
#include "MyMiniProForwardPD.h"
#include "MyMiniProForwardRecovery.h"
#include "MyMiniProForwardRamp.h"

using Action = MyMiniProForward::Action;
namespace {
uint16_t normalizeReading(uint16_t reading,uint16_t minimum,uint16_t maximum) {
  if(maximum<=minimum)return MyMiniPro::kUncalibratedNormalizedValue;
  const long value=map(reading,minimum,maximum,0,1000);
  return static_cast<uint16_t>(constrain(value,0L,1000L));
}
}

MyMiniPro::ForwardResult MyMiniPro::F_Line(int l,int r,float kp,float cm,LineAction a,int brake) {
  const ForwardMotion motion{a.value,0,brake,LineSensor::F0,ForwardMotion::Form::Finish};
  return runForward(l,r,kp,false,cm,LineSensor::F0,&motion);
}
MyMiniPro::ForwardResult MyMiniPro::F_Line(int l,int r,float kp,LineSensor exit,LineAction a,int brake) {
  const ForwardMotion motion{a.value,0,brake,exit,ForwardMotion::Form::Finish};
  return runForward(l,r,kp,true,0,exit,&motion);
}
MyMiniPro::ForwardResult MyMiniPro::F_Line(int l,int r,float kp,float cm,LineAction a,
                                         int speed,LineSensor selected,int brake) {
  const ForwardMotion motion{a.value,speed,brake,selected,ForwardMotion::Form::Turn};
  return runForward(l,r,kp,false,cm,LineSensor::F0,&motion);
}
MyMiniPro::ForwardResult MyMiniPro::F_Line(int l,int r,float kp,LineSensor exit,LineAction a,
                                         int speed,LineSensor selected,int brake) {
  const ForwardMotion motion{a.value,speed,brake,selected,ForwardMotion::Form::Turn};
  return runForward(l,r,kp,true,0,exit,&motion);
}
MyMiniPro::ForwardResult MyMiniPro::F_Line(int l,int r,float kp,float cm,LineAction a,
                                         LineSensor selected,int brake) {
  const ForwardMotion motion{a.value,0,brake,selected,ForwardMotion::Form::Cross};
  return runForward(l,r,kp,false,cm,LineSensor::F0,&motion);
}

bool MyMiniPro::forwardSensorReady(LineSensor sensor) const {
  const unsigned id=static_cast<unsigned>(sensor);
  if(id>33)return false;
  if(id==16 || id==17)return hasUnderbodyCalibration();
  const Array array=id<16 ? Array::Front : Array::Rear;
  const unsigned ch=id<16 ? id : id-18;
  return hasCalibration(array) &&
         int(maxValues(array)[ch])-int(minValues(array)[ch])>=MyMiniProLinePID::kMinSpan;
}
uint16_t MyMiniPro::forwardSensorStrength(LineSensor sensor,const ForwardSettings &cfg,uint16_t &lastStrength) {
  const unsigned id=static_cast<unsigned>(sensor);
  uint16_t value=kUncalibratedNormalizedValue, raw=0;
  bool high=cfg.frontLineHigh;
  if(id<16) {
    scanMux(frontMux_,frontCurrent_,frontSensorFilterInitialized_,id,&raw);
    value=normalizeReading(raw,minSensorFront(id),maxSensorFront(id));
  } else if(id<18) {
    if(motorVoltageConversionPending_){delay(2);serviceMotorVoltageCompensation();}
    value=id==16 ? readAdcLNormalized() : readAdcRNormalized();
    high=cfg.centerLineHigh;
  } else if(id<34) {
    scanMux(rearMux_,rearCurrent_,rearSensorFilterInitialized_,id-18,&raw);
    value=normalizeReading(raw,minSensorRear(id-18),maxSensorRear(id-18));
    high=cfg.rearLineHigh;
  }
  if(value!=kUncalibratedNormalizedValue && !high)value=1000-value;
  lastStrength=value;
  return value;
}

MyMiniPro::ForwardResult MyMiniPro::runForward(int left,int right,float kp,bool sensorExit,
                                               float cm,LineSensor sensor,const ForwardMotion *motion,bool reverse) {
  struct Stop { MyMiniPro &robot; bool retain=false; ~Stop(){if(!retain)robot.stopMotors();} } guard{*this};
  stopMotors();
  const ForwardSettings cfg=reverse ? backwardSettings_ : forwardSettings_;
  const uint16_t confirmation=reverse ? bLineSensorDebounceMs_ : fLineSensorDebounceMs_;
  const float kd=reverse ? bLineKd_ : fLineKd_;
  const float recoveryThreshold=reverse ? bLineRecoveryKpThreshold_ : fLineRecoveryKpThreshold_;
  const bool customApproach=!reverse && !sensorExit && fLineApproachLeft_>0;
  const int approachLeft=customApproach ? fLineApproachLeft_ : left;
  const int approachRight=customApproach ? fLineApproachRight_ : right;
  uint16_t &lastStrength=reverse ? lastBLineSensorStrength_ : lastFLineSensorStrength_;
  lastStrength=kUncalibratedNormalizedValue;
  const int direction=reverse ? -1 : 1;
  const float errorScale=reverse ? 1.0f : MyMiniProForward::kPositionErrorScale;
  const Array trackingArray=reverse ? Array::Rear : Array::Front;
  const uint8_t *trackingMap=reverse ? cfg.rearMap : cfg.frontMap;
  const bool trackingHigh=reverse ? cfg.rearLineHigh : cfg.frontLineHigh;
  const MuxPins &trackingMux=reverse ? rearMux_ : frontMux_;
  uint16_t *trackingCurrent=reverse ? rearCurrent_ : frontCurrent_;
  bool &trackingInitialized=reverse ? rearSensorFilterInitialized_ : frontSensorFilterInitialized_;
  const uint16_t *trackingMin=minValues(trackingArray),*trackingMax=maxValues(trackingArray);
  const unsigned id=static_cast<unsigned>(sensor);
  const uint32_t runLimit=sensorExit ? cfg.sensorMaximumRunMs : cfg.maximumRunMs;
  if(left<=0 || right<=0 || left>100 || right>100 || !isfinite(kp) ||
     (reverse && (kp<0 || kp>100)) ||
     (!sensorExit && !runLimit) || runLimit>60000 || !cfg.debounceMs ||
     !MyMiniProLinePID::validMap(trackingMap) ||
     (sensorExit ? id>33 : (!isfinite(cm) || cm<0)))return ForwardResult::InvalidArgument;
  if(!reverse && !sensorExit &&
     (!isfinite(cfg.distanceDecelerationCm) || cfg.distanceDecelerationCm<0 ||
      (cfg.distanceDecelerationCm>0 &&
       (!cfg.distanceEndSpeedPercent || cfg.distanceEndSpeedPercent>100))))return ForwardResult::InvalidArgument;

  const Action action=motion ? motion->action : Action::Invalid;
  const bool turn=action==Action::FL || action==Action::FR || action==Action::CL ||
                  action==Action::CR || action==Action::NL || action==Action::NR;
  const bool center=action==Action::CL || action==Action::CR;
  const bool leftTurn=action==Action::FL || action==Action::CL || action==Action::NL;
  const bool forwardArc=!reverse && (action==Action::FL || action==Action::FR);
  const bool centerGroup=action==Action::CS || action==Action::CP;
  const bool frontGroup=action==Action::FS || action==Action::FP || (forwardArc && !sensorExit);
  const bool groupPass=action==Action::CP || action==Action::FP;
  const bool newFinish=action==Action::NS || centerGroup || action==Action::FS || action==Action::FP;
  const LineSensor groupSensors[4]={centerGroup ? LineSensor::CL : LineSensor::F0,
      centerGroup ? LineSensor::CR : LineSensor::F1,LineSensor::F14,LineSensor::F15};
  const unsigned groupCount=centerGroup ? 2 : 4;
  const LineSensor arcSensors[2]={leftTurn ? LineSensor::F0 : LineSensor::F14,
                                 leftTurn ? LineSensor::F1 : LineSensor::F15};
  const int arcLeftPercent=leftTurn ? fLineFLLeftPercent_ : fLineFRLeftPercent_;
  const int arcRightPercent=leftTurn ? fLineFLRightPercent_ : fLineFRRightPercent_;
  const bool seekCenter=center && !(sensorExit && (id==16 || id==17));
  const LineSensor centerSensor=leftTurn ? LineSensor::CL : LineSensor::CR;
  if(motion) {
    if(newFinish && (reverse || sensorExit || motion->form!=ForwardMotion::Form::Finish))
      return ForwardResult::InvalidAction;
    if(static_cast<unsigned>(action)>=static_cast<unsigned>(Action::Invalid) ||
       (turn!=(motion->form==ForwardMotion::Form::Turn)) ||
       (motion->form==ForwardMotion::Form::Cross && (sensorExit || action!=Action::CROSS)) ||
       (sensorExit && ((id>=18 && turn) || ((id==16 || id==17) && turn && !center))))
      return ForwardResult::InvalidAction;
    const unsigned selected=static_cast<unsigned>(motion->selected);
    const bool arraySensor=selected<16 || (selected>=18 && selected<34);
    if(turn && !arraySensor)return ForwardResult::InvalidTurnSensor;
    if(action==Action::CROSS && !sensorExit &&
       (motion->form!=ForwardMotion::Form::Cross || !arraySensor))return ForwardResult::InvalidCrossSensor;
    if(cfg.sensorMaximumRunMs>60000)return ForwardResult::InvalidArgument;
  }
  if(leftMotorPins_.pwm==255 || rightMotorPins_.pwm==255)return ForwardResult::MotorsNotReady;
  if(calibrationActive() || underbodyCalibrationActive() || !hasCalibration(trackingArray))
    return ForwardResult::CalibrationNotReady;
  MyMiniProLinePID::Calibration cal;
  cal.haveWhite=cal.haveBlack=true;
  for(uint8_t ch=0;ch<16;++ch) {
    cal.white[ch]=trackingHigh ? trackingMin[ch] : trackingMax[ch];
    cal.black[ch]=trackingHigh ? trackingMax[ch] : trackingMin[ch];
  }
  if(!cal.valid() || (sensorExit && !forwardSensorReady(sensor)))return ForwardResult::CalibrationNotReady;
  if(centerGroup || frontGroup)for(unsigned i=0;i<groupCount;++i)
    if(!forwardSensorReady(groupSensors[i]))return ForwardResult::CalibrationNotReady;
  if(motion && ((turn && !forwardSensorReady(motion->selected)) ||
     (seekCenter && !forwardSensorReady(centerSensor)) ||
     (!sensorExit && action==Action::CROSS && !forwardSensorReady(motion->selected))))
    return ForwardResult::CalibrationNotReady;
  if(!sensorExit &&
     (!isfinite(cfg.centimetersPerSecondPerPercent) || cfg.centimetersPerSecondPerPercent<=0))
    return ForwardResult::DistanceNotCalibrated;
  if(!motion && !sensorExit && cm==0)return ForwardResult::DistanceReached;

  enum class Stage { Follow, Center, Group, ArcBrake, ArcPair, Turn, CrossBlack, CrossWhite, Brake };
  Stage stage=Stage::Follow;
  MyMiniProForward::Edge edge;
  MyMiniProForward::Edge neighborEdge;
  MyMiniProForward::Tracker tracker;
  MyMiniProForward::PD controller;
  MyMiniProForward::Recovery recovery(errorScale);
  MyMiniProForward::StartupRamp startupRamp(reverse ? 0 : cfg.startupRampMs);
  bool wasLost=false, timing=false;
  bool arcSawBlack[2]={false,false},arcBlackTiming[2]={false,false};
  uint32_t arcBlackSince[2]={0,0};
  bool groupSawBlack[4]={false,false,false,false},groupTiming[4]={false,false,false,false};
  uint32_t groupSince[4]={0,0,0,0};
  uint32_t since=0, stageStarted=millis(), previous=stageStarted;
  float distance=0, previousSpeed=0;
  const int brake=motion ? MyMiniProForward::clampPercent(motion->brake) : 0;
  const int speed=motion ? MyMiniProForward::clampPercent(motion->turnSpeed) : 0;
  MyMiniProForward::BrakePulse pulse{0,0,0};
  ForwardResult completion=ForwardResult::StopCompleted;
  bool released=startButtonPin_==255 || digitalRead(startButtonPin_)!=LOW;
  const auto enter=[&](Stage next) { stage=next;stageStarted=millis();timing=false;edge=MyMiniProForward::Edge(); };
  const auto stable=[&](bool qualified,uint32_t now,uint16_t duration) {
    if(!qualified){timing=false;return false;}
    if(!timing){timing=true;since=now;}
    return uint32_t(now-since)>=duration;
  };
  const auto beginBrake=[&](ForwardResult reason,bool prepareArc=false) {
    completion=reason;
    pulse=MyMiniProForward::brakePulse(leftMotorCommand(),rightMotorCommand(),brake);
    enter(prepareArc ? Stage::ArcBrake : Stage::Brake);
    if(pulse.durationMs)Motor(pulse.left,pulse.right);
  };
  const auto driveTranslation=[&](int nominalLeft,int nominalRight) {
    int l=compensateBaseMotorSpeed(direction*nominalLeft);
    int r=compensateBaseMotorSpeed(direction*nominalRight);
    // Scale the complete, bounded PD/recovery output, including compensation.
    // Ramping only the base would still allow a large D term to kick at startup.
    if(!reverse) {
      const float startup=cfg.startupRampMs ? startupRamp.factor(millis()) : 1;
      const auto wheelFactor=[&](int target) {
        // Eligibility follows the requested translation speed, not signed PD output.
        // Each low-speed wheel responds immediately even if its partner ramps.
        if(target<25)return 1.0f;
        if(stage!=Stage::Follow || sensorExit)return startup;
        return fminf(startup,MyMiniProForward::distanceRampFactor(cm-distance,
                     cfg.distanceDecelerationCm,cfg.distanceEndSpeedPercent,target,target));
      };
      l=int(l*wheelFactor(left));r=int(r*wheelFactor(right));
    }
    Motor(l,r);
    previousSpeed=cfg.centimetersPerSecondPerPercent*direction*(l+r)/(2.0f*motorVoltageCompensationGain());
  };
  const auto followLine=[&](bool valid,float position,uint32_t sampledUs) {
    const float error=recovery.error(valid,position*errorScale,kp,left,right,recoveryThreshold);
    if(!valid || wasLost)controller=MyMiniProForward::PD();
    const float correction=controller.step(error,kp,valid ? kd : 0,sampledUs,!reverse);
    wasLost=!valid;
    const float minimum=reverse ? 0.0f : -100.0f;
    const int l=int(fmaxf(minimum,fminf(100,left+correction)));
    const int r=int(fmaxf(minimum,fminf(100,right-correction)));
    driveTranslation(l,r);
  };
  const auto followApproach=[&]() {
    scan(Direction::Forward);
    const uint32_t sampledUs=micros();
    uint16_t raw[16];
    for(uint8_t ch=0;ch<16;++ch)raw[ch]=values(trackingArray,ch).current;
    const auto line=tracker.locate(raw,cal,trackingMap);
    followLine(line.valid,line.error,sampledUs);
  };
  while(true) {
    updateBuzzer();
    if(cfg.stopRequested && cfg.stopRequested())return ForwardResult::Stopped;
    if(startButtonPin_!=255) {
      const bool pressed=digitalRead(startButtonPin_)==LOW;
      if(!pressed)released=true;
      if(pressed && released)return ForwardResult::Stopped;
    }
    if(calibrationActive() || underbodyCalibrationActive())return ForwardResult::CalibrationNotReady;
    if(stage==Stage::Brake || stage==Stage::ArcBrake) {
      if(uint32_t(millis()-stageStarted)>=pulse.durationMs) {
        if(stage==Stage::ArcBrake) {
          stopMotors();
          enter(Stage::ArcPair);
          continue;
        }
        if(completion==ForwardResult::CrossCompleted && !brake)guard.retain=true;
        return completion;
      }
      delay(1);
      continue;
    }
    serviceMotorVoltageCompensation();
    if(stage==Stage::Group) {
      followApproach();
      bool anyBlack=false,allSeen=true,allWhite=true,anySeen=false;
      for(unsigned i=0;i<groupCount;++i) {
        if(!forwardSensorReady(groupSensors[i]))return ForwardResult::CalibrationNotReady;
        const uint16_t value=forwardSensorStrength(groupSensors[i],cfg,lastStrength);
        if(value==kUncalibratedNormalizedValue)return ForwardResult::SensorError;
        const uint32_t sampledAt=millis();
        if(value>=600) {
          if(!groupTiming[i]){groupTiming[i]=true;groupSince[i]=sampledAt;}
          if(uint32_t(sampledAt-groupSince[i])>=confirmation){groupSawBlack[i]=true;anyBlack=true;}
        } else groupTiming[i]=false;
        allSeen=allSeen && groupSawBlack[i];
        anySeen=anySeen || groupSawBlack[i];
        allWhite=allWhite && value<=400;
      }
      const uint32_t now=millis();
      if(cfg.sensorMaximumRunMs && uint32_t(now-stageStarted)>=cfg.sensorMaximumRunMs)return ForwardResult::Timeout;
      if(groupPass) {
        // CP requires both centers to have seen black; FP accepts any front end.
        // Clear all watched channels together before completing either crossing.
        if(stable((centerGroup ? allSeen : anySeen) && allWhite,now,cfg.debounceMs))
          beginBrake(ForwardResult::CrossCompleted);
      } else if(anyBlack) {
        if(forwardArc)beginBrake(ForwardResult::TurnCompleted,true);
        else beginBrake(ForwardResult::StopCompleted);
      }
      delay(1);
      continue;
    }
    if(stage==Stage::Turn || stage==Stage::ArcPair ||
       stage==Stage::CrossBlack || stage==Stage::CrossWhite) {
      const bool turning=stage==Stage::Turn;
      int l=left,r=right;
      if(turning) {
        const bool arc=action==Action::FL || action==Action::FR;
        l=leftTurn ? (arc ? 0 : -speed) : speed;
        r=leftTurn ? speed : (arc ? 0 : -speed);
        if(forwardArc) {
          l=speed*arcLeftPercent/100;
          r=speed*arcRightPercent/100;
        }
      }
      // Turns keep the robot-front orientation in BOTH travel modes.
      // Only following/approach/CROSS use reverse wheel signs.
      const int phaseDirection=turning ? 1 : direction;
      if(!turning && !reverse && !sensorExit)followApproach();
      else if(!turning)driveTranslation(l,r);
      else Motor(compensateBaseMotorSpeed(phaseDirection*l),compensateBaseMotorSpeed(phaseDirection*r));
      if(stage==Stage::ArcPair) {
        if(!forwardSensorReady(motion->selected))return ForwardResult::CalibrationNotReady;
        bool bothWhite=true;
        for(unsigned i=0;i<2;++i) {
          if(!forwardSensorReady(arcSensors[i]))return ForwardResult::CalibrationNotReady;
          const uint16_t value=forwardSensorStrength(arcSensors[i],cfg,lastStrength);
          if(value==kUncalibratedNormalizedValue)return ForwardResult::SensorError;
          const uint32_t sampledAt=millis();
          if(!arcSawBlack[i]) {
            if(value>=600) {
              if(!arcBlackTiming[i]){arcBlackTiming[i]=true;arcBlackSince[i]=sampledAt;}
              if(uint32_t(sampledAt-arcBlackSince[i])>=confirmation)arcSawBlack[i]=true;
            } else arcBlackTiming[i]=false;
          }
          bothWhite=bothWhite && arcSawBlack[i] && value<=400;
        }
        const uint32_t now=millis();
        if(cfg.sensorMaximumRunMs && uint32_t(now-stageStarted)>=cfg.sensorMaximumRunMs)return ForwardResult::Timeout;
        // Each channel owns its black history; both must remain white together.
        if(stable(bothWhite,now,cfg.debounceMs))enter(Stage::Turn);
        else delay(1);
        continue;
      }
      const LineSensor selected=turning || !sensorExit ? motion->selected : sensor;
      if(!forwardSensorReady(selected))return ForwardResult::CalibrationNotReady;
      const uint16_t strength=forwardSensorStrength(selected,cfg,lastStrength);
      if(strength==kUncalibratedNormalizedValue)return ForwardResult::SensorError;
      const uint32_t now=millis();
      if(cfg.sensorMaximumRunMs && uint32_t(now-stageStarted)>=cfg.sensorMaximumRunMs)return ForwardResult::Timeout;
      // The selected stop sensor must clear after turn entry before a new hit.
      if(turning &&
         edge.update(strength,now,cfg.debounceMs,confirmation))beginBrake(ForwardResult::TurnCompleted);
      else if(stage==Stage::CrossBlack && stable(strength>=600,now,confirmation))enter(Stage::CrossWhite);
      else if(stage==Stage::CrossWhite && stable(strength<=400,now,cfg.debounceMs))beginBrake(ForwardResult::CrossCompleted);
      else delay(1);
      continue;
    }

    const bool centerWait=stage==Stage::Center;
    const bool detectSensor=centerWait || sensorExit;
    const LineSensor selected=centerWait ? centerSensor : sensor;
    const unsigned selectedId=static_cast<unsigned>(selected);
    const bool onTrackingArray=reverse ? selectedId>=18 && selectedId<34 : selectedId<16;
    const unsigned selectedChannel=reverse ? selectedId-18 : selectedId;
    const bool frontExitPair=!reverse && !centerWait && sensorExit && selectedId<16;
    const unsigned neighborChannel=selectedId<8 ? selectedId+1 : selectedId-1;
    uint16_t exitRaw=0,neighborRaw=0;
    if(frontExitPair && (!forwardSensorReady(selected) ||
       !forwardSensorReady(static_cast<LineSensor>(neighborChannel))))return ForwardResult::CalibrationNotReady;
    if(detectSensor && onTrackingArray)scanMux(trackingMux,trackingCurrent,trackingInitialized,selectedChannel,&exitRaw,
                                             frontExitPair ? neighborChannel : 255,frontExitPair ? &neighborRaw : nullptr);
    else scan(reverse ? Direction::Reverse : Direction::Forward);
    const uint32_t sampleUs=micros();
    uint16_t raw[16];
    for(uint8_t ch=0;ch<16;++ch)raw[ch]=values(trackingArray,ch).current;
    const auto line=tracker.locate(raw,cal,trackingMap);
    uint16_t strength=0,neighborStrength=0;
    if(detectSensor) {
      if(onTrackingArray) {
        strength=normalizeReading(exitRaw,trackingMin[selectedChannel],trackingMax[selectedChannel]);
        if(strength!=kUncalibratedNormalizedValue && !trackingHigh)strength=1000-strength;
        lastStrength=strength;
      } else strength=forwardSensorStrength(selected,cfg,lastStrength);
      if(strength==kUncalibratedNormalizedValue)return ForwardResult::SensorError;
      if(frontExitPair) {
        neighborStrength=normalizeReading(neighborRaw,trackingMin[neighborChannel],trackingMax[neighborChannel]);
        if(neighborStrength==kUncalibratedNormalizedValue)return ForwardResult::SensorError;
        if(!trackingHigh)neighborStrength=1000-neighborStrength;
      }
    }
    const uint32_t now=millis();
    const uint32_t limit=centerWait ? cfg.sensorMaximumRunMs : runLimit;
    if(limit && uint32_t(now-stageStarted)>=limit)return ForwardResult::Timeout;
    distance+=previousSpeed*(uint32_t(now-previous)/1000.0f);
    previous=now;
    const bool selectedDetected=detectSensor && (centerWait ? stable(strength>=600,now,confirmation) :
                                         edge.update(strength,now,cfg.debounceMs,confirmation));
    const bool neighborDetected=frontExitPair && neighborEdge.update(neighborStrength,now,cfg.debounceMs,confirmation);
    const bool detected=selectedDetected || neighborDetected;
    if(detected || (!detectSensor && distance>=cm)) {
      if(!motion)return sensorExit ? ForwardResult::SensorDetected : ForwardResult::DistanceReached;
      // Switch nominal translation speed only after the distance phase completes.
      // Turn speed is independent; braking still uses the actual incoming output.
      if(!detectSensor){left=approachLeft;right=approachRight;}
      if(centerWait)enter(Stage::Turn);
      else if(action==Action::STOP || action==Action::NS)beginBrake(ForwardResult::StopCompleted);
      else if(centerGroup || frontGroup)enter(Stage::Group);
      // A neighbor exit is not evidence that the selected CROSS sensor saw black.
      else if(action==Action::CROSS)enter(sensorExit && selectedDetected ? Stage::CrossWhite : Stage::CrossBlack);
      else if(forwardArc) {
        beginBrake(ForwardResult::TurnCompleted,true);
      }
      else enter(seekCenter ? Stage::Center : Stage::Turn);
      continue;
    }
    // Geometry/tracker hysteresis stays normalized. Only F_Line's control
    // boundary converts to user-facing [-50,+50] units before recovery and PD.
    followLine(line.valid,line.error,sampleUs);
    delay(1);
  }
}
