#pragma once

#include <BNO055.h>
#include <my_BNO085.h>
#include <math.h>
#include "GyroModel.h"

// Keep the motion controller's existing BNO055-shaped interface while the
// selected sensor supplies heading and rate readings.
class MyMINIGyro {
public:
  MyMINIGyro() : bno085_(0x4B, Wire1) {}

  bool select(GyroModel model, int8_t yawSign, TwoWire& bno085Wire) {
    if (started_ || (model != GyroModel::BNO055 && model != GyroModel::BNO085) ||
        (yawSign != 1 && yawSign != -1)) return false;
    if (!bno085_.selectWire(bno085Wire)) return false;
    model_ = model;
    yawSign_ = yawSign;
    bno085Wire_ = &bno085Wire;
    return true;
  }
  GyroModel model() const { return model_; }
  TwoWire& bus() const { return model_ == GyroModel::BNO085 ? *bno085Wire_ : Wire; }

  bool begin(uint8_t address, TwoWire& wire) {
    started_ = model_ == GyroModel::BNO055
        ? bno055_.begin(address, wire) : bno085_.begin();
    error_ = started_ ? BNO055UpdateError::None
                      : BNO055UpdateError::ReadFailed;
    return started_;
  }

  bool update() {
    if (!started_) {
      error_ = BNO055UpdateError::ReadFailed;
      return false;
    }
    if (model_ == GyroModel::BNO055) return bno055_.update();
    bno085_.update();
    if (!bno085_.angleReady()) {
      error_ = bno085_.headingJumped()
          ? BNO055UpdateError::ImplausibleHeading
          : BNO055UpdateError::ReadFailed;
      return false;
    }
    error_ = BNO055UpdateError::None;
    return true;
  }

  float yaw() const {
    if (model_ == GyroModel::BNO055) return bno055_.yaw();
    float wrapped = fmodf(yawSign_ * bno085_.yawRaw(), 360.0f);
    if (wrapped < 0.0f) wrapped += 360.0f;
    return wrapped >= 360.0f ? 0.0f : wrapped;
  }
  float yawRaw() const {
    return model_ == GyroModel::BNO055 ? bno055_.yawRaw()
                                       : yawSign_ * bno085_.yawRaw();
  }
  float gyro(char axis) const {
    if (model_ == GyroModel::BNO055) return bno055_.gyro(axis);
    const float rate = bno085_.rate(axis);
    return axis == 'z' || axis == 'Z' ? yawSign_ * rate : rate;
  }
  bool resetAngles() {
    return model_ == GyroModel::BNO055 ? bno055_.resetAngles()
                                       : bno085_.resetYaw();
  }
  BNO055UpdateError lastUpdateError() const {
    return model_ == GyroModel::BNO055 ? bno055_.lastUpdateError() : error_;
  }
  void printBno085Diagnostics(Print& output) const {
    if (model_ == GyroModel::BNO085) bno085_.printDiagnostics(output);
  }
  void setLPF(float alpha) {
    if (model_ == GyroModel::BNO055) bno055_.setLPF(alpha);
  }
  bool calibrate(uint8_t buzzerPin, bool forceNew) {
    return model_ == GyroModel::BNO055 &&
           bno055_.calibrate(buzzerPin, forceNew);
  }
  bool calibrationStatus(BNO055CalibrationStatus& status) {
    return model_ == GyroModel::BNO055 &&
           bno055_.calibrationStatus(status);
  }
  bool saveFusionCalibrationProfile(bool resetYaw = true) {
    return model_ == GyroModel::BNO055 &&
           bno055_.saveFusionCalibrationProfile(resetYaw);
  }

private:
  GyroModel model_ = GyroModel::BNO055;
  int8_t yawSign_ = 1;
  BNO055 bno055_;
  my_BNO085 bno085_;
  TwoWire* bno085Wire_ = &Wire1;
  bool started_ = false;
  BNO055UpdateError error_ = BNO055UpdateError::None;
};
