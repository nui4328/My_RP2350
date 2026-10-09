#include "DualMuxSensors.h"

using namespace MyMINIConfig;

void DualMuxSensors::begin() {
  for (uint8_t channel = 0; channel < SENSOR_COUNT; ++channel) {
    filteredInitialized_[channel] = false;
  }
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(FRONT_MUX_SELECT[i], OUTPUT);
    pinMode(REAR_MUX_SELECT[i], OUTPUT);
  }
  pinMode(FRONT_MUX_SIGNAL, INPUT);
  pinMode(REAR_MUX_SIGNAL, INPUT);
  analogReadResolution(ADC_BITS);
  selectChannel_(0);
  selectedAtUs_ = micros();
  state_ = ScanState::WaitForSettling;
}

bool DualMuxSensors::configureRead(uint16_t settleUs, uint8_t discardReads,
                                   uint8_t averageReads,
                                   uint8_t smoothingDivisor) {
  if (settleUs < 4u || settleUs > 100u || discardReads < 1u ||
      discardReads > 4u || averageReads < 1u || averageReads > 8u ||
      (smoothingDivisor != 1u && smoothingDivisor != 2u &&
       smoothingDivisor != 4u && smoothingDivisor != 8u)) {
    return false;
  }
  settleUs_ = settleUs;
  discardReads_ = discardReads;
  averageReads_ = averageReads;
  smoothingDivisor_ = smoothingDivisor;
  // Start a new smoother from the most recent raw sample, without altering
  // calibration samples or waiting for several frames to reach the signal.
  for (uint8_t channel = 0; channel < SENSOR_COUNT; ++channel) {
    filteredInitialized_[channel] = frameSequence_ != 0;
    frontSmoothQ8_[channel] = static_cast<int32_t>(frontRaw_[channel]) << 8;
    rearSmoothQ8_[channel] = static_cast<int32_t>(rearRaw_[channel]) << 8;
    frontFiltered_[channel] = frontRaw_[channel];
    rearFiltered_[channel] = rearRaw_[channel];
  }
  return true;
}

bool DualMuxSensors::update(uint32_t nowUs) {
  if (state_ == ScanState::SelectChannel) {
    selectChannel_(channel_);
    selectedAtUs_ = nowUs;
    state_ = ScanState::WaitForSettling;
    return false;
  }

  if (static_cast<uint32_t>(nowUs - selectedAtUs_) < settleUs_) {
    return false;
  }

  frontRaw_[channel_] = readAveraged_(FRONT_MUX_SIGNAL);
  rearRaw_[channel_] = readAveraged_(REAR_MUX_SIGNAL);
  updateFiltered_(channel_);

  ++channel_;
  if (channel_ >= SENSOR_COUNT) {
    channel_ = 0;
    ++frameSequence_;
    lastFrameMicros_ = micros();
    state_ = ScanState::SelectChannel;
    return true;
  }

  state_ = ScanState::SelectChannel;
  return false;
}

void DualMuxSensors::selectChannel_(uint8_t channel) {
  for (uint8_t bit = 0; bit < 4; ++bit) {
    const uint8_t level = (channel >> bit) & 0x01U;
    digitalWrite(FRONT_MUX_SELECT[bit], level);
    digitalWrite(REAR_MUX_SELECT[bit], level);
  }
}

uint16_t DualMuxSensors::readAveraged_(uint8_t pin) const {
  for (uint8_t discard = 0; discard < discardReads_; ++discard) {
    (void)analogRead(pin); // Discard after ADC input/channel switching.
  }
  uint32_t sum = 0;
  for (uint8_t sample = 0; sample < averageReads_; ++sample) {
    sum += static_cast<uint16_t>(analogRead(pin));
  }
  return static_cast<uint16_t>(sum / averageReads_);
}

void DualMuxSensors::updateFiltered_(uint8_t channel) {
  const int32_t frontTarget = static_cast<int32_t>(frontRaw_[channel]) << 8;
  const int32_t rearTarget = static_cast<int32_t>(rearRaw_[channel]) << 8;
  if (!filteredInitialized_[channel]) {
    frontSmoothQ8_[channel] = frontTarget;
    rearSmoothQ8_[channel] = rearTarget;
    filteredInitialized_[channel] = true;
  } else {
    frontSmoothQ8_[channel] +=
        (frontTarget - frontSmoothQ8_[channel]) / smoothingDivisor_;
    rearSmoothQ8_[channel] +=
        (rearTarget - rearSmoothQ8_[channel]) / smoothingDivisor_;
  }
  frontFiltered_[channel] =
      static_cast<uint16_t>((frontSmoothQ8_[channel] + 128) >> 8);
  rearFiltered_[channel] =
      static_cast<uint16_t>((rearSmoothQ8_[channel] + 128) >> 8);
}

void DualMuxSensors::printFrame(Print& output) const {
  output.print(F("frame="));
  output.println(frameSequence_);

  output.print(F("front:"));
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    output.print(' ');
    output.print(frontRaw_[i]);
  }
  output.println();

  output.print(F("rear :"));
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    output.print(' ');
    output.print(rearRaw_[i]);
  }
  output.println();
}
