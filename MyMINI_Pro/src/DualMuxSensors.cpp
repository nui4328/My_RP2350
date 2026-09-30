#include "DualMuxSensors.h"

using namespace MyMINIConfig;

void DualMuxSensors::begin() {
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

bool DualMuxSensors::update(uint32_t nowUs) {
  if (state_ == ScanState::SelectChannel) {
    selectChannel_(channel_);
    selectedAtUs_ = nowUs;
    state_ = ScanState::WaitForSettling;
    return false;
  }

  if (static_cast<uint32_t>(nowUs - selectedAtUs_) < MUX_SETTLE_US) {
    return false;
  }

  frontRaw_[channel_] = readAveraged_(FRONT_MUX_SIGNAL);
  rearRaw_[channel_] = readAveraged_(REAR_MUX_SIGNAL);

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
  (void)analogRead(pin); // Discard after ADC input/channel switching.
  uint32_t sum = 0;
  for (uint8_t sample = 0; sample < MUX_AVERAGE_SAMPLES; ++sample) {
    sum += static_cast<uint16_t>(analogRead(pin));
  }
  return static_cast<uint16_t>(sum / MUX_AVERAGE_SAMPLES);
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
