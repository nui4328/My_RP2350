#pragma once

#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <sstream>
#include <string>

using std::isnan;

using byte = uint8_t;

constexpr uint8_t INPUT_PULLUP = 0x2;
constexpr uint8_t OUTPUT = 0x1;
constexpr uint8_t LOW = 0x0;
constexpr uint8_t HIGH = 0x1;

#define F(value) value
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

inline uint8_t highByte(uint16_t value) { return static_cast<uint8_t>(value >> 8); }
inline uint8_t lowByte(uint16_t value) { return static_cast<uint8_t>(value & 0xff); }

template <typename T>
T constrain(T value, T low, T high) {
  return value < low ? low : (value > high ? high : value);
}

inline long map(long value, long fromLow, long fromHigh, long toLow, long toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

class Stream {
 public:
  virtual ~Stream() = default;
  template <typename T>
  void print(const T &value) {
    std::ostringstream buffer;
    buffer << value;
    contents += buffer.str();
  }
  void println() { contents += '\n'; }

  std::string contents;
};

extern Stream Serial;

unsigned long millis();
unsigned long micros();
void delay(unsigned long duration);
void delayMicroseconds(unsigned int duration);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogReadResolution(int bits);
void analogWrite(uint8_t pin, int value);
void analogWriteFreq(uint32_t frequency);
void analogWriteRange(uint32_t range);
void tone(uint8_t pin, unsigned int frequency);
void tone(uint8_t pin, unsigned int frequency, unsigned long duration);
void noTone(uint8_t pin);
