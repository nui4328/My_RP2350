#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <string.h>

// MyMINI TFT S3 1.14-inch profile (240x135 landscape). Change these pins
// if your ESP32-S3 TFT board uses a different ST7789 wiring.
constexpr int kTftPower = 21;
constexpr int kTftBacklight = 45;
constexpr int kTftSck = 36;
constexpr int kTftMosi = 35;
constexpr int kTftCs = 7;
constexpr int kTftDc = 39;
constexpr int kTftReset = 40;
constexpr int kI2cSda = 42;
constexpr int kI2cScl = 41;
constexpr uint8_t kSlaveAddress = 0x42;

Adafruit_ST7789 tft(&SPI, kTftCs, kTftDc, kTftReset);
portMUX_TYPE colorLock = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t pendingCommand = 0;
uint8_t pendingOrder[6] = {};
uint8_t pendingSingle = 6;
uint8_t shownOrder[6] = {};
bool haveOrder = false;
uint8_t shownSingle = 6;
uint8_t shownMode = 0;
bool screenEnabled = true;

const char* const colorNames[6] = {
    "RED", "GREEN", "YELLOW", "BLUE", "WHITE", "BLACK"};
const uint16_t colorFill[6] = {
    ST77XX_RED, ST77XX_GREEN, ST77XX_YELLOW,
    ST77XX_BLUE, ST77XX_WHITE, ST77XX_BLACK};

void receiveColorOrder(int byteCount) {
  uint8_t packet[9] = {};
  uint8_t used = 0;
  uint8_t checksum = 0xA5;
  for (int i = 0; i < byteCount; ++i) {
    const int value = Wire.read();
    if (i < 9 && value >= 0) packet[i] = static_cast<uint8_t>(value);
  }
  if (packet[0] != 0xC6) return;
  if (byteCount == 3 && (packet[1] == 2 || packet[1] == 3)) {
    if (static_cast<uint8_t>(checksum ^ packet[0] ^ packet[1]) != packet[2]) return;
    portENTER_CRITICAL(&colorLock);
    pendingCommand = packet[1];
    portEXIT_CRITICAL(&colorLock);
    return;
  }
  if (byteCount == 4 && packet[1] == 4 && packet[2] <= 6) {
    if (static_cast<uint8_t>(checksum ^ packet[0] ^ packet[1] ^ packet[2]) != packet[3]) return;
    portENTER_CRITICAL(&colorLock);
    pendingSingle = packet[2];
    pendingCommand = 4;
    portEXIT_CRITICAL(&colorLock);
    return;
  }
  if (byteCount != 9 || packet[1] != 1) return;
  for (uint8_t i = 0; i < 8; ++i) checksum ^= packet[i];
  if (checksum != packet[8]) return;
  for (uint8_t i = 0; i < 6; ++i) {
    const uint8_t color = packet[i + 2];
    if (color > 5 || (used & (1u << color))) return;
    used |= 1u << color;
  }
  if (used != 0x3f) return;
  portENTER_CRITICAL(&colorLock);
  for (uint8_t i = 0; i < 6; ++i) pendingOrder[i] = packet[i + 2];
  pendingCommand = 1;
  portEXIT_CRITICAL(&colorLock);
}

void drawOrder(const uint8_t order[6]) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(54, 3);
  tft.print("COLOR ORDER");
  // Six columns across the 240 px landscape display: band 1 is leftmost.
  constexpr int kBarWidth = 38;
  constexpr int kBarPitch = 39;
  constexpr int kBarTop = 42;
  constexpr int kBarHeight = 79;
  for (uint8_t i = 0; i < 6; ++i) {
    const uint8_t color = order[i];
    const int x = 3 + i * kBarPitch;
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(x + 16, 29);
    tft.print(i + 1);
    tft.fillRect(x, kBarTop, kBarWidth, kBarHeight, colorFill[color]);
    if (color == 5) tft.drawRect(x, kBarTop, kBarWidth, kBarHeight, ST77XX_WHITE);
    tft.setTextColor(color == 2 || color == 4 ? ST77XX_BLACK : ST77XX_WHITE);
    const int labelWidth = 6 * strlen(colorNames[color]);
    tft.setCursor(x + (kBarWidth - labelWidth) / 2, 104);
    tft.print(colorNames[color]);
  }
}

void drawSingleColor(uint8_t color) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(12, 5);
  tft.print("TCS COLOR");
  const uint16_t fill = color < 6 ? colorFill[color] : 0x8410;
  tft.fillRect(15, 35, 210, 85, fill);
  if (color == 5) tft.drawRect(15, 35, 210, 85, ST77XX_WHITE);
  tft.setTextColor(color == 2 || color == 4 ? ST77XX_BLACK : ST77XX_WHITE);
  tft.setCursor(30, 68);
  tft.print(color < 6 ? colorNames[color] : "UNKNOWN");
}

void setup() {
  Serial.begin(115200);
  pinMode(kTftPower, OUTPUT);
  digitalWrite(kTftPower, HIGH);
  pinMode(kTftBacklight, OUTPUT);
  digitalWrite(kTftBacklight, HIGH);
  SPI.begin(kTftSck, -1, kTftMosi, kTftCs);
  tft.init(135, 240, SPI_MODE0);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(12, 45);
  tft.print("WAIT COLOR SCAN");

  Wire.onReceive(receiveColorOrder);
  if (!Wire.begin(kSlaveAddress, kI2cSda, kI2cScl, 100000)) {
    tft.setCursor(12, 75);
    tft.print("I2C ERROR");
    Serial.println("COLOR I2C: slave begin failed");
  } else {
    Serial.println("COLOR I2C: slave 0x42 ready");
  }
}

void loop() {
  uint8_t command = 0;
  uint8_t incomingSingle = 6;
  uint8_t incoming[6] = {};
  portENTER_CRITICAL(&colorLock);
  command = pendingCommand;
  if (command == 1) {
    for (uint8_t i = 0; i < 6; ++i) incoming[i] = pendingOrder[i];
  }
  if (command == 4) incomingSingle = pendingSingle;
  pendingCommand = 0;
  portEXIT_CRITICAL(&colorLock);
  if (command == 0) return;
  if (command == 2) {
    digitalWrite(kTftBacklight, LOW);
    tft.enableDisplay(false);
    screenEnabled = false;
    Serial.println("COLOR I2C: TFT off");
    return;
  }
  if (!screenEnabled) {
    tft.enableDisplay(true);
    digitalWrite(kTftBacklight, HIGH);
    screenEnabled = true;
  }
  if (command == 3) {
    if (shownMode == 1 && haveOrder) drawOrder(shownOrder);
    else if (shownMode == 4) drawSingleColor(shownSingle);
    Serial.println("COLOR I2C: TFT on");
    return;
  }
  if (command == 4) {
    if (shownMode != 4 || incomingSingle != shownSingle) {
      shownSingle = incomingSingle;
      shownMode = 4;
      drawSingleColor(shownSingle);
      Serial.print("COLOR I2C: single ");
      Serial.println(shownSingle < 6 ? colorNames[shownSingle] : "UNKNOWN");
    }
    return;
  }
  bool sameOrder = shownMode == 1 && haveOrder;
  if (sameOrder)
    for (uint8_t i = 0; i < 6; ++i)
      if (incoming[i] != shownOrder[i]) sameOrder = false;
  if (sameOrder) return;
  for (uint8_t i = 0; i < 6; ++i) shownOrder[i] = incoming[i];
  drawOrder(shownOrder);
  haveOrder = true;
  shownMode = 1;
  Serial.print("COLOR I2C: order");
  for (uint8_t i = 0; i < 6; ++i) {
    Serial.print(' ');
    Serial.print(colorNames[shownOrder[i]]);
  }
  Serial.println();
}
