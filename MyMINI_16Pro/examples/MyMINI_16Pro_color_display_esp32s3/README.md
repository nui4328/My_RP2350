# Six-color scan display over I2C

Upload `MyMINI_16Pro_color` to the Pico 2 and this sketch to the ESP32-S3
TFT 1.14-inch board. The display sketch uses the local MyMINI TFT S3
240x135 ST7789 pin profile. Check its constants before uploading if your
ESP32-S3 board has a different TFT pinout.

| Pico 2 MyMINI_16Pro | ESP32-S3 TFT |
| --- | --- |
| GP4 / SDA | GPIO42 / SDA |
| GP5 / SCL | GPIO41 / SCL |
| GND | GND |

Both boards use 3.3 V I2C levels. Keep the existing GP4/GP5 sensor wiring;
connect the ESP32 in parallel to that bus. The display is I2C slave `0x42`.
Both boards use 100 kHz. The bus needs 3.3 V pullups, normally already present
on the sensor bus. Do not add another strong set of pullups without checking
the existing bus.

The Pico sends a nine-byte packet after `scanSixColorBands()` returns a
complete `6/6` order. It resends the same order every two seconds. A partial
scan sends nothing. The display validates marker, command, checksum, and six
distinct color IDs before showing the result. ID order is `RED=0`, `GREEN=1`,
`YELLOW=2`, `BLUE=3`, `WHITE=4`, `BLACK=5`.
The TFT shows the six bands in one horizontal row, band 1 at the left and
band 6 at the right.

To bypass scanning, call the Pico sketch functions after `setup_color()`:

```cpp
sendColorBands(BAND_GREEN, BAND_YELLOW, BAND_BLACK,
               BAND_RED, BAND_BLUE, BAND_WHITE);
turnOffTft();  // Turns off the ST7789 display and backlight.
turnOnTft();   // Turns it on and restores the last color order.
```

`sendColorBands` requires each of the six colors exactly once. The off/on
commands are three-byte I2C packets. While off, the Pico repeats the off
command instead of repeating the color order; sending a new order turns the
display on again.

To send one TCS reading, call `readFast()` first; `getColor()` classifies the
most recently read channels:

```cpp
if (tcs0.readFast()) {
  String color = tcs0.getColor();
  sendColorBands(color); // Shows one color, including UNKNOWN.
}
```

This one-color overload sends a four-byte packet and replaces the display's
last shown content. The six-band overload remains available for an order.

Serial Monitor is 115200 on both boards. Pico prints `COLOR I2C: display ACK`
when the I2C transaction succeeds. ESP32 prints the received
order. I2C ACK confirms delivery to the slave controller; the ESP32 Serial
log and TFT confirm successful decoding and display.
