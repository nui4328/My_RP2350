Pico 2 (Earle Philhower Arduino core)
Hardware I2C0: SDA=GP4, SCL=GP5
Software I2C: SDA=GP28, SCL=GP1
Connect SDA and SCL pull-up resistors (typically 4.7k each) to 3.3V.
Connect a common GND. Do not connect either GPIO to 5V.
Open Pico2_SoftI2C.ino in Arduino IDE and set Serial Monitor 115200.
SoftI2C.h implements a basic 7-bit master with start/stop, byte read/write, register access and probe.
Limitations: does not handle clock stretching, multi-master arbitration, or bus timeouts. This is a basic prototype rather than a drop-in replacement for Wire/Wire1.
GP1 must not be used for UART/servo while Software I2C is active; GP28 must not be used for ADC/servo concurrently.
