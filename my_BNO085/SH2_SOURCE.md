# SH-2 protocol source

`sh2.c`, `sh2.h`, `sh2_SensorValue.c`, `sh2_SensorValue.h`,
`sh2_util.c`, `sh2_util.h`, `sh2_hal.h`, `sh2_err.h`, `shtp.c`, and
`shtp.h` are the Hillcrest/CEVA SH-2 protocol implementation. Their
copyright notices are retained in each file. They are distributed under
Apache License 2.0; see `SH2_APACHE_LICENSE.txt` and `SH2_NOTICE.txt`.

The `my_BNO085` I2C transport and robot-facing API are implemented in
`my_BNO085.cpp` and `my_BNO085.h`. This library does not include or link
`Adafruit_BNO08x.h` or Adafruit BusIO. It currently supports one BNO085
instance over I2C, with Game Rotation Vector or Rotation Vector heading
and calibrated gyroscope reports.

Local change (2026-10-20): added an optional `sh2_Hal_t::cancelled` callback
and cancellation checks in SH-2 operation/open loops and SHTP transmit retries.
The robot uses this to bound explicit gyro reset/startup to one shared deadline.
HAL implementations without cancellation must initialize the callback to null.
