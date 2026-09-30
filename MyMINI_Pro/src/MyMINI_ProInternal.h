#pragma once

#include <BNO055.h>

#include "CalibrationManager.h"
#include "DiagnosticsConsole.h"
#include "DualMuxSensors.h"
#include "I2CDevices.h"
#include "TB6612Driver.h"

extern DualMuxSensors sensorArrays;
extern I2CDevices i2cDevices;
extern BNO055 imu;
extern bool imuReady;
extern TB6612Driver motorDriver;
extern DiagnosticsConsole diagnostics;
extern CalibrationManager calibrationManager;
