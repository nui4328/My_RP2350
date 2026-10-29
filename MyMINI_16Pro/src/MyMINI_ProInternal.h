#pragma once

#include "MyMINIGyro.h"
#include "MotionHeading.h"

#include "CalibrationManager.h"
#include "DiagnosticsConsole.h"
#include "DualMuxSensors.h"
#include "I2CDevices.h"
#include "TB6612Driver.h"

extern DualMuxSensors sensorArrays;
extern I2CDevices i2cDevices;
extern MyMINIGyro imu;
extern bool imuReady;
extern bool headingReferenceReady;
extern TB6612Driver motorDriver;
extern DiagnosticsConsole diagnostics;
extern CalibrationManager calibrationManager;
