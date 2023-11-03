#pragma once
#ifndef STEPPER_CONTROLLER_H
#define STEPPER_CONTROLLER_H

#include <Arduino.h>
#include "PIDParameters.h"
#include "StepperConfig.h"
#include "A4954.h"
#include "AS5047.h"
#include "PositionSensor.h"
#include "DirectIO.h"

void IRAM_ATTR update(void);
void closeLoopInit();
void closeLoopEnable();
void closeLoopDisable();
void oneStep();
void calibrate();
void hybridControl();
void openLoop();

#endif