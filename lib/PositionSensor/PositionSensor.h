#ifndef POSITIONSENSOR_H
#define POSITIONSENSOR_H

#include <Arduino.h>
#include "AS5047.h"
#include "StepperConfig.h"
#include "StepperController.h"

u16_t readAngle();
u16_t readRawAngle();

#endif