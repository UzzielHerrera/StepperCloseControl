#ifndef SERIALINTERFACE_H
#define SERIALINTERFACE_H

#include <Arduino.h>
#include "StepperController.h"
#include "PIDParameters.h"
#include "PositionSensor.h"

#define SerialUSB Serial

void serialInit();
void serialCheck();
void printAngle();
void printSerialMenu();
void printSineGenerator();
void printParameters();
void printParametersEditMenu();
void printParametersEditPositionControl();
void printParametersEditVelocityControl();
void printParametersEditOther();
void antiCoggingCalibration();
void stepResponse();

#endif