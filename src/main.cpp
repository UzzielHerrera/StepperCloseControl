#include <Arduino.h>
#include "A4954.h"
#include "PositionSensor.h"
#include "StepperConfig.h"
#include "SerialInterface.h"
#include "StepperController.h"

// TODO: TERMINAR FUNCION CALIBRATE DE STEPPER CONTROLLER
// TODO: REVISAR FUNCIONES MOVEREL Y MOVEABS DE STEPPER CONTROLLER
// TODO: CREAR LIBRERIA STEP/DIR/ENABLE.
// TODO: CREAR LIBRERIA MCP2515.
// TODO: CREAR LIBRERIA COMUNICACIONES CAN.
// TODO: PROBAR TODO EL CIRCUITO.

u8_t motor_unique_id = MOTOR_UNIQUE_ID;

void setup(){
    serialInit();
    a4954_init();
    closeLoopInit();
    closeLoopEnable();
}

void loop(){
    
}
