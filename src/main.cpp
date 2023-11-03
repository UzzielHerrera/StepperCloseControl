#include <Arduino.h>
#include "StepperConfig.h"
#include "A4954.h"
#include "AS5047.h"
#include "StepperController.h"

u8_t motor_unique_id = MOTOR_UNIQUE_ID;

void setup(){
    a4954_init();
    closeLoopInit();
    closeLoopEnable();
}

void loop(){
    
}
