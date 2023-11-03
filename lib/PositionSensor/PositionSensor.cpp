#include "PositionSensor.h"

AS5047 position_sensor(AS5047_CLK, AS5047_MISO, AS5047_MOSI, AS5047_SS);

u16_t readAngle(){
    const int avg = 10;
    int encoderReading = 0;
    
    closeLoopDisable();

    for(int reading = 0; reading < avg; reading++){
        encoderReading += readRawAngle();
        delay(5);
    }

    closeLoopEnable();

    return lookup[encoderReading / avg];
}

u16_t readRawAngle(){
    return position_sensor.readRawAngleUncompensated();
}
