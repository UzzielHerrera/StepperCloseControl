#include "StepperController.h"
#include "SerialInterface.h"

hw_timer_t *controllerTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR update() {
    portENTER_CRITICAL_ISR(&timerMux);
    directWriteHigh(ISR_LED);

    y = lookup[readRawAngle()];

    if ((y - y_1) < -180.0) wrap_count++;
    else if((y - y_1) > 180.0) wrap_count--;

    yw = y + (360.0 * wrap_count);
    if(mode == 'h'){
        hybridControl();
    } else {
        switch(mode){
            case 'x':   // Position close control
                // Error calculation
                e = r - yw;
                // Integral term calculation
                ITerm += pKi * e;
                if (ITerm > 150.0) ITerm = 150.0;
                else if (ITerm < -150.0) ITerm = -150.0;
                // Derivative term calculation
                DTerm = pLPFa * DTerm - pLPFb * pKd * (yw - yw_1);
                // Control signal = Proportional term + Intergral term + Derivative term
                u = (pKp * e) + ITerm + DTerm;
                break;
            case 'v':   // Velocity close control
                // Filtered velocity similar to Derivative term
                v = vLPFa * v + vLPFb * (yw - yw_1);
                // Error calculation
                e = r - v;
                // Integral term calculation
                ITerm += vKi * e;
                if (ITerm > 200) ITerm = 200;
                else if (ITerm < -200) ITerm = -200;
                // Control signal
                u = ((vKp * e) + ITerm - (vKd * (e - e_1)));
                break;
            case 't':   // Torque close control
                u = 1.0 * r;
                break;
            default:    // No control selected
                u = 0;
                break;
        }
        y_1 = y;

        if (u > 0){
            y += PA;
            if(u > uMAX) u = uMAX;
        }
        else {
            y -= PA;
            if(u < -uMAX) u = -uMAX;
        }

        U = abs(u);
        if(abs(e) < 0.1) directWriteHigh(BUSY_LED);
        else directWriteLow(BUSY_LED);
        
        output(-y, round(U));
    }

    e_2 = e_1;
    e_1 = e;
    u_2 = u_1;
    u_1 = u;
    yw_1 = yw;

    directWriteLow(ISR_LED);
    portEXIT_CRITICAL_ISR(&timerMux);
}

void closeLoopInit() {

    pinMode(ISR_LED, OUTPUT);
    directWriteLow(ISR_LED);
    pinMode(BUSY_LED, OUTPUT);
    directWriteLow(BUSY_LED);

    controllerTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(controllerTimer, &update, true);
    timerAlarmWrite(controllerTimer, 100, true);
    timerAlarmEnable(controllerTimer);
}

void closeLoopEnable() {
    timerAlarmEnable(controllerTimer);
}

void closeLoopDisable() {
    timerAlarmDisable(controllerTimer);
}

void calibrate() {
    int encoderReading = 0;
    int currentEncoderReading = 0;
    int lastEncoderReading = 0;
    int avg = 10;

    int iStart = 0;
    int jStart = 0;
    int stepNum = 0;

    int fullStepReadings[steps_per_revolution];
    int fullStep = 0;
    int ticks = 0;
    float lookUpAngle = 0.0;
    SerialUSB.println("Beginning calibration routine");

    encoderReading = readRawAngle();
    dir = true;
    oneStep();
    delay(500);

    if(readRawAngle() - encoderReading < 0){
        SerialUSB.println("Wired backwards");
        return;
    }

    while(stepNumber != 0){
        dir = (stepNumber > 0) ? true : false;
        oneStep();
        delay(100);
    }

    for(int x = 0; x < steps_per_revolution; x ++){
        encoderReading = 0;
        delay(20);
        lastEncoderReading = readRawAngle();
        for(int reading = 0; reading < avg; reading++){
            currentEncoderReading = readRawAngle();

            if((currentEncoderReading - lastEncoderReading) < (-(count_per_revolution / 2))){
                currentEncoderReading += count_per_revolution;
            }
            else if((currentEncoderReading - lastEncoderReading) > (+(count_per_revolution / 2))){
                currentEncoderReading -= count_per_revolution;
            }

            encoderReading += currentEncoderReading;
            delay(10);
            lastEncoderReading = currentEncoderReading;
        }

        encoderReading = encoderReading / avg;
        if (encoderReading > count_per_revolution)
            encoderReading -= count_per_revolution;
        else if(encoderReading < 0)
        encoderReading += count_per_revolution;

        fullStepReadings[x] = encoderReading;
        if (x % 20 == 0) {
            SerialUSB.println();
            SerialUSB.print(100 * x / steps_per_revolution);
            SerialUSB.print("% ");
        } else {
            SerialUSB.print('.');
        }

        oneStep();
    }

    SerialUSB.println();

    for(int i = 0; i < steps_per_revolution; i++){
        ticks = fullStepReadings[mod(i + 1, steps_per_revolution)] - fullStepReadings[mod(i, steps_per_revolution)];
        if( ticks < -15000){
            ticks += count_per_revolution;
        }
        else if(ticks > 15000) {
            ticks -= count_per_revolution;
        }

        if(ticks > 1){
            for(int j = 0; j < ticks; j++){
                stepNum = mod(fullStepReadings[i] + j, count_per_revolution);
                if(stepNum == 0){
                    iStart = i;
                    jStart = j;
                }
            }
        }

        if(ticks < 1){
            for(int j = -ticks; j > 0; j--){
                stepNum = mod(fullStepReadings[ steps_per_revolution - 1 - i] + j, count_per_revolution);
                if(stepNum == 0){
                    iStart = i;
                    jStart = j;
                }
            }
        }
    }

    // store lookup table
    for( int i = iStart; i  < (iStart + steps_per_revolution + 1); i ++){
        ticks = fullStepReadings[mod((i + 1), steps_per_revolution)] - fullStepReadings[mod(i, steps_per_revolution)];

        if(ticks < -15000){
            ticks += count_per_revolution;
        }
        else if(ticks > 15000){
            ticks -= count_per_revolution;
        }

        if(ticks > 1){
            if(i == iStart){
                for(int j = jStart; j < ticks; j++){
                    lookUpAngle = 0.001 * mod(1000 * ((angle_per_step * i) + ((angle_per_step * j) / float(ticks))), 360000.0);
                    SerialUSB.print(lookUpAngle);
                    SerialUSB.print(", ");
                }
            }
            else if(i == (iStart + steps_per_revolution)){
                for(int j = 0; j < jStart; j++){
                    lookUpAngle = 0.001 * mod(1000 * ((angle_per_step * i) + ((angle_per_step * j) / float(ticks))), 360000.0);
                    SerialUSB.print(lookUpAngle);
                    SerialUSB.print(", ");
                }
            }
            else{
                for(int j = 0; j < ticks; j++){
                    lookUpAngle = 0.001 * mod(1000 * ((angle_per_step * i) + ((angle_per_step * j ) / float(ticks))), 360000.0);
                    SerialUSB.print(lookUpAngle);
                    SerialUSB.print(" , ");
                }
            }
        }
        else if (ticks < 1) {             //similar to above... for case when encoder counts were decreasing during cal routine
            if (i == iStart) {
                for (int j = - ticks; j > (jStart); j--) {
                    lookUpAngle = 0.001 * mod(1000 * (angle_per_step * (i) + (angle_per_step * ((ticks + j)) / float(ticks))), 360000.0);
                    SerialUSB.print(lookUpAngle);
                    SerialUSB.print(" , ");
                }
            }
            else if (i == iStart + steps_per_revolution) {
                for (int j = jStart; j > 0; j--) {
                    lookUpAngle = 0.001 * mod(1000 * (angle_per_step * (i) + (angle_per_step * ((ticks + j)) / float(ticks))), 360000.0);
                    SerialUSB.print(lookUpAngle);
                    SerialUSB.print(" , ");
                }
            }
            else {
                for (int j = - ticks; j > 0; j--) {
                    lookUpAngle = 0.001 * mod(1000 * (angle_per_step * (i) + (angle_per_step * ((ticks + j)) / float(ticks))), 360000.0);
                    SerialUSB.print(lookUpAngle);
                    SerialUSB.print(" , ");
                }
            }
        }
    }
    SerialUSB.println(" ");
}

void oneStep(){
    if (!dir) {
        stepNumber += 1;
    }
    else {
        stepNumber -= 1;
    }

    //output(1.8 * stepNumber, 64); //updata 1.8 to aps..., second number is control effort
    output(angle_per_step * stepNumber, (int)(0.33 * uMAX));
    delay(10);
}

void hybridControl(){
    static int missed_steps = 0;
    static float iLevel = 0.6;  //hybrid stepping current level.  In this mode, this current is continuous (unlike closed loop mode). Be very careful raising this value as you risk overheating the A4954 driver!
    static float rSense = 0.15;

    if (yw < r - angle_per_step) {
        missed_steps -= 1;
    }
    else if (yw > r + angle_per_step) {
        missed_steps += 1;
    }

    output(0.1125 * (-(r - missed_steps)), (255 / 3.3) * (iLevel * 10 * rSense));
}
