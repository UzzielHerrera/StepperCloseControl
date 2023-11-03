#include "StepperController.h"

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
