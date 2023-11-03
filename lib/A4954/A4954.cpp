#include <A4954.h>

int mod(int xmod, int mmod){
    return (xmod % mmod + mmod) % mmod;
}

void a4954_init(){
    // Setup driver pins as outputs
    pinMode(DRIVER_VREF_2, OUTPUT);
    pinMode(DRIVER_VREF_1, OUTPUT);
    pinMode(DRIVER_IN_4, OUTPUT);
    pinMode(DRIVER_IN_3, OUTPUT);
    pinMode(DRIVER_IN_2, OUTPUT);
    pinMode(DRIVER_IN_1, OUTPUT);

    // Setup channel 1 and vref 1
    ledcSetup(DRIVER_VREF1_CHANNEL, DRIVER_FREQ, DRIVER_BITS);
    ledcAttachPin(DRIVER_VREF_1, DRIVER_VREF1_CHANNEL);
    ledcWrite(DRIVER_VREF1_CHANNEL, DRIVER_VREF1_INIT);

    // Setup channel 2 and vref 2
    ledcSetup(DRIVER_VREF2_CHANNEL, DRIVER_FREQ, DRIVER_BITS);
    ledcAttachPin(DRIVER_VREF_2, DRIVER_VREF2_CHANNEL);
    ledcWrite(DRIVER_VREF2_CHANNEL, DRIVER_VREF2_INIT);
}

void output(long theta, int effort){
    int angle_1;
    int angle_2;
    int v_coil_A;
    int v_coil_B;
    int sin_coilA;
    int sin_coilB;
    int phase_multiplier = 10 * steps_per_revolution / 4;

    angle_1 = mod(phase_multiplier * theta, 3600);
    angle_2 = mod((phase_multiplier * theta) + 900, 3600);

    sin_coilA = sin_1[angle_1];
    sin_coilB = sin_1[angle_2];

    v_coil_A = (effort * sin_coilA) / 1024;
    v_coil_B = (effort * sin_coilB) / 1024;

    ledcWrite(DRIVER_VREF1_CHANNEL, abs(v_coil_A));
    ledcWrite(DRIVER_VREF2_CHANNEL, abs(v_coil_B));

    if(v_coil_A >= 0){
        directWriteHigh(DRIVER_IN_2);
        directWriteLow(DRIVER_IN_1);
    } else {
        directWriteLow(DRIVER_IN_2);
        directWriteHigh(DRIVER_IN_1);
    }

    if(v_coil_B >= 0){
        directWriteHigh(DRIVER_IN_4);
        directWriteLow(DRIVER_IN_3);
    } else {
        directWriteLow(DRIVER_IN_4);
        directWriteHigh(DRIVER_IN_3);
    }
}

void relaxed_mode_with_resistance(){
    directWriteHigh(DRIVER_IN_1);
    directWriteHigh(DRIVER_IN_2);
    directWriteHigh(DRIVER_IN_3);
    directWriteHigh(DRIVER_IN_4);
}

