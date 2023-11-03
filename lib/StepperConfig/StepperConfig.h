#ifndef STEPPERCONFIG_H
#define STEPPERCONFIG_H

// === Delta Close Loop Stepper Controller version ===
#define DELTA_STEPPER_CONTROLLER_VERSION_MAYOR '1'
#define DELTA_STEPPER_CONTROLLER_VERSION_MINOR '0'
#define DELTA_STEPPER_CONTROLLER_VERSION_PATCH '0'

// === Motor Unique ID ===
#define MOTOR_UNIQUE_ID 1

// === AS5047 Magnetic Position Sensor ===
#define AS5047_CLK 12
#define AS5047_MOSI 11
#define AS5047_MISO 13
#define AS5047_SS 10

// === A4954 Motor Driver ===
#define KEEP_RESISTANCE_WHEN_DETACHED 1

#define DRIVER_IN_1 47
#define DRIVER_IN_2 48
#define DRIVER_IN_3 39
#define DRIVER_IN_4 45

#define DRIVER_VREF_1 21
#define DRIVER_VREF_2 40

#define DRIVER_VREF1_CHANNEL 0
#define DRIVER_VREF2_CHANNEL 0
#define DRIVER_FREQ 20000
#define DRIVER_BITS 15

#define DRIVER_VREF1_INIT 80
#define DRIVER_VREF2_INIT 80

// === Current Parameters ===
extern volatile float Ts;
extern volatile float Fs;

extern volatile float pKp;
extern volatile float pKi;
extern volatile float pKd;
extern volatile float pLPF;

extern volatile float vKp;
extern volatile float vKi;
extern volatile float vKd;
extern volatile float vLPF;

extern const float lookup[];

extern volatile float pLPFa;
extern volatile float pLPFb;
extern volatile float vLPFa;
extern volatile float vLPFb;

extern const int steps_per_revolution;
extern const float angle_per_step;
extern int count_per_revolution;
extern const float stepangle;

extern volatile float PA;
extern const float iMAX;
extern const float rSense;
extern volatile int uMAX;

extern const int sin_1[];

// === Stepper Controller ===
#define STEPPER_CPR 200
#define ISR_LED 1
#define BUSY_LED 2

// === Stepper Controller ===

#endif