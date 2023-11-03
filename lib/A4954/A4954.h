#ifndef A4954_H
#define A4954_H

#include <Arduino.h>
#include "StepperConfig.h"
#include "DirectIO.h"

void a4954_init();
int mod(int xmod, int mmod);
void output(long theta, int effort);
void relaxed_mode_with_resistance();


#endif