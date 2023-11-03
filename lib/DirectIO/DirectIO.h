#ifndef DIRECT_IO_H
#define DIRECT_IO_H

#include <Arduino.h>

#define IO_REG_TYPE uint32_t

void directWriteLow(IO_REG_TYPE pin);
void directWriteHigh(IO_REG_TYPE pin);
IO_REG_TYPE directRead(IO_REG_TYPE pin);

#endif