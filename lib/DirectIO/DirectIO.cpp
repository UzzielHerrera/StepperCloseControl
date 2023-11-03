#include "DirectIO.h"

// static inline __attribute__((always_inline))
void directWriteLow(IO_REG_TYPE pin){
    if(pin < 32)
        GPIO.out_w1tc = ((u32_t) 1 << pin);
    else if (pin > 31)
        GPIO.out1_w1tc.val = ((u32_t) 1 << (pin - 32));
}

// static inline __attribute__((always_inline))
void directWriteHigh(IO_REG_TYPE pin){
    if (pin < 32)
        GPIO.out_w1ts = ((u32_t) 1 << pin);
    else if (pin > 31)
        GPIO.out1_w1ts.val = ((u32_t) 1 << (pin - 32));
}

// static inline __attribute__((always_inline))
IO_REG_TYPE directRead(IO_REG_TYPE pin){
    if (pin < 32)
        return (GPIO.in >> pin) & 0x1;
    else if (pin > 31)
        return (GPIO.in1.val >> (pin - 32)) & 0x1;
    
    return 0;
}