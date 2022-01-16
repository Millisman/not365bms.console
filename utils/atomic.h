#pragma once

#include <avr/interrupt.h>


namespace utils {
    
    class Atomic {
        uint8_t sreg;
    public:
        __attribute__((always_inline)) inline Atomic() { sreg = SREG; cli(); }
        __attribute__((always_inline)) inline ~Atomic() { SREG = sreg; sei(); }
    };
    
}  // namespace utils

