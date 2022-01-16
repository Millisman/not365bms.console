#pragma once
#include <stdint.h>


// volatile uint32_t timer0_millis = 0;

namespace mcu {
    // Simple tick counter for various timing tasks.
    // Using this will enable global interrupts and use
    // timer0.
    class Timer {
        Timer();        
    public:
        // This is lazily initialized. First call will always
        // return 0. TImer overflows every ~49 days.
        static uint32_t millis();
        static void setmillis(const uint32_t nnm);
    private:
        uint32_t millis_impl() const;
    };
}  // namespace mcu
