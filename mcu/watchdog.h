#pragma once

#include "utils/cpp.h"


namespace mcu {
    
    class Watchdog {
        Watchdog();        
    public:
        static void enable(const uint8_t value);
        static void disable();
        static void reset();
        static void forceRestart();
    private:
        DISALLOW_COPY_AND_ASSIGN(Watchdog);
    };
}
