#include <avr/wdt.h>
#include "mcu/watchdog.h"

namespace mcu {    
    void Watchdog::enable(const uint8_t value) { wdt_enable(value); }
    void Watchdog::disable() { wdt_disable(); }
    void Watchdog::reset() { wdt_reset(); }
    void Watchdog::forceRestart() {
        wdt_enable(WDTO_15MS);
        while (1) {
        }
    }

}
