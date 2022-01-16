#include "pin.h"
#include <avr/io.h>

#define _AVR_REG(addr) (*reinterpret_cast<volatile uint8_t *>(addr))

#define _PIN _AVR_REG(reg_pin)
#define _DDR _AVR_REG(reg_pin + 1)
#define _PORT _AVR_REG(reg_pin + 2)

namespace mcu {
    
    Pin::Pin(volatile uint8_t *reg_pin, const uint8_t b, const PinDirection d):
    reg_pin(reg_pin), b(b)
    {
        _DDR &= ~b;
        
        if (d == PIN_OUT) _DDR |= b;
        else if (d == PIN_IN_PULLUP)
            _PORT |= b;
    }
    
    Pin &Pin::operator=(const uint8_t rhs) {
        if (rhs)
            _PORT |= b;
        else
            _PORT &= ~b;
        
        return *this;
    }
    
    bool Pin::operator!() const { return (_PIN & b) ? false : true; }    
}
