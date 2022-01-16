#pragma once
#include <avr/io.h>
#include <avr/interrupt.h>
#include "utils/atomic.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdint.h>


#include "utils/cpp.h"

namespace mcu {
    
class Usart {
public:
    static Usart &get();
    uint8_t read();
    void write(const uint8_t b);
    uint16_t avail();
    bool isActivity();
    void enable_TxRx()  { UCSR0B |=  ((1 << RXEN0) | (1 << TXEN0)); }
    void disable_TXRx() { UCSR0B &= ~((1 << RXEN0) | (1 << TXEN0)); }
private:
    Usart(uint32_t baud, uint8_t config);
    DISALLOW_COPY_AND_ASSIGN(Usart);
};

}
