#pragma once

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/twi.h>
#include <stdbool.h>
#include <stdint.h>
#include "utils/cpp.h"

namespace mcu {

class I2CMaster {
    
public:
    I2CMaster();
    ~I2CMaster();
    void write(const uint8_t addr, uint8_t *data, const uint8_t len);
    void  read(const uint8_t addr, uint8_t *data, const uint8_t len);
private:
    //DISALLOW_COPY_AND_ASSIGN(I2CMaster);
};

}
