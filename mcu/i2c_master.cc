#include "i2c_master.h"

#define I2C_FREQ 100000UL

//                      TWINT          TWEA          TWSTA          TWSTO          TWEN          TWIE
#define I2C_START (1 << TWINT) | (1 << TWEA) | (1 << TWSTA) |                (1 << TWEN) | (1 << TWIE)
#define I2C_STOP  (1 << TWINT) |                              (1 << TWSTO) | (1 << TWEN)
#define I2C_SEND  (1 << TWINT) | (1 << TWEA) |                               (1 << TWEN) | (1 << TWIE)
#define I2C_ACK   (1 << TWINT) | (1 << TWEA) |                               (1 << TWEN) | (1 << TWIE)
#define I2C_NACK  (1 << TWINT) |                                             (1 << TWEN) | (1 << TWIE)
#define I2C_RESET                                                            (1 << TWEN)

namespace {
    
volatile uint8_t *i2c_data = 0;
volatile uint8_t i2c_len = 0;
uint8_t i2c_addr = 0;

ISR(TWI_vect) {
    switch (TW_STATUS) {
        // Master read/write --------------------------------------
        
        case TW_START:
        case TW_REP_START:
            TWDR = i2c_addr;
            TWCR = I2C_SEND;
            break;
            
        case TW_MT_ARB_LOST:  // Same as TW_MR_ARB_LOST
            TWCR = I2C_START;
            break;
            
            // Master write -------------------------------------------
            
        case TW_MT_SLA_ACK:
        case TW_MT_DATA_ACK:
            if (i2c_len) {
                --i2c_len;
                TWDR = *i2c_data++;
                TWCR = I2C_SEND;
            } else {
                TWCR = I2C_STOP;
                while (TWCR & (1 << TWSTO)) {} // TODO Blocking
            }
            break;
            
        case TW_MT_SLA_NACK:
        case TW_MT_DATA_NACK:
            TWCR = I2C_STOP;
            while (TWCR & (1 << TWSTO)) {} // // TODO Blocking
            break;
            
            // Master read --------------------------------------------
            
        case TW_MR_DATA_ACK:
            *i2c_data++ = TWDR;
            --i2c_len;
            [[fallthrough]];
        case TW_MR_SLA_ACK:
            TWCR = (i2c_len > 1) ? I2C_ACK : I2C_NACK;
            break;
            
        case TW_MR_DATA_NACK:
            *i2c_data = TWDR;
            --i2c_len;
            [[fallthrough]];
        case TW_MR_SLA_NACK:
            TWCR = I2C_STOP;
            break;
            
            // Anything else, just disable interrupts ...
        default:
            TWCR = I2C_RESET;
            break;
    }
}

void i2c_rw(const uint8_t addr, uint8_t *data, const uint8_t len) {
    i2c_addr = addr;
    i2c_data = data;
    i2c_len = len;
    
    // Start send/receive
    TWCR = I2C_START;
    
    // Busy wait
    while (TWCR & (1 << TWIE)) {}; // TODO Blocking
}
    
}  // namespace

namespace mcu {

I2CMaster::I2CMaster() {
    power_twi_enable();

    // Enable internal pull-ups
    // Any system using this REALLY should be using external pull-ups
    // as well (4.7k recommended). The internal pull-ups are too weak
    // for pretty much any cable length (if cables are used).
    PORTC |= (1 << PC4) | (1 << PC5);
    TWSR = 0x00;
    TWBR = (F_CPU / I2C_FREQ - 16) / 2;
    sei();
}

I2CMaster::~I2CMaster() {
    power_twi_disable();
}

void I2CMaster::write(const uint8_t addr, uint8_t *data, const uint8_t len) {
    i2c_rw(TW_WRITE | (addr << 1), data, len);
}

void I2CMaster::read(const uint8_t addr, uint8_t *data, const uint8_t len) {
    i2c_rw(TW_READ | (addr << 1), data, len);
}

}
