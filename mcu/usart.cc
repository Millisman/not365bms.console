#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include "mcu/usart.h"
#include "utils/atomic.h"


#define USART0_RX_BUFFER_SIZE 128
#define USART0_TX_BUFFER_SIZE 128

namespace {

static volatile bool Activity;
    
static volatile uint8_t USART0_RX_BUFFER[USART0_RX_BUFFER_SIZE];
static volatile uint8_t USART0_RX_BUFFER_HEAD;
static volatile uint8_t USART0_RX_BUFFER_TAIL;

static volatile uint8_t USART0_TX_BUFFER[USART0_TX_BUFFER_SIZE];
static volatile uint8_t USART0_TX_BUFFER_HEAD;
static volatile uint8_t USART0_TX_BUFFER_TAIL;

ISR(USART_RX_vect) {
    if (bit_is_set(UCSR0A, UPE0)) {
        UDR0;
    } else {
        Activity = true;
        uint8_t c = UDR0;
        uint8_t i = (USART0_RX_BUFFER_HEAD + 1 >= USART0_RX_BUFFER_SIZE) ? 0 : USART0_RX_BUFFER_HEAD + 1;
        if (i != USART0_RX_BUFFER_TAIL) {
            USART0_RX_BUFFER[USART0_RX_BUFFER_HEAD] = c;
            USART0_RX_BUFFER_HEAD = i;
        }
    }
}

ISR(USART_TX_vect) {
    if (USART0_TX_BUFFER_HEAD != USART0_TX_BUFFER_TAIL) {
        uint8_t c = USART0_TX_BUFFER[USART0_TX_BUFFER_TAIL];
        if (++USART0_TX_BUFFER_TAIL >= USART0_TX_BUFFER_SIZE) USART0_TX_BUFFER_TAIL = 0;  // хвост двигаем
        UDR0 = c;
    }
}
    
}


namespace mcu {


    
    
Usart &Usart::get() {
    static Usart usart(115200, (1<<UCSZ01) | (1<<UCSZ00));
    return usart;
}

Usart::Usart(uint32_t baud, uint8_t config) {
    USART0_RX_BUFFER_HEAD = 0;
    USART0_RX_BUFFER_TAIL = 0;
    USART0_TX_BUFFER_HEAD = 0;
    USART0_TX_BUFFER_TAIL = 0;
    // Try u2x mode first
    uint16_t baud_setting = (F_CPU / 4 / baud - 1) / 2;
    UCSR0A = 1 << U2X0;
    // hardcoded exception for 57600 for compatibility with the bootloader
    // shipped with the Duemilanove and previous boards and the firmware
    // on the 8U2 on the Uno and Mega 2560. Also, The baud_setting cannot
    // be > 4095, so switch back to non-u2x mode if the baud rate is too
    // low.
    //         if (((F_CPU == 16000000UL) && (baud == 57600)) || (baud_setting >4095))
    //         {
    //             UCSR0A = 0;
    //             baud_setting = (F_CPU / 8 / baud - 1) / 2;
    //         }
    
    // assign the baud_setting, a.k.a. ubrr (USART Baud Rate Register)
    UBRR0H = baud_setting >> 8;
    UBRR0L = baud_setting;
    UCSR0B = ((1<<RXCIE0) | (1<<TXCIE0)); // (1<<TXEN0) | (1<<RXEN0) | 
    enable_TxRx();
    UCSR0C = config; //((1<<UCSZ01) | (1<<UCSZ00));
    Activity = false;
}

bool Usart::isActivity() {
    if (Activity) {
        Activity = false;
        return true;
    }
    return false;
}


uint8_t Usart::read() {
    if (USART0_RX_BUFFER_HEAD == USART0_RX_BUFFER_TAIL) return -1;
    uint8_t c = USART0_RX_BUFFER[USART0_RX_BUFFER_TAIL];
    if (++USART0_RX_BUFFER_TAIL >= USART0_RX_BUFFER_SIZE) USART0_RX_BUFFER_TAIL = 0;  // хвост двигаем
    return c;
}

uint16_t Usart::avail() {
    return ((uint16_t)(USART0_RX_BUFFER_SIZE + USART0_RX_BUFFER_HEAD - USART0_RX_BUFFER_TAIL)) % USART0_RX_BUFFER_SIZE;
}


void Usart::write(const uint8_t data) {
    uint8_t i = (USART0_TX_BUFFER_HEAD + 1 >= USART0_TX_BUFFER_SIZE) ? 0 : USART0_TX_BUFFER_HEAD + 1;
    // ждать освобождения места в буфере
    while ( (i + 1) == USART0_TX_BUFFER_TAIL);
    
    // Не сохранять новые данные если нет места
    if (i != USART0_TX_BUFFER_TAIL) {
        USART0_TX_BUFFER[USART0_TX_BUFFER_HEAD] = data;
        USART0_TX_BUFFER_HEAD = i;
    }
    while (!(UCSR0A & (1<<UDRE0)));
    if (USART0_TX_BUFFER_HEAD != USART0_TX_BUFFER_TAIL) {
        uint8_t c = USART0_TX_BUFFER[USART0_TX_BUFFER_TAIL];
        if (++USART0_TX_BUFFER_TAIL >= USART0_TX_BUFFER_SIZE) USART0_TX_BUFFER_TAIL = 0;  // хвост двигаем
        UDR0 = c;
    }
}
    
}  // namespace mcu
