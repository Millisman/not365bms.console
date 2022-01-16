/* Shell console for battery management based bq769x Ic
 * Copyright (c) 2022 Sergey Kostanoy (https://arduino.uno)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include "mcu/watchdog.h"
#include "mcu/usart.h"
#include "mcu/pin.h"
#include "stream/uartstream.h"
#include <avr/sleep.h>
#include "protocol/console.h"

#define PIN_LED_SCK MAKEPIN(B, 5, OUT)

void activate_INT0();
void activate_pin_change_int();
void deactivate_pin_change_int();
static volatile bool isrWU = false;
static volatile bool isrRX = false;

// ISR(INT1_vect) // ISR(INT2_vect)
ISR(INT0_vect)   { isrWU = true; }
ISR(PCINT2_vect) { isrRX = true; }
static volatile uint16_t timer2ovf = 0;
ISR(TIMER2_OVF_vect) { timer2ovf++; }

int main() {
    MCUSR = 0;
    mcu::Watchdog::disable();
    sei();
    mcu::Pin led(PIN_LED_SCK);   
    led = 1;
    mcu::Usart &ser = mcu::Usart::get();
    protocol::Console proto; // Console load conf
    power_adc_disable();
    power_spi_disable();
    power_timer1_disable();
    power_twi_disable(); // managed by I2CMaster::    
    activate_INT0();
    _delay_ms(100);
    led = 0;
    _delay_ms(900);
    mcu::Watchdog::enable(WDTO_4S);
    proto.begin(); // init  bq769x0, print
    uint32_t last_Activity = 0;

    while (1) {
        if (ser.isActivity() || proto.update(led, isrWU) || proto.Recv()) {
            last_Activity = mcu::Timer::millis();
        }

        if((uint32_t)(mcu::Timer::millis() - last_Activity) >= 60000) { // 1 mim
            ser.disable_TXRx();
            cli();
            set_sleep_mode(SLEEP_MODE_PWR_SAVE);
            // Timer/Counter2 8-byte OVF 12MHz /1024 = 21.76ms            
            TCCR2A = 0;
            TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
            TCNT2 = 0;
            TIMSK2 = (1 << TOIE2);
            activate_pin_change_int();
            mcu::Watchdog::reset();
            sleep_enable();
            sei();
            isrWU = false;
            isrRX = false;
            do {
                sleep_cpu();
            } while (!isrWU && !isrRX && (timer2ovf < 20));
            sleep_disable();
            deactivate_pin_change_int();
            ser.enable_TxRx();
            // Disable Timer/Counter2 and add elapsed time to Arduinos 'timer0_millis'
            TCCR2B = 0;
            TIMSK2 = 0;
            float elapsed_time = timer2ovf * 21.76 + TCNT2 * 21.76 / 255.0;
            mcu::Timer::setmillis(mcu::Timer::millis() + (uint32_t)elapsed_time);
            timer2ovf = 0;
            if(isrRX) last_Activity = mcu::Timer::millis();
            isrRX = false;
            isrWU = true; // forcing
        }
        mcu::Watchdog::reset();
    }
}


#define CHANGE  1
#define FALLING 2
#define RISING  3

void activate_INT0() {
    EICRA = (1 << ISC01) | (1 << ISC00) | (RISING << ISC00);
    EIMSK = (1 << INT0);
    //----------------------------------------------------------------- INT1_vect
    //EICRA = (1 << ISC11) | (1 << ISC10) | (RISING << ISC10);
    //EIMSK = (1 << INT1);
    //----------------------------------------------------------------- INT2_vect
    //EICRA = (1 << ISC21) | (1 << ISC20) | (RISING << ISC20);
    //EIMSK = (1 << INT2);
}

void activate_pin_change_int() {
    // Enable pin change interrupt on the PCINT16 pin using Pin Change Mask Register 2 (PCMSK2)
    PCMSK2 |= (1 << PCINT16); // PD0 RXD
    // Enable pin change interrupt 2 using the Pin Change Interrrupt Control Register (PCICR)
    PCICR |= (1 << PCIE2);    
}

void deactivate_pin_change_int() {
    // Disable pin change interrupt 2 using the Pin Change Interrrupt Control Register (PCICR)
    PCICR &= ~(1 << PCIE2);
    // Enable pin change interrupt on the PCINT16 pin using Pin Change Mask Register 2 (PCMSK2)
    PCMSK2 &= ~(1 << PCINT16); // PD0 RXD
}
