
#include "uartstream.h"

namespace stream {

UartStream::UartStream(mcu::Usart &uart) : uart(uart) {}

bool UartStream::avail() { return uart.avail(); }

UartStream &UartStream::operator>>(char &ch) {
    ch = uart.read();
    return *this;
}

void UartStream::write(const char ch) { uart.write(ch); }

}
