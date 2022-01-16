#pragma once

#include "mcu/usart.h"
#include "stream/outputstream.h"

namespace stream {

class UartStream : public OutputStream {
public:
    UartStream(mcu::Usart &uart);
    
    bool avail();
    UartStream &operator>>(char &ch);
    
private:
    void write(const char ch) override;
    
    mcu::Usart &uart;
};

}
