#include "outputstream.h"

#include <avr/pgmspace.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

namespace stream {

OutputStream::OutputStream() : flags(0) {}

OutputStream::~OutputStream() {}

OutputStream &OutputStream::operator<<(const Flags flag) {
    flags |= (1 << flag);
    return *this;
}

OutputStream &OutputStream::operator<<(const Spaces spaces) {
    for (size_t i = 0; i != spaces.num; ++i) *this << ' ';
    return *this;
}

OutputStream &OutputStream::operator<<(const char ch) {
    write(ch);
    return *this;
}

OutputStream &OutputStream::operator<<(const char *str) {
    if (flags & (1 << Flags::PGM)) {
        while (pgm_read_byte(str)) write(pgm_read_byte(str++));
    } else {
        while (*str) write(*str++);
    }
    flags = 0;
    return *this;
}


OutputStream &OutputStream::operator<<(const   int8_t val){
    memset(buffer, 0, sizeof(*buffer));
    itoa(val, buffer, 10);
    return *this << buffer;
}

OutputStream &OutputStream::operator<<(const  int16_t val){
    memset(buffer, 0, sizeof(*buffer));
    itoa(val, buffer, 10);
    return *this << buffer;
    
}

OutputStream &OutputStream::operator<<(const  int32_t val){
    memset(buffer, 0, sizeof(*buffer));
    itoa(val, buffer, 10);
    return *this << buffer;
}

OutputStream &OutputStream::operator<<(const uint8_t val) {
    if (flags == 0) {
        memset(buffer, 0, sizeof(*buffer));
        utoa(val, buffer, 10);
        return *this << buffer;
    }
    
    else if (flags & ((1 << Flags::PAD_ZERO) | (1 << Flags::PAD_SPACE))) {
        if (val < 10)
            *this << static_cast<char>(flags & (1 << Flags::PAD_ZERO) ? '0' : ' ');
        else
            *this << static_cast<char>('0' + val / 10);
        *this << static_cast<char>('0' + val % 10);
    }
    
    flags = 0;
    return *this;
}

OutputStream &OutputStream::operator<<(const uint16_t val){
    memset(buffer, 0, sizeof(*buffer));
    ltoa(val, buffer, 10);
    return *this << buffer;
}

OutputStream &OutputStream::operator<<(const uint32_t val){
    memset(buffer, 0, sizeof(*buffer));
    ltoa(val, buffer, 10);
    return *this << buffer;
    
}

OutputStream &OutputStream::operator<<(const float val) {
    double number = val;
    uint8_t digits = 2; // TODO
    
    if (isnan(number)) return *this << "nan";
    if (isinf(number)) return *this << "inf";
    if (number > 4294967040.0) return *this << "ovf";  // constant determined empirically
    if (number <-4294967040.0) return *this << "ovf";  // constant determined empirically
    
    // Handle negative numbers
    if (number < 0.0) {
        write('-');
        number = -number;
    }
    
    // Round correctly so that print(1.999, 2) prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i=0; i<digits; ++i)
        rounding /= 10.0;
    
    number += rounding;
    
    // Extract the integer part of the number and print it
    uint32_t int_part = (uint32_t)number;
    double remainder = number - (double)int_part;
    *this << int_part;
    
    // Print the decimal point, but only if there are digits beyond
    if (digits > 0) { write('.'); }
    
    // Extract digits from the remainder one at a time
    while (digits-- > 0) {
        remainder *= 10.0;
        uint16_t toPrint = (uint16_t)(remainder);
        *this << toPrint ;
        remainder -= toPrint; 
    }
    return *this;
}



}  // namespace stream
