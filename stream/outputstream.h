#pragma once

#include <stddef.h>
#include <stdint.h>

#define EOL "\r\n"
#define CR '\r'
#define LF '\n'

namespace stream {

enum Flags : uint8_t { PGM, PAD_ZERO, PAD_SPACE };

struct Spaces {
    explicit constexpr Spaces(const size_t num) : num(num) {}
    const size_t num;
};

class OutputStream {
    char buffer[34];
public:
    OutputStream();
    virtual ~OutputStream();
    
    OutputStream &operator<<(const Flags flags);
    OutputStream &operator<<(const Spaces spaces);
    OutputStream &operator<<(const char ch);
    OutputStream &operator<<(const char *str);

    OutputStream &operator<<(const   int8_t val);
    OutputStream &operator<<(const  uint8_t val);
    OutputStream &operator<<(const  int16_t val);
    OutputStream &operator<<(const uint16_t val);
    OutputStream &operator<<(const  int32_t val);
    OutputStream &operator<<(const uint32_t val);
    OutputStream &operator<<(const    float val);
protected:
    virtual void write(const char ch) = 0;
    
private:
    uint8_t flags;
};
    
}  // namespace stream
