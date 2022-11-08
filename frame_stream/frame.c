#include "frame.h"
void encode_uint32(my_uint32_t n, char* dst) {
    for (int i = 3; i >= 0; i--) {
        dst[i] = n >> (8 * (3 - i));
    }
}

my_uint32_t decode_uint32(const char* src) {
    my_uint32_t result = 0;
    // We store unsigned numbers in signed chars; convert, otherwise the MSB
    // being set would be interpreted as sign and taken over to uint32_t's MSB.
    const unsigned char* src_ = (const unsigned char*)src;

    for (int i = 3; i >= 0; i--) {
        result |= (my_uint32_t)(src_[i]) << (8 * (3 - i));
    }

    return result;
}

