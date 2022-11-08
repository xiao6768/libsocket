#ifndef __FRAME_C_H__
#define __FRAME_C_H__

typedef unsigned int my_uint32_t;
// enum {
//     FRAMING_PREFIX_LENGTH = 4
// };

#define FRAMING_PREFIX_LENGTH 4
#define RECV_BUF_SIZE 256

void encode_uint32(my_uint32_t n, char* dst);
my_uint32_t decode_uint32(const char* src);

#endif // __FRAME_C_H__

