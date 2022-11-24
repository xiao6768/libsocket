
#include "frame_stream.h"
#include "frame.h"
#include <memory.h>
#include <stdio.h>

static IoFuncTable funcTable;
static char prefix_buffer[FRAMING_PREFIX_LENGTH];
static char RECV_BUF[RECV_BUF_SIZE];
int receive_bytes(int n) ;
int receive_header(void);
void set_funct(IoFuncTable fc)
{
    funcTable = fc;
}

int frame_sndmsg(const void* buf, int len)
{
    encode_uint32((len), prefix_buffer);
    int result = funcTable.snd(prefix_buffer, FRAMING_PREFIX_LENGTH, 0);

    if (result < 0) return result;

    result = funcTable.snd(buf, len, 0);

    if (result < 0) return result;

    return result;   
}


int frame_snd_w_topic(const void* topic_buf, int topic_len,
    const void* payload_buf, int payload_len)
{
    frame_sndmsg(topic_buf, topic_len);
    frame_sndmsg(payload_buf, payload_len);

    return 0;
}



int receive_header(void)
{
    int pos = 0;

    do {
        int result =
            funcTable.rcv(prefix_buffer + pos, FRAMING_PREFIX_LENGTH, 0);

        if (result < 0)
            return -100;

        pos += result;
    } while (pos < 4);

    return decode_uint32(prefix_buffer);   
}

int receive_bytes(int n) {
    if (n == 0) return 0;

    // Ignore rest of message.
    int rest_len = n > RECV_BUF_SIZE ? RECV_BUF_SIZE : n;
    int pos = 0;

    while (rest_len > 0) {
        int recvd = funcTable.rcv(RECV_BUF + pos, rest_len, 0);

        if (recvd <= 0) return n - rest_len;

        rest_len -= recvd;
        pos += recvd;
    }

    return pos;
}

int frame_rcvmsg(void* dst, int len)
{
    int expected = receive_header();

    int to_receive = len < expected ? len : expected;
    int received = 0;

    while (received < to_receive) {
        int result = receive_bytes(to_receive - received);

        if (result < 0)
            return -200;

        memcpy(dst, RECV_BUF, result);
        dst = (void*)((char*)(dst) + result);
        received += result;
    }

    // Consume remaining frame that doesn't fit into dst.
    int rest = expected - to_receive;
    while (rest > 0) {
        rest -= receive_bytes(rest);
    }
    return received;
}