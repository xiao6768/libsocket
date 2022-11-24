#include <string.h>
#include <unistd.h>
#include <string>

/*
   The committers of the libsocket project, all rights reserved
   (c) 2016, dermesser <lbo@spheniscida.de>

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
   2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS “AS IS” AND ANY
   EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/**
 * @file dgramoverstream.cpp
 * @brief Simple framing over streams.
 * @addtogroup libsocketplusplus
 * @{
 */

#include <dgramoverstream_holometer_v3.hpp>
#include <exception.hpp>

namespace libsocket_holometer_v3 {
using namespace libsocket;


void dgram_over_stream_holometer_v3::encode_uint32_holometer_v3(uint32_t n, char* dst, 
        EProcessType  process_type, EPayloadType payload_type) 
{
    if(process_type >= PROCESS_ENUM_MAX || payload_type >= PAYLOAD_TYPE_ENUM_MAX)
    {
         throw socket_exception(
                __FILE__, __LINE__,
                "dgram_over_stream_holometer_v3::encode_uint32_holometer_v3():process_type >= PROCESS_ENUM_MAX || payload_type >= PAYLOAD_TYPE_ENUM_MAX!",
                false);
    }
    n &= VALID_SIZE_MASK; // only low 16bit is avalible
    n |= ((process_type) & 0x1) << PROCESS_TYPE_BIT;
    n |= ((payload_type) & 0x1) << PAYLOAD_TYPE_BIT;
    for (int i = 3; i >= 0; i--) {
        dst[i] = n >> (8 * (3 - i));
    }
}

uint32_t dgram_over_stream_holometer_v3::decode_uint32_holometer_v3(const char* src, HeaderParameter& header_para) 
{
    uint32_t result = 0;
    // We store unsigned numbers in signed chars; convert, otherwise the MSB
    // being set would be interpreted as sign and taken over to uint32_t's MSB.
    const unsigned char* src_ = (const unsigned char*)src;

    for (int i = 3; i >= 0; i--) {
        result |= uint32_t(src_[i]) << (8 * (3 - i));
    }

    header_para.process_type = (EProcessType)((result >> PROCESS_TYPE_BIT) & 0x1);
    header_para.payload_type = (EPayloadType)((result >> PAYLOAD_TYPE_BIT) & 0x1);

    result &= VALID_SIZE_MASK;

    return result;
}



dgram_over_stream_holometer_v3::dgram_over_stream_holometer_v3(stream_client_socket socket)
    : inner(std::unique_ptr<stream_client_socket>(
          new stream_client_socket(std::move(socket)))) {
    enable_nagle(false);
}

dgram_over_stream_holometer_v3::dgram_over_stream_holometer_v3(
    std::unique_ptr<stream_client_socket> inner_)
    : inner(std::move(inner_)) {
    enable_nagle(false);
}

/**
 * @brief Set TCP_NODELAY to `!enabled` on the underlying socket.
 *
 * TCP_NODELAY causes writes to the socket to be pushed to the network
 * immediately. This emulates the behavior of datagram sockets, and is very
 * useful for datagram-like use of streams, like this class implements. However,
 * it creates slight overhead as data are not batched.
 *
 * (clarification: If Nagle's algorithm is *enabled*, that means that
 * `TCP_NODELAY` is *disabled*, and vice versa)
 */
void dgram_over_stream_holometer_v3::enable_nagle(bool enabled) const {
    int enabled_ = int(!enabled);
    inner->set_sock_opt(IPPROTO_TCP, TCP_NODELAY, (const char*)&enabled_,
                        sizeof(int));
}

ssize_t dgram_over_stream_holometer_v3::sndmsg(const std::string& msg, EProcessType process_type, EPayloadType payload_type) {
    return sndmsg(msg.c_str(), msg.size(), process_type, payload_type);
}

/**
 * @brief Receive a message and place it into dst.
 *
 * No more than dst.size() bytes will be received and placed into dst.
 */
ssize_t dgram_over_stream_holometer_v3::rcvmsg(std::string* dst, HeaderParameter& header_para) {
    uint32_t expected = receive_header(header_para);

    if (expected <= dst->size()) dst->resize(expected);

    size_t to_receive = dst->size();
    size_t received = 0;

    while (received < to_receive) {
        ssize_t result = receive_bytes(to_receive - received);

        if (result < 0)
            throw socket_exception(
                __FILE__, __LINE__,
                "dgram_over_stream_holometer_v3::rcvmsg(): Could not receive message!",
                false);

        dst->replace(received, result, RECV_BUF);

        received += (size_t)result;
    }

    // Consume remaining frame that doesn't fit into dst.
    ssize_t rest = expected - to_receive;
    while (rest > 0) {
        rest -= receive_bytes(rest);
    }

    return received;
}

/**
 * @brief Send the message `msg` as one frame.
 * @returns How many bytes were sent; should be `msg.size()`.
 * @throws socket_exception
 */
ssize_t dgram_over_stream_holometer_v3::sndmsg(const std::vector<uint8_t>& msg, EProcessType process_type, EPayloadType payload_type) {
    return sndmsg(static_cast<const void*>(msg.data()), msg.size(), process_type, payload_type);
}

/**
 * @brief Receive up to `dst.size()` bytes and store them in `dst`.
 * @returns Number of bytes actually received.
 * @throws socket_exception
 *
 * Resize `dst` before calling in order to adjust the number of bytes you will
 * receive.
 */
ssize_t dgram_over_stream_holometer_v3::rcvmsg(std::vector<uint8_t>* dst, HeaderParameter& header_para) {
    uint32_t expected = receive_header(header_para);

    if (expected <= dst->size()) dst->resize(expected);

    size_t to_receive = dst->size();
    size_t received = 0;
    std::vector<uint8_t>::iterator dst_iter = dst->begin();

    while (received < to_receive) {
        ssize_t result = receive_bytes(to_receive - received);

        if (result < 0)
            throw socket_exception(
                __FILE__, __LINE__,
                "dgram_over_stream_holometer_v3::rcvmsg(): Could not receive message!",
                false);

        for (ssize_t i = 0; i < result; i++, dst_iter++)
            *dst_iter = RECV_BUF[i];

        received += result;
    }

    // Consume remaining frame that doesn't fit into dst.
    ssize_t rest = expected - to_receive;
    while (rest > 0) {
        rest -= receive_bytes(rest);
    }

    return received;
}

/**
 * @brief Send the message in buf with length len as one frame.
 * @returns The total number of bytes sent.
 * @throws A socket_exception.
 */
ssize_t dgram_over_stream_holometer_v3::sndmsg(const void* buf, size_t len, EProcessType process_type, EPayloadType payload_type) 
{
    ssize_t result = inner->snd(magic_code_header_, MAGIC_CODE_PREFIX_LENGTH, 0);
    if (result < 0) return result;

    encode_uint32_holometer_v3(uint32_t(len), prefix_buffer, process_type, payload_type);
    result = inner->snd(prefix_buffer, FRAMING_PREFIX_LENGTH, 0);

    if (result < 0) return result;

    result = inner->snd(buf, len, 0);

    if (result < 0) return result;

    return result;
}

/**
 * @brief Receive a message and store the first `len` bytes into `buf`.
 * @returns The number of bytes received.
 * @throws A socket_exception.
 *
 * Bytes in the message beyond `len` are discarded.
 */
ssize_t dgram_over_stream_holometer_v3::rcvmsg(void* dst, size_t len, HeaderParameter& header_para) {
    uint32_t expected = receive_header(header_para);

    size_t to_receive = len < expected ? len : expected;
    size_t received = 0;

    while (received < to_receive) {
        ssize_t result = receive_bytes(to_receive - received);

        if (result < 0)
            throw socket_exception(
                __FILE__, __LINE__,
                "dgram_over_stream_holometer_v3::rcvmsg(): Could not receive message!",
                false);

        memcpy(dst, RECV_BUF, result);
        dst = static_cast<void*>(static_cast<char*>(dst) + result);
        received += result;
    }

    // Consume remaining frame that doesn't fit into dst.
    ssize_t rest = expected - to_receive;
    while (rest > 0) {
        rest -= receive_bytes(rest);
    }
    return received;
}

// Places up to n bytes into this->RECV_BUF.
ssize_t dgram_over_stream_holometer_v3::receive_bytes(size_t n) {
    if (n == 0) return 0;

    // Ignore rest of message.
    ssize_t rest_len = n > RECV_BUF_SIZE ? RECV_BUF_SIZE : n;
    size_t pos = 0;

    while (rest_len > 0) {
        ssize_t recvd = inner->rcv(RECV_BUF + pos, rest_len, 0);

        if (recvd <= 0) return n - rest_len;

        rest_len -= recvd;
        pos += recvd;
    }

    return pos;
}

/**
 * @brief Receive and decode length header.
 * @returns The expected length received.
 * @throws socket_exception
 */
uint32_t dgram_over_stream_holometer_v3::receive_header(HeaderParameter& header_para) {
    ssize_t pos = 0;
    ssize_t magic_code_comp_index = 0;
    char temp_char;

    // loop to find out 4 bytes of magic_code_header_; 
    do {
        ssize_t result =
                    inner->rcv(&temp_char, 1, 0);
         if (result < 0)
            throw socket_exception(__FILE__, __LINE__,
                                   "dgram_over_stream_holometer_v3::receive_header(): Could "
                                   "not receive 1 length prefix of magic_code_header_!",
                                   false);       
        if(temp_char == magic_code_header_[magic_code_comp_index])
        {
            ++magic_code_comp_index;
        } 
        else 
        {
            magic_code_comp_index = 0;
        }
    } while (magic_code_comp_index < MAGIC_CODE_PREFIX_LENGTH);

    do {
        ssize_t result =
            inner->rcv(prefix_buffer + pos, FRAMING_PREFIX_LENGTH, 0);

        if (result < 0)
            throw socket_exception(__FILE__, __LINE__,
                                   "dgram_over_stream_holometer_v3::receive_header(): Could "
                                   "not receive length prefix!",
                                   false);

        pos += result;
    } while (pos < FRAMING_PREFIX_LENGTH);

    return decode_uint32_holometer_v3(prefix_buffer, header_para);
}
}  // namespace libsocket

/**
 * @}
 */
