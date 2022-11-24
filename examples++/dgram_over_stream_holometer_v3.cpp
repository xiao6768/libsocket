#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

// #include <libsocket/dgramoverstream.hpp>
// #include <libsocket/exception.hpp>
// #include <libsocket/inetclientstream.hpp>
// #include <libsocket/inetserverstream.hpp>

#include "dgramoverstream_holometer_v3.hpp"
#include <exception.hpp>
#include <inetclientstream.hpp>
#include <inetserverstream.hpp>

/*
 * This example demonstrates the use of the dgram_over_stream class. As you can
 * read in that classes documentation, it simplifies sending discrete packets of
 * data over a stream connection. This is achieved by framing; concretely, the
 * length of a packet is sent first, so the receiver knows how many bytes to
 * expect.
 *
 * Usage:
 *   ./dgram_over_stream -c # client
 *   ./dgram_over_stream -s # server
 */

using std::string;

static const string HOST = "192.168.188.1";
// static const string HOST = "localhost";
static const string PORT = "4445";

void run_client(void);
void run_string_client(void);
void run_vec_client(void);
void run_server(void);

enum MODE {
    MODE_CLIENT,
    MODE_SERVER,
};

MODE get_mode(char** argv) {
    if (string(argv[1]) == "-c") {
        return MODE_CLIENT;
    } else if (string(argv[1]) == "-s") {
        return MODE_SERVER;
    } else {
        perror("Bad command line; please use either -s or -c");
        exit(1);
    }
}

void run_client(void) {
    static const size_t bufsize = 1024;
    char buf[bufsize];
    memset(buf, 0, bufsize);

    libsocket::inet_stream client(HOST, PORT, LIBSOCKET_IPv4);
    libsocket_holometer_v3::dgram_over_stream_holometer_v3 dgram_cl(std::move(client));

    dgram_cl.sndmsg("Hello", 5);
    libsocket_holometer_v3::dgram_over_stream_holometer_v3::HeaderParameter para;
    std::cout << "Client received " << dgram_cl.rcvmsg(buf, bufsize, para)
              << " bytes.\n";
    std::cout << buf << std::endl;

    return;
}

void run_string_client(void) {
    libsocket::inet_stream client(HOST, PORT, LIBSOCKET_IPv4);
    libsocket_holometer_v3::dgram_over_stream_holometer_v3 dgram_cl(std::move(client));

    std::string recvbuf(0, 'a');
    recvbuf.resize(3);

    dgram_cl.sndmsg(std::string("Hello"));
    libsocket_holometer_v3::dgram_over_stream_holometer_v3::HeaderParameter para;
    std::cout << "Client received " << dgram_cl.rcvmsg(&recvbuf, para)
              << " bytes into std::string.\n";
    std::cout << recvbuf << std::endl;

    return;
}

void run_vec_client(void) {
    libsocket::inet_stream client(HOST, PORT, LIBSOCKET_IPv4);
    libsocket_holometer_v3::dgram_over_stream_holometer_v3 dgram_cl(std::move(client));

    std::vector<uint8_t> recvbuf;
    recvbuf.resize(15);

    libsocket_holometer_v3::dgram_over_stream_holometer_v3::HeaderParameter para;
    dgram_cl.sndmsg(std::string("Hello"));
    std::cout << "Client received " << dgram_cl.rcvmsg(&recvbuf, para)
              << " bytes into std::vec.\n";

    std::for_each(recvbuf.begin(), recvbuf.end(),
                  [](uint8_t b) { std::cout << static_cast<char>(b); });
    std::cout << std::endl;

    return;
}

void run_server(void) {
    static const size_t bufsize = 1024;
    char buf[bufsize];
    memset(buf, 0, bufsize);

    libsocket::inet_stream_server srv(HOST, PORT, LIBSOCKET_IPv4);

    while (true) {
        libsocket::inet_stream* client = srv.accept(1);
        libsocket_holometer_v3::dgram_over_stream_holometer_v3 dgram_cl(std::move(*client));
        ssize_t len = 0;

        libsocket_holometer_v3::dgram_over_stream_holometer_v3::HeaderParameter para;
        std::cout << "Server received " << (len = dgram_cl.rcvmsg(buf, bufsize, para))
                  << " bytes.\n";
        std::cout << buf << std::endl;
        std::string buf_str(buf);
        std::string bounce_back_str = buf_str + " back";
        dgram_cl.sndmsg(bounce_back_str.c_str(), bounce_back_str.size());

        memset(buf, 0, len);
    }

    return;
}



int main(int argc, char** argv) {
    
    libsocket::inet_stream client(HOST, PORT, LIBSOCKET_IPv4);
    libsocket_holometer_v3::dgram_over_stream_holometer_v3 dgram_cl(std::move(client));
    const char* topic = "Hello_topic";
    const char* payload = "Hi,Iampayload^_^";
    dgram_cl.sndmsg(topic, strlen(topic), libsocket_holometer_v3::dgram_over_stream_holometer_v3::EProcessType::SOC,
        libsocket_holometer_v3::dgram_over_stream_holometer_v3::EPayloadType::TOPIC);
    dgram_cl.sndmsg(payload, strlen(payload), libsocket_holometer_v3::dgram_over_stream_holometer_v3::EProcessType::SOC,
        libsocket_holometer_v3::dgram_over_stream_holometer_v3::EPayloadType::SERIAL_DATA);       
    // if(argc < 2)
    // {
    //     printf("Usage: \r\nApp -s or App -c\r\n");
    //     exit(-1);
    // }

    // MODE mode = get_mode(argv);

    // try {
    //     if (mode == MODE_CLIENT) {
    //         run_client();
    //         run_string_client();
    //         run_vec_client();
    //     } else if (mode == MODE_SERVER) {
    //         run_server();
    //     }
    // } catch (libsocket::socket_exception e) {
    //     std::cerr << e.mesg << std::endl;
    // }

    return 0;
}

