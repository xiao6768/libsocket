#include "frame_stream.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "../headers/libinetsocket.h"


char* ip = "192.168.3.8";
char* port = "10086";
int sfd = 0;

// typedef struct {
//    int (*snd)(const void* buf, int len, int flags);
//    int (*rcv)(void* buf, int len, int flags);
//    int (*set_sock_opt)(int level, int optname, const char* optval,int optlen); 
// } IoFuncTable;

IoFuncTable funcs;



int snd(const void* buf, int len, int flags)
{
    printf("call snd with len = %d\r\n", len);
    return write(sfd, buf, len);
}

int rcv(void* buf, int len, int flags)
{
    return read(sfd, buf, len);
}

char* buf = "abcde";
char* topic = "hello_world_topic";
char my_buff[100];

int main(int argc, char** arv)
{
    int ret = sfd = create_inet_stream_socket(ip, port, LIBSOCKET_IPv4, 0);
    if (ret < 0) {
        perror(0);
        exit(1);
    }

    funcs.snd = snd;
    funcs.rcv = rcv;

    set_funct(funcs);
    // frame_sndmsg(buf, strlen(buf));
    // frame_snd_w_topic(topic, strlen(topic), buf, strlen(buf) );
    int recv_size = frame_rcvmsg(my_buff, 90);
    printf("recv %d\r\n", recv_size);

    recv_size = frame_rcvmsg(my_buff, 90);
    printf("recv2 %d\r\n", recv_size);


    ret = destroy_inet_socket(sfd);

    if (ret < 0) {
        perror(0);
        exit(1);
    }


    return 0;
}