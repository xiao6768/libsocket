#ifndef __FRAME_STREAM_H__
#define __FRAME_STREAM_H__

typedef struct {
   int (*snd)(const void* buf, int len, int flags);
   int (*rcv)(void* buf, int len, int flags);
   int (*set_sock_opt)(int level, int optname, const char* optval,int optlen); 
} IoFuncTable;

void set_funct(IoFuncTable fc);

int frame_rcvmsg(void* dst, int len);
int frame_sndmsg(const void* buf, int len);
int frame_snd_w_topic(const void* topic_buf, int topic_len,
    const void* payload_buf, int payload_len);

#endif // __FRAME_STREAM_H__