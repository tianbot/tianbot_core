#ifndef __UDP_H__
#define __UDP_H__

#include "comm_if.h"
#include <pthread.h>
#include <arpa/inet.h>
#include <string>
using namespace std;

struct udp_cfg {
    int udp_recv_port;
    int udp_send_port;
    string client_addr;
};

class Udp : public CommInterface {
public:
    bool open(void *cfg, recv_cb cb) override;
    int send(uint8_t *data, int len) override;
    void close(void) override;
    ~Udp(void);

private:
    pthread_t recv_thread_;
    static void *udp_recv(void *p);
    int running_;
    recv_cb recv_cb_;
    int sockfd_;
    struct sockaddr_in client_addr;
};

#endif