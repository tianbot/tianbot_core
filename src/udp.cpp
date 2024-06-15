#include "udp.h"
#include "string.h"
#include <unistd.h>
#include <fcntl.h>

void *Udp::udp_recv(void *p)
{
    uint8_t recvbuff[2048];
    int recvlen = 0;
    fd_set rfds;
    timeval timeout;
    Udp *pThis = (Udp *)p;
    int ready;
    struct sockaddr_in clientAddr;
    socklen_t addrLen = sizeof(clientAddr);
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    usleep(100000);

    memset(&clientAddr, 0, sizeof(clientAddr));

    while (pThis->running_)
    {
        memset(recvbuff, 0, sizeof(recvbuff));
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        FD_ZERO(&rfds);
        FD_SET(pThis->sockfd_, &rfds);
        ready = select(pThis->sockfd_ + 1, &rfds, NULL, NULL, &timeout);
        if (ready < 0)
        {
            perror("select");
            return NULL;
        }
        else if (ready == 0)
        {
            continue;
        }
        else
        {
            recvlen = recvfrom(pThis->sockfd_, recvbuff, sizeof(recvbuff), 0, (struct sockaddr *)&clientAddr, &addrLen);
            if (recvlen <= 0)
            {
                continue;
            }
            pThis->recv_cb_(recvbuff, recvlen);
        }
    }
    return NULL;
}

bool Udp::open(void *cfg, recv_cb cb)
{
    struct sockaddr_in serv_addr;
    struct udp_cfg *pcfg = (struct udp_cfg *)cfg;
    if ((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("Socket creation failed");
        return false;
    }
    memset(&serv_addr, 0, sizeof(serv_addr));
    memset(&client_addr, 0, sizeof(client_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(pcfg->udp_recv_port);
    client_addr.sin_addr.s_addr = inet_addr(pcfg->client_addr.c_str());
    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons(pcfg->udp_send_port);

    if (bind(sockfd_, (const struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        perror("Bind failed");
        ::close(sockfd_);
        return false;
    }

    recv_cb_ = cb;
    running_ = 1;
    pthread_create(&recv_thread_, nullptr, udp_recv, this);
    return true;
}

int Udp::send(uint8_t *data, int len)
{
    if (sendto(sockfd_, data, len, 0, (struct sockaddr *)&client_addr, sizeof(client_addr)) < 0)
    {
        perror("sendto failed");
        ::close(sockfd_);
        return -1;
    }
    return 0;
}

void Udp::close(void)
{
    running_ = 0;
    if (recv_thread_)
        pthread_join(recv_thread_, nullptr);
    ::close(sockfd_);
}

Udp::~Udp(void)
{
    close();
}