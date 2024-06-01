#ifndef __COMM_IF__
#define __COMM_IF__

#include <functional>
#include <cstdint>

typedef std::function<void(uint8_t *data, unsigned int data_len)> recv_cb;

class CommInterface {
public:
    virtual bool open(void *cfg, recv_cb cb) = 0;
    virtual int send(uint8_t *data, int len) = 0;
    virtual void close(void) = 0;
    virtual ~CommInterface() {}
};

#endif
