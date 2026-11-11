// BSD 3-Clause License
//
// Copyright (c) 2019, TIANBOT
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "comm_if.h"
#include "stdint.h"
#include <pthread.h>

struct serial_cfg {
    char *device;
    int rate;
    int flow_ctrl;
    int databits;
    int stopbits;
    int parity;
};

class Serial : public CommInterface
{
public:
    bool open(void *cfg, recv_cb cb) override;
    int send(uint8_t *data, int len) override;
    void close(void) override;
    ~Serial(void);

private:
    bool config(int speed, int flow_ctrl, int databits, int stopbits,
                int parity);
    pthread_t recv_thread_;
    static void *serial_recv(void *p);
    int running_;
    recv_cb recv_cb_;
    int fd_;

    // ---- 通讯录音（可选旁听，默认关闭）----
    // 环境变量 TIANBOT_TAP_DIR 指向一个目录时启用；不设则两路 fd 均为 -1，
    // 每次收发只多一次判空。只写普通文件，不碰串口，收发路径一个字节都不变。
    // 两个方向各自一个 fd、各自只被一个线程写，故不需要加锁。
    int tap_to_fd_;    // 上位机 → 下位机，在 send() 里写
    int tap_from_fd_;  // 下位机 → 上位机，在 serial_recv() 里写
    long tap_written_to_;
    long tap_written_from_;
    long tap_limit_;   // 每个方向的字节上限，到顶后停止录音（继续转发）
};

#endif
