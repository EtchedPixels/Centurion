#pragma once

#include <stdint.h>

void tty_init(void);
void net_init(unsigned short port);

void throttle_emulation(uint64_t expected_time_ns);
void throttle_init();
void throttle_set_speed(float speed);