#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

void get_init_angle();
unsigned short get_tared_angle();
unsigned short get_time();
void serial1Flush();
int get_raw_angle();
void request_data();
void recv_with_blocking();

#endif