#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

void showNewData();
void getInitAngle();
unsigned short getAngle();
unsigned short getTime();
void serial1Flush();
int getRawAngle();
void requestData();
void recvWithStartEndMarkers();
void recvWithBlocking();

// float ReadRawAngle();
// void checkQuadrant();
// void checkMagnetPresence();

#endif