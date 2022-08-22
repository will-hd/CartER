#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

void showNewData();
void getInitAngle();
int getAngle();
int getRawAngle();
void requestData();
void recvWithStartEndMarkers();

// float ReadRawAngle();
// void checkQuadrant();
// void checkMagnetPresence();

#endif