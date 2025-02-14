#ifndef INIT_H
#define INIT_H

#include <CustomArduino.h>
#include <PacketReactor.h>
#include <PacketSender.h>
#include <Mode.h>

// #define DEBUG
// #define SERIALUSB

// Definitions
#ifdef SERIALUSB
#define S SerialUSB
#else
#define S Serial3
#endif

void initialise();

const Mode INITIAL_MODE = COMMAND_AND_CONTROL;

const size_t INITIAL_OUTPUT_STOP_MARKER_LENGTH = 29;
const byte INITIAL_OUTPUT_STOP_MARKER[29] = {0x7E, 0x16, 0x00, 0x00, 0x00, 0x45, 0x4E, 0x44, 0x20, 0x4F, 0x46, 0x20, 0x49, 0x4E, 0x49, 0x54, 0x49, 0x41, 0x4C, 0x49, 0x53, 0x41, 0x54, 0x49, 0x4F, 0x4E, 0x0A, 0x0D, 0x0A};

const unsigned int STRING_BUF_SIZE = 100;

extern bool is_observing;
extern bool limit_finding_has_been_done;
extern bool experiment_done;

extern float_t id_tracker;
extern uint32_t Actions_Done_Counter; //global

// Networking
const unsigned int BAUDRATE = 115200;
// Can't get higher rates to work on Due due to a bug in the firmware :(

#endif