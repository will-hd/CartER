#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <CustomArduino.h>

#include <vector>
#include <PString.h>
#include <Packet.h>
#include <Init.h>

/** Protocol is little-endian.
  */


// Null
class NullPacket : public Packet
{
private:
    static const byte id = 0x00; // NUL

public:
    NullPacket();
    using Packet::construct;

    virtual byte get_id() const override;
};

// Unknown
class UnknownPacket : public Packet
{
private:
    static const byte id = 0x3F; // ?

public:
    UnknownPacket();
    using Packet::construct;

    virtual byte get_id() const override;
};

// Debug
class DebugPacket : public Packet
{
private:
    static const byte id = 0x7E; // ~

public:
    char* _msg;
    size_t _size;


    DebugPacket();
    using Packet::construct;

    virtual byte get_id() const override;
    virtual RawPacket to_raw_packet() override;

    void construct(char *message, size_t size);
};

// Error
class ErrorPacket : public Packet
{
private:
    static const byte id = 0x21; // !

public:
    ErrorPacket();
    using Packet::construct;

    virtual byte get_id() const override;
};

// Ping
class PingPacket : public Packet
{
private:
    static const byte id = 0x70; // p

public:
    unsigned long ping_timestamp;

    PingPacket();
    using Packet::construct;

    virtual byte get_id() const override;

    virtual RawPacket to_raw_packet() override;
    virtual void consume(Stream &sbuf) override;

    // virtual void construct();
    virtual void construct(unsigned long timestamp);
};

// Pong
class PongPacket : public PingPacket
{
private:
    static const byte id = 0x50; // P

public:

    PongPacket();

    virtual byte get_id() const override;
};

#endif
