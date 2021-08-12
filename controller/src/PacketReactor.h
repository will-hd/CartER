#ifndef PACKET_REACTOR_H
#define PACKET_REACTOR_H

#include <CustomArduino.h>

#include <memory>

#include <Utils.h>

// Packet buffer
constexpr byte PACKET_END = 0x0A; // \n

class PacketReactor
{
protected:
    Stream &_s;

    template <class T>
    std::unique_ptr<T> _read_and_construct_packet()
    {
        // std::make_unique<T> packet;
        std::unique_ptr<T> packet = std::make_unique<T>();

        packet->read(_s);

        packet->construct();

        return packet;
    }

public:
    explicit PacketReactor(Stream &stream);

    void tick();
};

extern PacketReactor packet_reactor;

#endif