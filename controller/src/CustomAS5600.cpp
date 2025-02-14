// #include <string>

// #include <CustomAS5600.h>
// #include <DebugUtils.h>

// #include <RotaryEncoder.h>
// #include <I2CMultiplexer.h>

// CustomAMS_5600::CustomAMS_5600() {}

// /**
//  * Converts a bit value to a float value in degrees.
//  * @param rawAngle
//  * @return
//  */
// float RealCustomAMS_5600::_rawAngleToDeg(unsigned int rawAngle)
// {
//     // 12 bit => 360/2^12 => 0.087 of a degree
//     float angleDeg = rawAngle * _BIT_DEG_RESOLUTION;

//     return angleDeg;
// }

// /**
//  * Selects the rotary encoder using the I2C interface
//  */
// void RealCustomAMS_5600::_select()
// {
//     I2C_select(_addr);
// }

// /**
//  * Unselects the rotary encoder using the I2C interface
//  */
// void RealCustomAMS_5600::_unselect()
// {
//     I2C_select(_UNSELECT_ADDR);
// }

// /**
//  * Starts the rotary encoder.
//  * May block indefinitely if I2C not connected
//  * @param addr
//  */
// void RealCustomAMS_5600::start(uint8_t addr)
// {
//     _addr = addr;

//     _select();

//     while (1)
//     {
//         if (detectMagnet() == 1)
//         {
//             packet_sender.send_debug("Current magnitude: " + std::to_string(getMagnitude()));
//             break;
//         }
//         else
//         {
//             packet_sender.send_info("Cannot detect magnet");
//         }
//         reset_I2C_multiplexer();
//     }

//     packet_sender.send_debug("Rotary Encoder " + std::to_string(_addr) + "| Magnitude: " + std::to_string(getMagnitude()));
//     packet_sender.send_debug("Rotary Encoder " + std::to_string(_addr) + "| Strength (1=too weak, 2=good, 3=too strong): " + std::to_string(getMagnetStrength()));
//     packet_sender.send_debug("Rotary Encoder " + std::to_string(_addr) + "| Detected: " + std::to_string(detectMagnet()));

//     _unselect();
// }

// /**
//  * Returns the angle degree as a float
//  * @return
//  */
// float RealCustomAMS_5600::readAngleDeg()
// {
//     _select();
//     float angleDeg = _rawAngleToDeg(get_raw_angle());
//     _unselect();

//     return angleDeg;
// }

// void FakeCustomAMS_5600::start(uint8_t addr)
// {
// }

// /**
//  * Returns the angle degree as a float
//  * @return
//  */
// float FakeCustomAMS_5600::readAngleDeg()
// {
//     return PI;
// }
