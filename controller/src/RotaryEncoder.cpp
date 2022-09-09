#include <RotaryEncoder.h>
#include <Wire.h> //This is for i2C
#include <Protocol.h>

// Inspired by https://forum.arduino.cc/t/serial-input-basics-updated/382007

const byte numChars = 5;
char receivedChars[numChars];

boolean requestedData = false;
unsigned short angleOffset = 0;


/**
 * Flush the serial connected to the MEGA. 
 */
void serial1Flush(){
    // packet_sender.send_debug("Flush"); 
    while(Serial1.available() > 0) {
    char t = Serial1.read();
    }
}

/**
 * Recieve data from Arduino MEGA while blocking until all data recieved.
 *
 * Start, end markers are used though probably unnecessary.
 * Ideally, I2C for rotary encoder would be from Arduino DUE and MEGA 
 * would not be required....
 */
void recv_with_blocking(){


    // Packet markers
    char startMarker = '<';
    char endMarker = '>';
    
    char rc; // Recieved bytes
    while (requestedData == true) {
        rc = Serial1.read();

        if (rc == startMarker){
            // Serial.println("st");
            Serial1.readBytes(receivedChars, 4);
        }
        else if (rc == endMarker){
            requestedData = false;
            // Serial.println("DOne");
        }
    }
}

/**
 * Send "request data" byte to arduino MEGA.
 */
void request_data(){
    Serial1.write(0x3F);
    requestedData = true;
    // packet_sender.send_debug("req"); 
}


/**
 * Read the raw angle (0-4095) sent from MEGA.
 */
int get_raw_angle(){
    char highByte_show = receivedChars[0];
    char lowByte_show = receivedChars[1];
    unsigned short recombined_int = (highByte_show << 8) | lowByte_show;

    return recombined_int;
}

/**
 * Get angle mod offset angle (due to veritcal not being zero on encoder).
 * Aligning the magnets to zero would be impractical.
 */
unsigned short get_tared_angle(){
    char highByte_show = receivedChars[0];
    char lowByte_show = receivedChars[1];
    unsigned short recombined_int = (highByte_show << 8) | lowByte_show;
    unsigned short shifted_angle = (4096 + angleOffset - recombined_int) % 4096;

    return shifted_angle;
}

/**
 * Read time of angle measurements.
 * Used for estimating angular velocity.
 */
unsigned short get_time() {
    char highByte_show = receivedChars[2];
    char lowByte_show = receivedChars[3];
    unsigned short recombined_int = (highByte_show << 8) | lowByte_show;
    // Serial.println(recombined_int);
    return recombined_int;
}

void get_init_angle() {
    // Request
    request_data();
        
    // Receive
    recv_with_blocking();
    // angleOffset = get_raw_angle();
    // The angle given by rot enc when vertically down.
    // Assumes that the magnet and rotary encoder will not shift relative to each other
    angleOffset = 3598; 

    packet_sender.send_debug("INIT Angle (hardcoded - not detected): "+std::to_string(angleOffset));
}