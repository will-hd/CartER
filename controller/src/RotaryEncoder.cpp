#include <RotaryEncoder.h>
#include <Wire.h> //This is for i2C
#include <Protocol.h>

// Example 3 - Receive with start- and end-markers

const byte numChars = 3;
char receivedChars[numChars];

boolean newData = false;
boolean requestedData = false;
int angleOffset = 0;

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;

    // Packet markers
    char startMarker = '<';
    char endMarker = '>';
    
    char rc; // Recieved bytes
    while (Serial1.available() == 0);
    // packet_sender.send_debug("Waiting");

    while (Serial1.available() > 0 && newData == false) { //Serial1.available > 0, requestedData == true
        // packet_sender.send_debug("Available");   
        rc = Serial1.read();
        // Serial.println(rc);

        if (recvInProgress == true) {
            // Serial.println("Ava");

            if ((rc != endMarker) & (rc != startMarker)) {
                // packet_sender.send_debug("rc:" +std::to_string(rc));    
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
                requestedData = false;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void requestData(){
    Serial1.write(0x3F);
    requestedData = true;
}
int getRawAngle(){
    if (newData == true) {
        char highByte_show = receivedChars[0];
       char lowByte_show = receivedChars[1];
       uint16_t recombined_int = (highByte_show << 8) | lowByte_show;

    //    int shifted_angle = (360 + angleOffset - recombined_int) % 360;
       return recombined_int;
    }
}

int getAngle(){
    if (newData == true) {
        char highByte_show = receivedChars[0];
        char lowByte_show = receivedChars[1];
        uint16_t recombined_int = (highByte_show << 8) | lowByte_show;

        int shifted_angle = (360 + angleOffset - recombined_int) % 360;

        // packet_sender.send_debug(std::to_string(shifted_angle));
        newData = false;
        return shifted_angle;
    }
    else
    packet_sender.send_debug("else: ");
}


void showNewData() {
    
    if (newData == true) {
        // Serial.println(receivedChars);
        int angle = getAngle();
        // int shifted_angle = (360 + angleOffset - angle) % 360;
        packet_sender.send_debug(std::to_string(angle));
        newData = false;
    }
}

void getInitAngle() {
    requestData();
    delay(100);
    recvWithStartEndMarkers();
    angleOffset = getRawAngle();

    packet_sender.send_debug(std::to_string(angleOffset));
    // Serial.println("Init angle");
    // Serial.print(angleOffset);
}