#include <RotaryEncoder.h>
#include <Wire.h> //This is for i2C
#include <Protocol.h>

// Example 3 - Receive with start- and end-markers

const byte numChars = 5;
char receivedChars[numChars];

boolean newData = false;
boolean requestedData = false;
int angleOffset = 0;

void serial1Flush(){
    // packet_sender.send_debug("Flush"); 
    while(Serial1.available() > 0) {
    char t = Serial1.read();
    }
}

void recvWithStartEndMarkers() {

    // Need a more robust way of getting velocity - so that reading is most current (i.e. empty buffer after each reading?)
    static boolean recvInProgress = false;
    static byte ndx = 0;

    // Packet markers
    char startMarker = '<';
    char endMarker = '>';
    
    char rc; // Recieved bytes
    while (Serial1.available() == 0);
    // packet_sender.send_debug("Waiting");

    while (Serial1.available() > 0 && newData == false) { //Serial1.available > 0, requestedData == true
        // packet_sender.send_debug('"'+std::to_string(Serial1.available()));   
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
    packet_sender.send_debug("req"); 
}

int getRawAngle(){

    char highByte_show = receivedChars[0];
    char lowByte_show = receivedChars[1];
    uint16_t recombined_int = (highByte_show << 8) | lowByte_show;
    
    newData == false;
//    int shifted_angle = (360 + angleOffset - recombined_int) % 360;
    return recombined_int;
}

// Don't think we need check of if (newData == true)
int getAngle(){
    if (newData == true){
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

int getSpeed(){
    if (newData == true){ 
    char highByte_show = receivedChars[2];
    char lowByte_show = receivedChars[3];

    int16_t recombined_int = (highByte_show << 8) | lowByte_show;
    packet_sender.send_debug(std::to_string(recombined_int));
    newData = false;
    
    return recombined_int;
    }
    else
    packet_sender.send_debug("else: ");
}

// std::array<int, 2> getData(){
//     if (newData == true){
//         char highByte1_show = receivedChars[0];
//         char lowByte1_show = receivedChars[1];
//         char highByte2_show = receivedChars[2];
//         char lowByte2_show = receivedChars[3];

//         int16_t recombined_angle = (highByte1_show << 8) | lowByte1_show;
//         int16_t recombined_speed = (highByte2_show << 8) | lowByte2_show;

//         int shifted_angle = (360 + angleOffset - recombined_int) % 360;
//         newData = false;
//         return {shifted_angle, recombined_speed};
// }

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
    // Request
    requestData();
    delay(100);
    
    // Receive
    recvWithStartEndMarkers();
    angleOffset = getRawAngle();

    packet_sender.send_debug(std::to_string(angleOffset));
}