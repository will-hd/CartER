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

// void setup() {
//     Serial.begin(9600);
//     Serial1.begin(115200);
//     Serial.println("<Arduino is ready>");
//     getInitAngle();
// }

// void loop() {
//     requestData();
//     delay(1000);
//     recvWithStartEndMarkers();
//     showNewData();
// }














// char str[3];

// void requestAngle(){
//   Serial1.write(0x3F); // '?'
// }

// float GetAngle(){

//     requestAngle();

//     int i=0;
//     if (Serial1.available()){
//         packet_sender.send_debug("available");
//         while(Serial1.available() && i<4) {
//             str[i++] = Serial1.read();
//         }
    
//     // packet_sender.send_debug(str[1]);
//     packet_sender.send_debug(std::to_string(atoi(str)));
//     // packet_sender.send_debug(std::to_string(sizeof(atoi(str))));
//     }
//     packet_sender.send_debug(">>>>>>>>>>>>");
// }

// // // Example 3 - Receive with start- and end-markers

// const byte numChars = 4;
// char receivedChars[numChars];

// boolean newData = false;

// void recvWithStartEndMarkers() {
//     static boolean recvInProgress = false;
//     static byte ndx = 0;
//     char startMarker = '<';
//     char endMarker = '>';
//     char rc;
 
//     while (Serial.available() > 0 && newData == false) {
//         packet_sender.send_debug("avv");
//         rc = Serial.read();

//         if (recvInProgress == true) {
//             if (rc != endMarker) {
//                 receivedChars[ndx] = rc;
//                 ndx++;
//                 if (ndx >= numChars) {
//                     ndx = numChars - 1;
//                 }
//             }
//             else {
//                 receivedChars[ndx] = '\0'; // terminate the string
//                 recvInProgress = false;
//                 ndx = 0;
//                 newData = true;
//             }
//         }

//         else if (rc == startMarker) {
//             recvInProgress = true;
//         }
//     }
// }























// //I2C pins:
// //STM32: SDA: PB7 SCL: PB6
// //Arduino: SDA: A4 SCL: A5

// //---------------------------------------------------------------------------
// //Magnetic sensor things
// int magnetStatus = 0; //value of the status register (MD, ML, MH)

// int lowbyte; //raw angle 7:0
// word highbyte; //raw angle 7:0 and 11:8
// int rawAngle; //final raw angle 
// float degAngle; //raw angle in degrees (360/4096 * [value between 0-4095])

// int quadrantNumber, previousquadrantNumber; //quadrant IDs
// float numberofTurns = 0; //number of turns
// float correctedAngle = 0; //tared angle - based on the startup value
// float startAngle = 0; //starting angle
// float totalAngle = 0; //total absolute angular displacement
// float previoustotalAngle = 0; //for the display printing

// float ReadRawAngle()
// { 
  
//   //7:0 - bits
//   Wire.beginTransmission(0x36); //connect to the sensor
//   Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
//   Wire.endTransmission(); //end transmission
//   Wire.requestFrom(0x36, 1); //request from the sensor
  
//   while(Wire.available() == 0)
//   packet_sender.send_debug("waiting");  
// //   Serial.println("waiting"); //wait until it becomes available 
//   lowbyte = Wire.read(); //Reading the data after the request
 
//   //11:8 - 4 bits
//   Wire.beginTransmission(0x36);
//   Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
//   Wire.endTransmission();
//   Wire.requestFrom(0x36, 1);
  
//   while(Wire.available() == 0)
//   packet_sender.send_debug("waiting");  
//   highbyte = Wire.read();
  
//   //4 bits have to be shifted to its proper place as we want to build a 12-bit number
//   highbyte = highbyte << 8; //shifting to left
//   //What is happening here is the following: The variable is being shifted by 8 bits to the left:
//   //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
//   //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in
  
//   //Finally, we combine (bitwise OR) the two numbers:
//   //High: 00001111|00000000
//   //Low:  00000000|00001111
//   //      -----------------
//   //H|L:  00001111|00001111
//   rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

//   //We need to calculate the angle:
//   //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
//   //360/4096 = 0.087890625
//   //Multiply the output of the encoder with 0.087890625
//   degAngle = rawAngle * 0.087890625;
// //   Serial.println(String(degAngle,DEC));
//   packet_sender.send_debug(std::to_string(degAngle)); 
//   return degAngle;
  
//   //Serial.print("Deg angle: ");
//   //Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle
  
// }

// void correctAngle()
// {
//   //recalculate angle
//   correctedAngle = degAngle - startAngle; //this tares the position

//   if(correctedAngle < 0) //if the calculated angle is negative, we need to "normalize" it
//   {
//   correctedAngle = correctedAngle + 360; //correction for negative numbers (i.e. -15 becomes +345)
//   }
//   else
//   {
//     //do nothing
//   }
//   //Serial.print("Corrected angle: ");
//   //Serial.println(correctedAngle, 2); //print the corrected/tared angle  
// }

// void checkQuadrant()
// {
//   /*
//   //Quadrants:
//   4  |  1
//   ---|---
//   3  |  2
//   */

//   //Quadrant 1
//   if(correctedAngle >= 0 && correctedAngle <=90)
//   {
//     quadrantNumber = 1;
//   }

//   //Quadrant 2
//   if(correctedAngle > 90 && correctedAngle <=180)
//   {
//     quadrantNumber = 2;
//   }

//   //Quadrant 3
//   if(correctedAngle > 180 && correctedAngle <=270)
//   {
//     quadrantNumber = 3;
//   }

//   //Quadrant 4
//   if(correctedAngle > 270 && correctedAngle <360)
//   {
//     quadrantNumber = 4;
//   }
//   //Serial.print("Quadrant: ");
//   //Serial.println(quadrantNumber); //print our position "quadrant-wise"

//   if(quadrantNumber != previousquadrantNumber) //if we changed quadrant
//   {
//     if(quadrantNumber == 1 && previousquadrantNumber == 4)
//     {
//       numberofTurns++; // 4 --> 1 transition: CW rotation
//     }

//     if(quadrantNumber == 4 && previousquadrantNumber == 1)
//     {
//       numberofTurns--; // 1 --> 4 transition: CCW rotation
//     }
//     //this could be done between every quadrants so one can count every 1/4th of transition

//     previousquadrantNumber = quadrantNumber;  //update to the current quadrant
  
//   }  
//   //Serial.print("Turns: ");
//   //Serial.println(numberofTurns,0); //number of turns in absolute terms (can be negative which indicates CCW turns)  

//   //after we have the corrected angle and the turns, we can calculate the total absolute position
//   totalAngle = (numberofTurns*360) + correctedAngle; //number of turns (+/-) plus the actual angle within the 0-360 range
//   //Serial.print("Total angle: ");
//   //Serial.println(totalAngle, 2); //absolute position of the motor expressed in degree angles, 2 digits
// }

// void checkMagnetPresence()
// {  
//   //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly
//   Wire.begin();
//   Wire.setClock(800000L);

//   while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
//   {
//     magnetStatus = 0; //reset reading

//     Wire.beginTransmission(0x36); //connect to the sensor
//     Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
//     Wire.endTransmission(); //end transmission
//     Wire.requestFrom(0x36, 1); //request from the sensor
    
//     while(Wire.available() == 0)
//     packet_sender.send_debug("checking"); //wait until it becomes available 
//     magnetStatus = Wire.read(); //Reading the data after the request



//     //Serial.print("Magnet status: ");
//     //Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
//   }      
  
//   //Status register output: 0 0 MD ML MH 0 0 0  
//   //MH: Too strong magnet - 100111 - DEC: 39 
//   //ML: Too weak magnet - 10111 - DEC: 23     
//   //MD: OK magnet - 110111 - DEC: 55

//   //Serial.println("Magnet found!");
//   //delay(1000);  
// }











// void setup()
// {
//   Serial.begin(9600); //start serial - tip: don't use serial if you don't need it (speed considerations)
//   Wire.begin(); //start i2C  
// 	Wire.setClock(800000L); //fast clock
//   Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>> ");

//   checkMagnetPresence(); //check the magnet (blocks until magnet is found)

//   ReadRawAngle(); //make a reading so the degAngle gets updated
//   startAngle = degAngle; //update startAngle with degAngle - for taring
  
// }

// void loop()
// {    
//     ReadRawAngle(); //ask the value from the sensor
//     correctAngle(); //tare the value
//     checkQuadrant(); //check quadrant, check rotations, check absolute angular position
//     delay(100); //wait a little - adjust it for "better resolution"

// }