#include <Arduino.h>
#include <RotEncoder.h>
#include <Wire.h>


//12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
//360/4096 = 0.087890625
//Multiply the output of the encoder with 0.087890625

//Sender Code
char msg;

char arr[6];
unsigned long newTime = 0;
unsigned long lastSentTime = 0;
unsigned long DT = 0;
unsigned int dt = 0;

float angularSpeed;
int newAngle;
int oldAngle = 0;
int raw;
int absRaw;
int absdTheta;
int direction;

// integer values - easier to send + accurate enough (?)
int intAngle;
int intSpeed;

void response(){
    Serial1.write(0x78);
}

void sendData(){
    // intAngle= (int) 1234;
    // intSpeed = (int) angularSpeed;
    // Serial.println(angularSpeed);
    // Serial.println(newAngle);
    // Serial.println(intSpeed);
    // Serial.println("-----");
    // Serial.println(newAngle);
    // Serial.println(dt);
 
    int16_t mask   = B11111111;          // 0000 0000 1111 1111
    uint8_t highByte1   = newAngle >> 8;   // >>>> >>>> 0001 0110
    uint8_t lowByte1 = newAngle & mask; // ____ ____ 0100 0111

    // 64 bit int
    uint8_t highByte2   = dt >> 8;   // >>>> >>>> 0001 0110
    uint8_t lowByte2 = dt & mask; // ____ ____ 0100 0111

    arr[0] = '<';
    arr[1] = highByte1;
    arr[2] = lowByte1;
    arr[3] = highByte2;
    arr[4] = lowByte2;
    arr[5] = '>';

    Serial1.write((byte *)&arr, 6);


    /* Didn't work!! think unsigned/signed ints causing issue*/
    // arr[0] = '<';
    
    // arr[1] = (uint8_t) intAngle >> 8;   // >>>> >>>> 0001 0110
    // arr[2] = (uint8_t) intAngle & mask; // ____ ____ 0100 0111
    // // Signed int!
    // arr[3] = (int8_t) intSpeed >> 8;   // >>>> >>>> 0001 0110
    // arr[4] = (int8_t) intSpeed & mask; // ____ ____ 0100 0111

    // arr[5] = '>';

    // Serial1.write('<');
    // Serial1.write(highByte1);
    // Serial1.write(lowByte1);
    // Serial1.write(highByte2);
    // Serial1.write(lowByte2);
    // Serial1.write('>');
}

void serial1Flush(){
    // packet_sender.send_debug("Flush"); 
    while(Serial1.available() > 0) {
    char t = Serial1.read();
    }
}

void setup() {
    Serial1.begin(115200);
    Serial.begin(9600);
    Wire.begin(); //start i2C  
    Wire.setClock(800000L); //fast clock

    // serial1Flush();
    // checkMagnetPresence(); //check the magnet (blocks until magnet is found)
}

void loop(){
    // check for request for data from DUE
    if (Serial1.available()){
    msg = Serial1.read();
    serial1Flush();
    // Serial.println(Serial1.available());
    }
    else
    msg = 0x00;

    // If data requested
    if (msg == 0x3F){
        newTime = micros();  // get time of current angle measurement. !Resets after 70 mins so will mess up dt!!
        // Serial.println(newTime);
        newAngle = ReadRawAngle();
        DT = newTime - lastSentTime;
        if (DT > 65535){ // check small enough to be uint16_t
            dt = 0;
        }
        else{
            dt = (unsigned int) (newTime - lastSentTime);
            // dt = 56789;
        }
        sendData();
        lastSentTime = newTime;
    }
}

