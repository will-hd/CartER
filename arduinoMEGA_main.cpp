#include <Arduino.h>
#include <RotEncoder.h>
#include <Wire.h>

//Sender Code
char msg;

char arr[6];
unsigned long newTime;
unsigned long oldTime = 0;
float dt = 1;

float angularSpeed;
float newAngle;
float oldAngle = 0;
float raw;
float absRaw;
float absdTheta;
float direction;

// integer values - easier to send + accurate enough (?)
int intAngle;
int intSpeed;

void sendData(){
    intAngle= (int) newAngle;
    intSpeed = (int) angularSpeed;
    Serial.println(intSpeed);
    // Serial.println(intSpeed);
    // Serial.println("-----");

 
    int16_t mask   = B11111111;          // 0000 0000 1111 1111
    uint8_t highByte1   = intAngle >> 8;   // >>>> >>>> 0001 0110
    uint8_t lowByte1 = intAngle & mask; // ____ ____ 0100 0111

    // Signed int!
    int8_t highByte2   = intSpeed >> 8;   // >>>> >>>> 0001 0110
    int8_t lowByte2 = intSpeed & mask; // ____ ____ 0100 0111

    arr[0] = '<';
    arr[1] = highByte1;
    arr[2] = lowByte1;
    arr[3] = highByte2;
    arr[4] = lowByte2;
    arr[5] = '>';

    Serial1.write(arr, 6);


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

void setup() {
    Serial1.begin(115200);
    Serial.begin(9600);
    Wire.begin(); //start i2C  
    Wire.setClock(800000L); //fast clock

    // checkMagnetPresence(); //check the magnet (blocks until magnet is found)
}

void loop(){ 
    newTime = micros();  // get time of current angle measurement. !Resets after 70 mins so will mess up dt!!
    newAngle = ReadRawAngle();
    dt = ((float) newTime - (float) oldTime)*0.000001; //* 0.000001
    raw = newAngle - oldAngle;
    // Serial.println(newTime);

    if (raw == 0){
        angularSpeed = 0;
    }
    else {
        absRaw = fabs(raw); // only calculate once
        
        // modified from https://stackoverflow.com/a/57315426 <- incorrect sign during angle wrap around
        // assumes smaller angle is correct, first term is "modular" arithmetic in C++ (rather than remainder)
        // see https://stackoverflow.com/a/44197900
        absdTheta = min(fmod(360+fmod((-absRaw),360),360), fmod(absRaw, 360));  

        // hacky way of making sure direction is preserved at 360 -> 0 wrap around
        // assume that if the angle jump is greater than 200, we have passed through zero...
        if (absRaw > 200)
        direction = -1*(absRaw / raw); // since both floats, need to check not equal to zero to prevent nan/inf
        else
        direction = (absRaw / raw);

        // update temp values
        oldTime = newTime;
        oldAngle = newAngle;


        // calculate speed
        angularSpeed = direction * absdTheta / dt;
    }
    
    // if (angularSpeed == 0) {
    //     Serial.println("#");
    //     Serial.println(raw, 5);
    // }
    // Serial.println(dt, 3);


    // check for request for data from DUE
    if (Serial1.available())
    msg = Serial1.read();
    else
    msg = 0x00;

    if (msg == 0x3F)
    sendData();
}

// Function
/*
function ang_velocity:
    raw = angle_new - angle_old
    abs_dtheta = math.min((-math.abs(raw))%360,math.abs(raw)%360)
    direction = math.abs(raw)/raw
    return direction*turn*ipersec
end

*/
