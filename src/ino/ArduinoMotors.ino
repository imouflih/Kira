/*
   This program is used to control the motors for Kira
   see https://github.com/imouflih/Kira

   It uses an I2C bus to communicate with the RapsberryPi.
   The board send a PWM value between -255 and 255 for each motors.
   This code allows only to send a PWM value between -128 and 127

   Last updated : February 26 2022
   Author : Iliasse Mouflih
*/

#include <Wire.h>

// Comment or uncomment to activate
#define DEBUG // Used to print informations to the serial port

#define POWER_ENABLE 11 // pin to enable motor power

// Left motor
#define SPEED_LEFT 6     // PWM pin for controlling speed of left motor
#define DIRECTION_LEFT 7 // pin for controlling direction of left motor

// Right motor
#define SPEED_RIGHT 9     // PWM pin for controlling speed of right motor
#define DIRECTION_RIGHT 8 // pin for controlling direction of right motor

// Right encoder wheel
#define PULSE_RIGHT_ENCODER 3     // pin for right encoder pulse
#define DIRECTION_RIGHT_ENCODER 5 // pin for right encoder direction

// Left encoder wheel
#define PULSE_LEFT_ENCODER 2     // pin for left encoder pulse
#define DIRECTION_LEFT_ENCODER 4 // pin for left encoder direction

// I2C address of the board
#define ADDR_I2C 12

// Encoder wheels
#define LEFT 0
#define RIGHT 1

// Bytes used for communication with the Raspberry Pi
#define START_BYTE  0x25
#define STOP_BYTE   0x5A
#define INIT_COUNTERS 0xA1
#define EMERGENCY_STOP 0xA2

// Encoder wheels counters
volatile long countRight = 0;
volatile long countLeft = 0;

// When set to true, the robot is forced to stop. It is used to halt the robot when the emergency button is pressed or when the match time has ended
bool stopTheRobot = false;

void setup() {

#ifdef DEBUG
    Serial.begin(9600);
#endif

    // Setup the pins to OUTPUT
    pinMode(SPEED_LEFT, OUTPUT);
    pinMode(DIRECTION_LEFT, OUTPUT);

    pinMode(SPEED_RIGHT, OUTPUT);
    pinMode(DIRECTION_RIGHT, OUTPUT);

    pinMode(PULSE_RIGHT_ENCODER, INPUT);
    pinMode(PULSE_LEFT_ENCODER, INPUT);

    pinMode(DIRECTION_RIGHT_ENCODER, INPUT);
    pinMode(DIRECTION_LEFT_ENCODER, INPUT);

    digitalWrite(POWER_ENABLE, HIGH);   // enable motor power

    attachInterrupt(digitalPinToInterrupt(PULSE_RIGHT_ENCODER), countRightEncoder, FALLING);
    attachInterrupt(digitalPinToInterrupt(PULSE_LEFT_ENCODER), countLeftEncoder, FALLING);

    // Make sure the motors are stopped
    stop();

    // Start I2C
    Wire.begin(ADDR_I2C);

    // On request function
    Wire.onRequest(sendData);

    // Add interrupt function
    Wire.onReceive(recv);
}

void loop() {
    // do nothing
}

// Initialize the encoder wheel counters
void initCounters() {
    countLeft = 0;
    countRight = 0;
}

// Function that detects and increament/decreament right wheel direction
void countRightEncoder() {
    int8_t direction = !digitalRead(DIRECTION_RIGHT_ENCODER);
    countRight += direction ? -1 : 1;
}

// Function that detects and increament/decreament left wheel direction
void countLeftEncoder() {
    int8_t direction = digitalRead(DIRECTION_LEFT_ENCODER);
    countLeft += direction ? -1 : 1;
}

// Function that stop the motors
void stop() {
    analogWrite(SPEED_LEFT, LOW);
    digitalWrite(DIRECTION_LEFT, LOW);

    analogWrite(SPEED_RIGHT, LOW);
    digitalWrite(DIRECTION_RIGHT, LOW);
}

// Function that returns the direction depending on the sign of the speed
uint8_t computeDirection(int8_t speed) {
    return (speed < 0) ? 2 : (speed > 0) ? 1 : 0;
}

// Function that send an order to the motors 
// Speed is between -128 and 127
void orderMove(int8_t speedRight, int8_t speedLeft) { // TODO : fix inversion
    if (stopTheRobot) {
        stop();
    }
    else {
        orderLeft(computeDirection(speedLeft), abs(speedLeft));
        orderRight(computeDirection(speedRight), abs(speedRight));
    }
}

// Function that send an order to the left motor
void orderLeft(uint8_t direction, uint8_t speed) {
    switch (direction) {
    case 0:
        // Stopping motors
        digitalWrite(DIRECTION_LEFT, LOW);
        analogWrite(SPEED_LEFT, 0);
        break;
    case 1:
        // Forward
        digitalWrite(DIRECTION_LEFT, HIGH);
        analogWrite(SPEED_LEFT, speed);
        break;
    case 2:
        // Backward
        digitalWrite(DIRECTION_LEFT, LOW);
        analogWrite(SPEED_LEFT, speed);
        break;
    }
}

// Function that send an order to the right motor
void orderRight(uint8_t direction, uint8_t speed) {
    switch (direction) {
    case 0:
        // Stopping motors
        digitalWrite(DIRECTION_RIGHT, LOW);
        analogWrite(SPEED_RIGHT, 0);
        break;
    case 1:
        // Forward
        digitalWrite(DIRECTION_RIGHT, HIGH);
        analogWrite(SPEED_RIGHT, speed);
        break;
    case 2:
        // Backward
        digitalWrite(DIRECTION_RIGHT, LOW);
        analogWrite(SPEED_RIGHT, speed);
        break;
    }
}

// Function that sends data over I2C
void sendData() {
    Wire.write(START_BYTE);

    Wire.write(countLeft);
    Wire.write(countLeft >> 8);
    Wire.write(countLeft >> 16);
    Wire.write(countLeft >> 24);

    Wire.write(countRight);
    Wire.write(countRight >> 8);
    Wire.write(countRight >> 16);
    Wire.write(countRight >> 24);

    Wire.write(STOP_BYTE);
}

// Function that receive the bytes from the I2C
void recv(int numBytes) {
    if (numBytes == 4) {
        int8_t speed_left = Wire.read() | (Wire.read() << 8);
        int8_t speed_right = Wire.read() | (Wire.read() << 8);

        orderMove(speed_right, speed_left);
    }
    else if (numBytes == 1) {
        uint8_t data = Wire.read();
        if (data == INIT_COUNTERS) {
            initCounters();
            // Serial.println("Initialisation des compteurs!!");
            stopTheRobot = false;
        }
        else if (data == EMERGENCY_STOP) {
            stop();
            // Serial.println("Emergency button stop is pressed!!");
            stopTheRobot = true;
        }
    }

    Wire.flush();
    while (Wire.available()) Wire.read();
}