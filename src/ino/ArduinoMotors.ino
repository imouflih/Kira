/*
   This program is used to control the motors for Kira
   see https://github.com/imouflih/Kira

   It uses an I2C bus to communicate with the RapsberryPi.
   The board send a PWM value between -255 and 255 for each motors.

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

#define START_BYTE  0x25
#define STOP_BYTE   0x5A
#define INIT_COUNTERS 0xA1

// Encoder wheels counters
volatile long countRight = 0;
volatile long countLeft = 0;
volatile long time;

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

void initCounters() {
    time = micros();
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
void orderMove(int8_t speedRight, int8_t speedLeft) { // to do : fix inversion
    orderLeft(computeDirection(speedLeft), abs(speedLeft));
    orderRight(computeDirection(speedRight), abs(speedRight));
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

void sendData() {
    time = micros();

    Wire.write(START_BYTE);

    Wire.write(countLeft);
    Wire.write(countLeft >> 8);
    Wire.write(countLeft >> 16);
    Wire.write(countLeft >> 24);

    Wire.write(countRight);
    Wire.write(countRight >> 8);
    Wire.write(countRight >> 16);
    Wire.write(countRight >> 24);

    Wire.write(time);
    Wire.write(time >> 8);
    Wire.write(time >> 16);
    Wire.write(time >> 24);

    Wire.write(STOP_BYTE);
}

// Function that receive the bytes from the I2C
void recv(int numBytes) {
    if (numBytes == 4) {
        int8_t speed_left = Wire.read() | (Wire.read() << 8);
        int8_t speed_right = Wire.read() | (Wire.read() << 8);

        // Serial.print("speed left : ");
        // Serial.println(speed_left);
        // Serial.print("speed right : ");
        // Serial.println(speed_right);

        orderMove(speed_right, speed_left);
    }
    else if (numBytes == 1) {
        uint8_t data = Wire.read();
        if (data == INIT_COUNTERS) {
            initCounters();
            // Serial.println("Initialisation des compteurs!!");
        }
    }

    Wire.flush();
    while (Wire.available()) Wire.read();
}