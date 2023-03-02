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
// #define TEST  // Used to activate the motors on startup.

#define PIN_POWER_ENABLE    11

// Left motor
#define SPEED_LEFT       6
#define DIRECTION_LEFT   7

// Right motor
#define SPEED_RIGHT       9
#define DIRECTION_RIGHT   8

// Right encoder wheel
#define PULSE_RIGHT_ENCODER       3
#define DIRECTION_RIGHT_ENCODER   5

// Left encoder wheel
#define PULSE_LEFT_ENCODER        2
#define DIRECTION_LEFT_ENCODER    4

// I2C address
#define ADDR_I2C   12

// Odometry config
#define ENTRAXE     235 
#define DIAMETER     35
#define TICK_PER_TURN  512
#define TICK_SPEED     1000  // Values : 500, 1000, 2000, 4000, 8000

// Counters
volatile long countRight = 0;
volatile long countLeft = 0;

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

  digitalWrite(PIN_POWER_ENABLE, HIGH);

  attachInterrupt(digitalPinToInterrupt(PULSE_RIGHT_ENCODER), countRightEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(PULSE_LEFT_ENCODER), countLeftEncoder, FALLING);

  // Make sure the motors are stopped
  stop();

  // Start I2C
  Wire.begin(ADDR_I2C);

  // Add interrupt function
  Wire.onReceive(recv);
}

void loop() {

  int directionRight = !digitalRead(DIRECTION_RIGHT_ENCODER);
  int directionLeft = digitalRead(DIRECTION_LEFT_ENCODER);

  //clear screen
  for (int i = 0; i < 50; i++) {
    Serial.println();
  }

  Serial.print("Right encoder count: ");
  Serial.println(countRight);
  Serial.print("Left encoder count: ");
  Serial.println(countLeft);

  Serial.print("Right encoder direction: ");
  Serial.println(directionRight);
  Serial.print("Left encoder direction: ");
  Serial.println(directionLeft);

  delay(1000);
}

// Function that detects and increament/decreament right wheel direction
void countRightEncoder() {
  int direction = !digitalRead(DIRECTION_RIGHT_ENCODER);
  countRight += direction ? -1 : 1;
}

// Function that detects and increament/decreament left wheel direction
void countLeftEncoder() {
  int direction = digitalRead(DIRECTION_LEFT_ENCODER);
  countLeft += direction ? -1 : 1;
}

// Function that stop the motors
void stop() {
  analogWrite(SPEED_LEFT, LOW);
  digitalWrite(DIRECTION_LEFT, LOW);

  analogWrite(SPEED_RIGHT, LOW);
  digitalWrite(DIRECTION_RIGHT, LOW);
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
    digitalWrite(DIRECTION_LEFT, LOW);
    analogWrite(SPEED_LEFT, speed);
    break;
  case 2:
    // Backward
    digitalWrite(DIRECTION_LEFT, HIGH);
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
    digitalWrite(DIRECTION_RIGHT, LOW);
    analogWrite(SPEED_RIGHT, speed);
    break;
  case 2:
    // Backward
    digitalWrite(DIRECTION_RIGHT, HIGH);
    analogWrite(SPEED_RIGHT, speed);
    break;
  }
}

// Function that receive the bytes from the I2C
void recv(int numBytes) {

#ifdef DEBUG
  Serial.println("Reading some data ...");
#endif

  // We should receive 4 bytes from the Rasp
  if (numBytes == 4) {
    Serial.println("Receiving new packet");

    uint8_t PWM_Left = Wire.read();
    uint8_t PWM_Right = Wire.read();
    uint8_t leftDirection = Wire.read();
    uint8_t rightDirection = Wire.read();

#ifdef DEBUG
    Serial.print("PWM_Left : ");
    Serial.println(PWM_Left);

    Serial.print("PWM_Right : ");
    Serial.println(PWM_Right);

    Serial.print("leftDirection : ");
    Serial.println(leftDirection);

    Serial.print("rightDirection : ");
    Serial.println(rightDirection);
#endif

    orderLeft(leftDirection, PWM_Left);
    orderRight(rightDirection, PWM_Right);
  }

  Wire.flush();
  while (Wire.available()) Wire.read();
}