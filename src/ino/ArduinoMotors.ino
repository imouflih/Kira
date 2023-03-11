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

// Odometry config
#define ENTRAXE 235       // distance between the wheels in mm
#define DIAMETER 35       // diameter of the wheels in mm
#define TICK_PER_TURN 512 // number of ticks per wheel rotation

// Control config
#define THETA_TOLERANCE 0.08 // rad

// Encoder wheels counters
volatile long countRight = 0;
volatile long countLeft = 0;

// Global variables
int16_t x = 0;                   // x position of the robot
int16_t y = 0;                   // y position of the robot
float theta = 0;             // orientation of the robot
int16_t countLeft_previous = 0;  // previous count of left encoder
int16_t countRight_previous = 0; // previous count of right encoder

struct Position {
  int16_t x;
  int16_t y;
  float theta;
};

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
  go_to(100, 100);

  // Start I2C
  Wire.begin(ADDR_I2C);

  // Add interrupt function
  Wire.onReceive(recv);
}

void loop() {

  int8_t directionRight = !digitalRead(DIRECTION_RIGHT_ENCODER);
  int8_t directionLeft = digitalRead(DIRECTION_LEFT_ENCODER);

  //clear screen
  for (int8_t i = 0; i < 50; i++) {
    Serial.println();
  }

  // Update position
  update_position(countLeft, countRight);

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

void go_to(int16_t x_target, int16_t y_target) {

  // Update the current position
  Position position = update_position(countLeft, countRight);
  int16_t x_current = position.x;
  int16_t y_current = position.y;
  float angle_init = position.theta;

  // Calculate the distance and angle between the current position and target position
  int16_t dx = x_target - x_current;
  int16_t dy = y_target - y_current;
  float distance = sqrt(dx * dx + dy * dy);
  float angle = atan2(dy, dx);
  angle = fmod(angle, 4 * PI) < 0 ? fmod(angle, 4 * PI) + 4 * PI : fmod(angle, 4 * PI);

  delay(2000);
  Serial.print("Distance : ");
  Serial.println(distance);
  Serial.print("Target angle : ");
  Serial.println(angle);
  Serial.print("Current angle : ");
  Serial.println(angle_init);
  delay(5000);

  int8_t speed = 15;

  // Turn the robot towards the target position
  setTheta(angle);

  Serial.println("Placed to the target angle!! Going now to target x and y");

  orderMove(speed, speed);

  // Move the robot forward towards the target position
  while (distance > 30) {
    // Update the current position
    position = update_position(countLeft, countRight);
    x_current = position.x;
    y_current = position.y;

    // Calculate the new distance and angle to the target position
    dx = x_target - x_current;
    dy = y_target - y_current;
    distance = sqrt(dx * dx + dy * dy);
    Serial.println("GOING TO THE DIRECTION!!");
    // angle = atan2(dy, dx) * 180.0 / PI;
    // angle = fmod(angle, 4 * PI) < 0 ? fmod(angle, 4 * PI) + 4 * PI : fmod(angle, 4 * PI);

    // // Turn the robot towards the target position
    // setTheta(angle);
  }

  // Stop the robot
  stop();

  // Rotate the robot to the initial angle
  setTheta(angle_init);
}

// Function to rotate the robot
void setTheta(float theta) {

  // Get the current angle of the robot
  Position position = update_position(countLeft, countRight);
  float theta_current = position.theta;

  // Continue adjusting the angle until it is within the specified tolerance
  while (fmod(fabs(theta - theta_current), 4 * PI) > THETA_TOLERANCE) {

    // Rotate the robot, make sure that he always takes the shortest path
    fabs(theta - theta_current) < 6.28 ? theta > theta_current ? orderMove(-15, 15) : orderMove(15, -15) : theta > theta_current ? orderMove(15, -15) : orderMove(-15, 15);

    // Update the current angle of the robot
    position = update_position(countLeft, countRight);
    theta_current = position.theta;
  }

  stop(); // Stop the robot
}

// Function that update (x,y,theta) of the robot
Position update_position(int16_t countLeft, int16_t countRight) {

  // Calculate the distance traveled by each wheel
  float distanceLeft = (countLeft - countLeft_previous) * 2 * PI * DIAMETER / TICK_PER_TURN;
  float distanceRight = (countRight - countRight_previous) * 2 * PI * DIAMETER / TICK_PER_TURN;

  // Calculate the average distance traveled by the robot
  float distanceAverage = (distanceLeft + distanceRight) / 2;

  // Calculate the angle turned by the robot
  float angle = atan2((distanceRight - distanceLeft), ENTRAXE);

  // Calculate the change in position of the robot
  float delta_x = distanceAverage * cos(theta + angle / 2);
  float delta_y = distanceAverage * sin(theta + angle / 2);
  float delta_theta = angle;

  // Update the position of the robot
  x += delta_x;
  y += delta_y;
  theta += delta_theta;

  // Make sure that theta is between 0 and 4*PI
  theta = (fmod(theta, 4 * PI) < 0 ? fmod(theta, 4 * PI) + 4 * PI : fmod(theta, 4 * PI));

  // Update the previous count values
  countLeft_previous = countLeft;
  countRight_previous = countRight;

  Serial.print("Current position: (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print("), theta = ");
  Serial.println(theta);

  // Create and return the Position struct
  Position position = { x, y, theta };
  return position;
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

    update_position(countLeft, countRight);
    orderLeft(leftDirection, PWM_Left);
    orderRight(rightDirection, PWM_Right);
  }

  Wire.flush();
  while (Wire.available()) Wire.read();
}