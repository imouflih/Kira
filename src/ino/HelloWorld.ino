// #include <Wire.h>

// void setup() {
//   Serial.begin(9600);
//   Wire.begin(12); // Set the I2C address to 8
//   Wire.onRequest(requestEvent); // Set the onRequest event handler
// }

// void loop() {
//   // This loop is intentionally left empty
// }

// void requestEvent() {
//   Serial.println("Request received...");
//   Wire.write("Hello, Iliasse!"); // Respond with "Hello, world!"
// }

/*
   This program is used to control the motors for Krabbs
   see ```github```

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

// I2C address
#define ADDR_I2C   12


// Timeout
#define TIMEOUT_DELAY_MS 10000
uint16_t timeout_timer = 30000;

void setup() {

#ifdef DEBUG
  Serial.begin(115200);
#endif

  // Setup the pins to OUTPUT
  pinMode(SPEED_LEFT, OUTPUT);
  pinMode(DIRECTION_LEFT, OUTPUT);

  pinMode(SPEED_RIGHT, OUTPUT);
  pinMode(DIRECTION_RIGHT, OUTPUT);

  digitalWrite(PIN_POWER_ENABLE, HIGH);

  // Make sure the motors are stopped
  stop();

#ifdef TEST
  // Simple forward then backward
  forward(40);
  delay(1000);

  backward(40);
  delay(1000);

  stop();
#endif

  // Start I2C
  Wire.begin(ADDR_I2C);

  // Add interrupt function
  Wire.onReceive(recv);
}


/* Empty loop function. We wait for an interruption to do something. */
void loop() {
  if (millis() > timeout_timer) {
#ifdef DEBUG
    Serial.println("TIMEOUT !!! Emergency stop");
#endif
    //stop();

  }
}

/* Function that stop the motors */
void stop() {
  analogWrite(SPEED_LEFT, LOW);
  digitalWrite(DIRECTION_LEFT, LOW);

  analogWrite(SPEED_RIGHT, LOW);
  digitalWrite(DIRECTION_RIGHT, LOW);
}

#ifdef TEST
void forward(uint8_t speed) {
  digitalWrite(INPUT_1_LEFT, LOW);
  digitalWrite(INPUT_2_LEFT, HIGH);

  digitalWrite(INPUT_1_RIGHT, LOW);
  digitalWrite(INPUT_2_RIGHT, HIGH);

  analogWrite(ENABLE_RIGHT, speed);
  analogWrite(ENABLE_LEFT, speed);
}

void backward(uint8_t speed) {
  digitalWrite(INPUT_1_LEFT, HIGH);
  digitalWrite(INPUT_2_LEFT, LOW);

  digitalWrite(INPUT_1_RIGHT, HIGH);
  digitalWrite(INPUT_2_RIGHT, LOW);

  analogWrite(ENABLE_RIGHT, speed);
  analogWrite(ENABLE_LEFT, speed);
}
#endif

/* Function that send an order to the right motor */
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

/* Function that send an order to the right motor */
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

/* Function that receive the bytes from the I2C */
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

    timeout_timer = millis() + TIMEOUT_DELAY_MS;
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
