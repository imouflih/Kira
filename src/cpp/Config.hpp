#pragma once

// I2C communication constants
#define ADDRESS_I2C 12
#define START_BYTE 0xA5
#define STOP_BYTE 0x5A
#define INIT_COUNTERS 0xA1

// Odometry configuration
#define ENTRAXE 235                 // distance between the wheels in mm
#define DIAMETER 35                 // diameter of the wheels in mm
#define TICK_PER_TURN 512           // number of ticks per wheel rotation

// Control configuration constants
#define THETA_TOLERANCE 0.03        // rad (tolerance for angle adjustment)

const char* I2C_BUS = "/dev/i2c-1"; // I2C bus
const float Kp = 0.1;               // Proportional gain for speed correction