#include "motor_control.h"
#include <Arduino.h>

// Motor control pins
#define stepPinM1 22
#define dirPinM1 23
#define enPinM1 24
// Motion parameters
#define TOTAL_STEPS 100   // Steps per cycle
#define MIN_SPEED 50      // Minimum step delay (fastest speed, in microseconds)
#define MAX_SPEED 2000    // Maximum step delay (slowest speed, in microseconds)
#define CYCLE_DELAY 1000000 // 1-second delay between cycles (in microseconds)

// Exponential acceleration factor (adjust for faster ramping)
#define ACCEL_FACTOR 2.0   

unsigned long previousM1Micros = 0;
unsigned long cyclePauseStart = 0;
long stepInterval = MAX_SPEED; // Start at slowest speed
int m1Step = 1;
int currentStep = 0;
bool moving = true;  // Track motor movement state

void runMotorM1() {
  pinMode(stepPinM1, OUTPUT);
  pinMode(dirPinM1, OUTPUT);
  pinMode(enPinM1, OUTPUT);
  digitalWrite(enPinM1, LOW);
  digitalWrite(dirPinM1, LOW);  // Set direction

  while (true) {  // Infinite loop to keep motor running
    unsigned long currentMicros = micros();

    if (moving) {
      // Check if it's time to step
      if ((currentMicros - previousM1Micros) >= stepInterval) {
        // Make a step
        if (m1Step == 1) {
          digitalWrite(stepPinM1, HIGH);
          m1Step++;
        } else if (m1Step == 2) {
          digitalWrite(stepPinM1, LOW);
          m1Step = 1;
          currentStep++; // Count steps
        }

        previousM1Micros = currentMicros;

        // Exponential Acceleration & Deceleration
        float progress = (float)currentStep / TOTAL_STEPS;  // Progress from 0.0 to 1.0

        if (progress < 0.7) {  // First 70% = Acceleration
          stepInterval = MAX_SPEED / pow(ACCEL_FACTOR, (progress / 0.7));  // Exponential speed-up
          if (stepInterval < MIN_SPEED) stepInterval = MIN_SPEED;  // Cap at max speed
        } else {  // Last 30% = Deceleration
          float decelProgress = (progress - 0.7) / 0.3;  // Normalize 0.7-1.0 to 0.0-1.0
          stepInterval = MIN_SPEED * pow(ACCEL_FACTOR, decelProgress * 3);  // Exponential slow-down
          if (stepInterval > MAX_SPEED) stepInterval = MAX_SPEED;  // Cap at slowest speed
        }

        // Check if cycle is complete
        if (currentStep >= TOTAL_STEPS) {
          moving = false;
          cyclePauseStart = micros();  // Record the pause start time
          currentStep = 0; // Reset step counter
          stepInterval = MAX_SPEED; // Reset speed for next cycle
        }
      }
    } 
    else {
      // 1-second pause before restarting motion
      if (currentMicros - cyclePauseStart >= CYCLE_DELAY) {
        moving = true;
      }
    }
  }
}