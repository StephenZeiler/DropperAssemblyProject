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

while (true) {  
    unsigned long currentMicros = micros();

    if (moving) {
      // Check if enough time has passed to make the next step
      if ((currentMicros - previousM1Micros) >= stepInterval) {
        // Make the step
        digitalWrite(stepPinM1, HIGH);
        digitalWrite(stepPinM1, LOW);

        currentStep++; // Increment the step count
        previousM1Micros = currentMicros;  // Update the timestamp for the last step

        // Calculate acceleration and deceleration based on step progress
        float progress = (float)currentStep / TOTAL_STEPS;  // Progress from 0.0 to 1.0

        if (progress < 0.7) {  
          // Exponential acceleration for the first 70% of the total steps
          stepInterval = MAX_SPEED / pow(ACCEL_FACTOR, (progress / 0.7) * 4);
          if (stepInterval < MIN_SPEED) stepInterval = MIN_SPEED;
        } else {  
          // Exponential deceleration for the last 30% of the total steps
          float decelProgress = (progress - 0.7) / 0.3;
          stepInterval = MIN_SPEED * pow(ACCEL_FACTOR, decelProgress * 3);
          if (stepInterval > MAX_SPEED) stepInterval = MAX_SPEED;
        }

        // Stop after reaching the target step count (100 steps)
        if (currentStep >= TOTAL_STEPS) {
          moving = false;
        }
      }
    } 
    else {
      // Once we stop, wait 1 second before restarting the motion cycle
      if ((currentMicros - previousM1Micros) >= CYCLE_DELAY) {
        moving = true;
        currentStep = 0;  // Reset step counter
        stepInterval = MAX_SPEED;  // Reset speed for the next cycle
        previousM1Micros = currentMicros;  // Reset the timing
      }
    }
  }
}