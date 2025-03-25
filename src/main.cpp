#include <Arduino.h>

// Motor pins
const int stepPinM1 = 22;  // PUL - Step
const int dirPinM1 = 23;   // DIR - Direction
const int enPinM1 = 24;    // ENA - Enable

// Movement parameters
const int TOTAL_STEPS = 100;       // Exactly 100 steps
const int MIN_STEP_DELAY = 100;    // Fastest speed (µs)
const int MAX_STEP_DELAY = 2000;   // Slowest speed (µs)
const int ACCEL_STEPS = 70;        // 70 steps acceleration
const int DECEL_STEPS = 30;        // 30 steps deceleration
const unsigned long PAUSE_AFTER = 250000; // Pause after movement (µs)

// Motor state
unsigned long lastStepTime = 0;
unsigned long pauseStartTime = 0;
int stepsTaken = 0;
bool isMoving = true;
bool stepState = false;

void setup() {
  pinMode(stepPinM1, OUTPUT);
  pinMode(dirPinM1, OUTPUT);
  pinMode(enPinM1, OUTPUT);
  
  digitalWrite(enPinM1, LOW);      // Enable motor
  digitalWrite(dirPinM1, HIGH);    // Set CCW direction (never changes)
  lastStepTime = micros();         // Initialize timing
}

void loop() {
  unsigned long currentTime = micros();

  if (isMoving) {
    // Calculate current speed profile
    unsigned long stepDelay;
    if (stepsTaken < ACCEL_STEPS) {
      // Acceleration phase (0-69 steps)
      float progress = (float)stepsTaken / ACCEL_STEPS;
      stepDelay = MAX_STEP_DELAY * pow((float)MIN_STEP_DELAY/MAX_STEP_DELAY, progress);
    } else {
      // Deceleration phase (70-99 steps)
      float progress = (float)(stepsTaken - ACCEL_STEPS) / DECEL_STEPS;
      stepDelay = MIN_STEP_DELAY * pow((float)MAX_STEP_DELAY/MIN_STEP_DELAY, progress);
    }

    // Constrain the step delay to our limits
    stepDelay = constrain(stepDelay, MIN_STEP_DELAY, MAX_STEP_DELAY);

    // Time to take a step?
    if (currentTime - lastStepTime >= stepDelay) {
      // Generate step pulse (rising edge)
      digitalWrite(stepPinM1, HIGH);
      delayMicroseconds(5);        // Minimum pulse width (5µs typical)
      digitalWrite(stepPinM1, LOW);
      
      stepsTaken++;               // Count this step
      lastStepTime = currentTime;  // Reset timing

      // Check if movement complete
      if (stepsTaken >= TOTAL_STEPS) {
        isMoving = false;
        pauseStartTime = currentTime;
        stepsTaken = 0;           // Reset for next movement
      }
    }
  } else {
    // Pause between movements
    if (currentTime - pauseStartTime >= PAUSE_AFTER) {
      isMoving = true;
      lastStepTime = currentTime;  // Reset timing for new movement
    }
  }
}