#include <Arduino.h>

// Motor pins
const int stepPinM1 = 22; // PUL - Step
const int dirPinM1 = 23;  // DIR - Direction
const int enPinM1 = 24;   // ENA - Enable

// Movement parameters
const int TOTAL_STEPS = 100;      // Exactly 100 steps
const int MIN_SPEED = 100;        // Fastest speed (microseconds between steps)
const int MAX_SPEED = 2000;       // Slowest speed (microseconds between steps)
const float ACCEL_PHASE = 0.7f;   // 70% acceleration phase (70 steps)
const float DECEL_PHASE = 0.3f;   // 30% deceleration phase (30 steps)
const unsigned long PAUSE_AFTER = 250000; // Pause after movement (microseconds)

// Motor state
unsigned long lastStepTime = 0;
unsigned long pauseStartTime = 0;
int stepsTaken = 0;
bool isMoving = false;
bool stepState = false;

void setup() {
  pinMode(stepPinM1, OUTPUT);
  pinMode(dirPinM1, OUTPUT);
  pinMode(enPinM1, OUTPUT);
  
  digitalWrite(enPinM1, LOW);      // Enable motor
  digitalWrite(dirPinM1, HIGH);    // Set CCW direction
  isMoving = true;                 // Start moving immediately
}

void loop() {
  unsigned long currentTime = micros();

  if (isMoving) {
    // Calculate progress (0.0 to 1.0)
    float progress = (float)stepsTaken / TOTAL_STEPS;
    
    // Calculate step interval based on phase
    unsigned long stepInterval;
    if (progress < ACCEL_PHASE) {
      // Acceleration phase (0-70 steps)
      float phaseProgress = progress / ACCEL_PHASE;
      stepInterval = MAX_SPEED * pow((float)MIN_SPEED/MAX_SPEED, phaseProgress);
    } else {
      // Deceleration phase (70-100 steps)
      float phaseProgress = (progress - ACCEL_PHASE) / DECEL_PHASE;
      stepInterval = MIN_SPEED * pow((float)MAX_SPEED/MIN_SPEED, phaseProgress);
    }

    // Ensure we stay within speed limits
    stepInterval = constrain(stepInterval, MIN_SPEED, MAX_SPEED);

    // Time to take a step?
    if (currentTime - lastStepTime >= stepInterval) {
      // Generate step pulse
      digitalWrite(stepPinM1, stepState ? HIGH : LOW);
      stepState = !stepState;
      
      // Only count on falling edge
      if (!stepState) {
        stepsTaken++;
        lastStepTime = currentTime;
        
        // Check if we've completed all steps
        if (stepsTaken >= TOTAL_STEPS) {
          isMoving = false;
          pauseStartTime = currentTime;
          stepsTaken = 0;
        }
      }
    }
  } else {
    // Pause between movements
    if (currentTime - pauseStartTime >= PAUSE_AFTER) {
      isMoving = true;
    }
  }
}