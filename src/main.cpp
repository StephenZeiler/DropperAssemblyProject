#include <Arduino.h>
#include <Stepper.h>

// Motors - Currently 400 steps per rev
const int stepPinM1 = 22; // PUL+ Green Current step
const int dirPinM1 = 23;  // DIR+ Blue
const int enPinM1 = 24;   // ENA+ Red

#define TOTAL_STEPS 100    // Total steps to move
#define MIN_SPEED 50       // Minimum step delay (fastest speed, microseconds)
#define MAX_SPEED 2000     // Maximum step delay (slowest speed, microseconds)
#define ACCEL_PHASE 0.7    // 70% of distance for acceleration
#define DECEL_PHASE 0.3    // 30% of distance for deceleration
#define CYCLE_DELAY 250000 // Delay between cycles (microseconds)

unsigned long previousM1Micros = 0;
unsigned long cyclePauseStart = 0;
int currentStep = 0;
bool moving = true;
bool accelerating = true;
int m1Step = 1;

void setup() {
  // Motors
  pinMode(stepPinM1, OUTPUT);
  pinMode(dirPinM1, OUTPUT);
  pinMode(enPinM1, OUTPUT);
  digitalWrite(enPinM1, LOW);
  digitalWrite(dirPinM1, HIGH); // Set direction
}

void loop() {
  accelerateMotorM1();
}

void accelerateMotorM1() {
  unsigned long currentMicros = micros();
  
  if (moving) {
    // Calculate current phase
    float progress = (float)currentStep / TOTAL_STEPS;
    
    // Calculate step interval using exponential curves
    unsigned long stepInterval;
    
    if (progress <= ACCEL_PHASE) {
      // Acceleration phase (exponential)
      float phaseProgress = progress / ACCEL_PHASE;
      stepInterval = MAX_SPEED * pow(MIN_SPEED / (float)MAX_SPEED, phaseProgress);
    } else {
      // Deceleration phase (exponential)
      float phaseProgress = (progress - ACCEL_PHASE) / DECEL_PHASE;
      stepInterval = MIN_SPEED * pow(MAX_SPEED / (float)MIN_SPEED, phaseProgress);
    }
    
    // Constrain the step interval to our defined limits
    stepInterval = constrain(stepInterval, MIN_SPEED, MAX_SPEED);
    
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
      
      // Check if cycle is complete
      if (currentStep >= TOTAL_STEPS) {
        moving = false;
        cyclePauseStart = micros(); // Record the pause start time
        currentStep = 0; // Reset step counter
      }
    }
  } else {
    // Pause before restarting motion
    if (currentMicros - cyclePauseStart >= CYCLE_DELAY) {
      moving = true;
      digitalWrite(dirPinM1, !digitalRead(dirPinM1)); // Reverse direction for next cycle
    }
  }
}