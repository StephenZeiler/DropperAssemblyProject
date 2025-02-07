
#include <Arduino.h>
#include <Stepper.h>
//Motors
//Currently 400 steps per rev 
const int stepPinM1 = 22; //PUL+ Green Current setp
const int dirPinM1 = 23; //DIR+ Blue
const int enPinM1 = 24; //ENA+ Red
#define TOTAL_STEPS 800   // Steps per cycle
#define MIN_SPEED 10      // Minimum step delay (fastest speed, in microseconds)
#define MAX_SPEED 450    // Maximum step delay (slowest speed, in microseconds)
#define ACCEL_RATE 15000   // Acceleration rate (higher = faster acceleration)
#define CYCLE_DELAY 250000 // 1-second delay between cycles (in microseconds)

unsigned long previousM1Micros = 0;
unsigned long cyclePauseStart = 0;
long stepInterval = MAX_SPEED; // Start at slowest speed
int m1Step = 1;
int currentStep = 0;
bool moving = true;  // Track motor movement state

long m1Speed = 2000; // Initial step delay (microseconds)
long minSpeed = 50; // Minimum step delay (max speed, microseconds)
long accel = 2000;   // Acceleration rate (microseconds per step reduction)

// long calculateDegrees(long rotaryPosition) //converts the steps the stepper has stepped to degrees //a 400 step goes 0.9 degrees per step. 200 stepper motor is 1.8 degrees per step. Currently 800!
// {
//   long result = rotaryPosition * m1PulsePerRevMultiplier; 
//   return result;
// }


void accelerateMotorM1() {
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

      // Acceleration & Deceleration using smooth curve
      float progress = (float)currentStep / TOTAL_STEPS;  // Progress from 0.0 to 1.0

      // Use a sine wave approach for smooth acceleration & deceleration
      stepInterval = MAX_SPEED - (MAX_SPEED - MIN_SPEED) * sin(progress * PI); 

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


// void stepM1()
// {
//   digitalWrite(dirPinM1, HIGH);
//   digitalWrite(stepPinM1, HIGH);
//   delayMicroseconds(9000);
//   digitalWrite(stepPinM1, LOW); 
// }

void setup()
{
 
  //Motors
  pinMode(stepPinM1, OUTPUT);
  pinMode(dirPinM1, OUTPUT);
  pinMode(enPinM1, OUTPUT);
  digitalWrite(enPinM1, LOW);
  
}

void loop()
{
  accelerateMotorM1();
  // for (int i = 0; i < i+10; i++) {
  //       digitalWrite(stepPinM1, HIGH);
  //       delayMicroseconds(500);
  //       digitalWrite(stepPinM1, LOW);
  //       delayMicroseconds(500);
  //   }
}
