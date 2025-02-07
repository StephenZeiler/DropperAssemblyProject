
#include <Arduino.h>
#include <Stepper.h>
//Motors
//Currently 400 steps per rev 
const int stepPinM1 = 22; //PUL+ Green Current setp
const int dirPinM1 = 23; //DIR+ Blue
const int enPinM1 = 24; //ENA+ Red
long rotaryPosition = 0;
long previousPosition = 0;
int m1Step = 1;
int targetSteps = 400;
int currentStep = 0;
bool accelerating = true;
bool moving = true;  // Track motor movement state
unsigned long previousM1Micros = 0;
unsigned long cyclePauseStart = 0;
unsigned long previousSlowStepM1 = 0;

// long m1Speed = 400; // 1000 is 70/min.... 750 = 90/min... If change, to change speed change the m1 speed in else of runMotorM1()...
// long m2Speed = 910; //150 is 70/min.... 95 = 90/min @800 steps/rev --- 910 @200 steps/rev
// long m3Speed = 1400; //200 is 70/min.... 130 = 90/min @800 steps/rev --- 1250 @200 steps/rev
double m1PulsePerRevMultiplier = 0.9; //.9 for 400, .45 for 800 on driver
#define TOTAL_STEPS 1600  // Steps per cycle
#define MIN_SPEED 200     // Minimum delay between steps (fastest speed, in microseconds) Lower value = Faster speed (smaller delay between steps).
#define MAX_SPEED 600    // Maximum delay between steps (slowest speed, in microseconds) Lower value = Starts at a faster initial speed.
#define ACCEL 8000        // Acceleration rate (microseconds per step reduction) Increase from 2000 to 8000 for a much quicker ramp-up.
#define CYCLE_DELAY 1000000 // 1-second delay between cycles (in microseconds)

long m1Speed = 2000; // Initial step delay (microseconds)
long minSpeed = 50; // Minimum step delay (max speed, microseconds)
long accel = 2000;   // Acceleration rate (microseconds per step reduction)
long stepInterval = m1Speed;
long calculateDegrees(long rotaryPosition) //converts the steps the stepper has stepped to degrees //a 400 step goes 0.9 degrees per step. 200 stepper motor is 1.8 degrees per step. Currently 800!
{
  long result = rotaryPosition * m1PulsePerRevMultiplier; 
  return result;
}


// void runMotorM2()
// {
//     unsigned long currentMicros = micros();
//   digitalWrite(dirPinM2, HIGH);
//   for (int x = 0; x < 1; x++)
//   {
// if((currentMicros - previousM2Micros)> m2Speed)
//   {
//     if(m2Step ==1){
//     digitalWrite(stepPinM2, HIGH);
//       ++m2Step;
//     }
//     else if(m2Step ==2){
//       digitalWrite(stepPinM2, LOW);
//       m2Step = 1;
//     }
//   previousM2Micros = currentMicros; 
  
//   }
//   }
// }
void accelerateMotorM1() {
  digitalWrite(dirPinM1, LOW);
  unsigned long currentMicros = micros();

  // Check if it's time to step
  if ((currentMicros - previousM1Micros) >= stepInterval) {
    // Make a step
    if (m1Step == 1) {
      digitalWrite(stepPinM1, HIGH);
      m1Step++;
      previousPosition = rotaryPosition;
      rotaryPosition++;
    } else if (m1Step == 2) {
      digitalWrite(stepPinM1, LOW);
      m1Step = 1;
    }

    previousM1Micros = currentMicros;

    // Accelerate or decelerate
    if (rotaryPosition < targetSteps / 2) {
      // Accelerate
      if (stepInterval > minSpeed) {
        stepInterval -= accel / targetSteps;
      }
    } else {
      // Decelerate
      if (stepInterval < m1Speed) {
        stepInterval += accel / targetSteps;
      }
    }
  }
}
void accelTest(){
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

      // Adjust acceleration and deceleration logic
      if (currentStep < TOTAL_STEPS * 0.7) {  // Acceleration phase (70% of travel)
        if (stepInterval > MIN_SPEED) {
          stepInterval -= ACCEL / TOTAL_STEPS;  // Speed up faster
        }
      } else {  // Deceleration phase (last 30% of travel)
        if (stepInterval < MAX_SPEED) {
          stepInterval += (ACCEL / (TOTAL_STEPS / 2));  // Slow down more smoothly
        }
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
void runMotorM1()
{
  digitalWrite(dirPinM1, LOW);
  unsigned long currentMicros = micros();
  for (int x = 0; x < 1; x++)
  {
    if((currentMicros - previousM1Micros)> m1Speed){
      if(m1Step ==1){
        digitalWrite(stepPinM1, HIGH);
        ++m1Step;
        previousPosition = rotaryPosition;
        rotaryPosition = rotaryPosition + 1;
      }
      else if(m1Step ==2){
          digitalWrite(stepPinM1, LOW);
          m1Step = 1;
      }
      previousM1Micros = currentMicros; 
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
  accelTest();
  // for (int i = 0; i < i+10; i++) {
  //       digitalWrite(stepPinM1, HIGH);
  //       delayMicroseconds(500);
  //       digitalWrite(stepPinM1, LOW);
  //       delayMicroseconds(500);
  //   }
}
