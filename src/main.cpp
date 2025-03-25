
#include <Arduino.h>
#include <Stepper.h>
// Motor pins
const int stepPinM1 = 22; // PUL+ Green Current step
const int dirPinM1 = 23;  // DIR+ Blue
const int enPinM1 = 24;   // ENA+ Red

// Constants
#define TOTAL_STEPS 100    // Steps per cycle
#define MIN_SPEED 10       // Minimum step delay (fastest speed, in microseconds)
#define MAX_SPEED 450      // Maximum step delay (slowest speed, in microseconds)
#define ACCEL_RATIO 0.7    // Percentage for acceleration phase
#define CYCLE_DELAY 1000000 // 1-second delay between cycles (in microseconds)

// Global Variables
unsigned long previousMicros = 0;
unsigned long cyclePauseStart = 0;
int currentStep = 0;
bool moving = true;  // Motor movement state
float stepInterval = MAX_SPEED; // Initial step interval

void setup() {
    // Initialize motor pins
    pinMode(stepPinM1, OUTPUT);
    pinMode(dirPinM1, OUTPUT);
    pinMode(enPinM1, OUTPUT);
    digitalWrite(enPinM1, LOW);  // Enable motor driver
    digitalWrite(dirPinM1, LOW); // Set direction

    Serial.begin(9600);  // Start Serial Monitor for debugging
}

void loop() {
    if (moving) {
        accelerateDecelerateMotor();
    } else {
        pauseMotor();
    }
}

void accelerateDecelerateMotor() {
    unsigned long currentMicros = micros();

    if ((currentMicros - previousMicros) >= stepInterval) {
        // Step the motor
        digitalWrite(stepPinM1, HIGH);
        delayMicroseconds(50);  // Step pulse width
        digitalWrite(stepPinM1, LOW);
        currentStep++;

        // Calculate progress (0.0 to 1.0)
        float progress = (float)currentStep / TOTAL_STEPS;

        // Adjust speed based on progress
        if (progress < ACCEL_RATIO) {
            float accelProgress = pow(progress / ACCEL_RATIO, 2); // Quadratic acceleration
            stepInterval = MAX_SPEED - (MAX_SPEED - MIN_SPEED) * accelProgress;
        } else {
            float decelProgress = pow((progress - ACCEL_RATIO) / (1 - ACCEL_RATIO), 2); // Quadratic deceleration
            stepInterval = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * decelProgress;
        }

        previousMicros = currentMicros;

        // Stop the motor after completing all steps
        if (currentStep >= TOTAL_STEPS) {
            moving = false;
            cyclePauseStart = micros();
        }
    }
}

void pauseMotor() {
    unsigned long currentMicros = micros();

    // Pause before restarting motion
    if (currentMicros - cyclePauseStart >= CYCLE_DELAY) {
        currentStep = 0;         // Reset step counter
        stepInterval = MAX_SPEED; // Reset speed
        moving = true;           // Resume motion
    }
}