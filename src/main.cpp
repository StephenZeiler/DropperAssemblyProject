#include <Arduino.h>
#include "SlotObject.h"

// Motor pins
const int stepPin = 22;
const int dirPin = 23;
const int enablePin = 24;

// Movement parameters
const int TOTAL_STEPS = 100;
const int ACCEL_STEPS = 70;
const int DECEL_STEPS = 30;
const int MIN_STEP_DELAY = 100;   // microseconds
const int MAX_STEP_DELAY = 2000;  // microseconds
const unsigned long PAUSE_AFTER = 500000; // microseconds

//Sensor
const int homeSensorPin = 25;

// Motor state
unsigned long lastStepTime = 0;
unsigned long pauseStartTime = 0;
int stepsTaken = 0;
bool isMoving = false;
bool stepHigh = false;

// Slot tracking
SlotObject slots[] = {
    SlotObject(0), SlotObject(1), SlotObject(2), SlotObject(3),
    SlotObject(4), SlotObject(5), SlotObject(6), SlotObject(7),
    SlotObject(8), SlotObject(9), SlotObject(10), SlotObject(11),
    SlotObject(12), SlotObject(13), SlotObject(14), SlotObject(15)
};
int currentHomePosition = 0;



void updateSlotPositions() {
    for(int i = 0; i < 16; i++) {
        // Simple forward position calculation
        int relativePos = (currentHomePosition + slots[i].getId()) % 16;
        slots[i].setPosition(relativePos);
    }
}

void processAssembly() {
    for(int i = 0; i < 16; i++) {
        if(slots[i].getError()) {
            Serial.print("Slot ");
            Serial.print(slots[i].getId());
            Serial.println(" has error - skipping");
            continue;
        }
        
        if(slots[i].isAtCapInjection()) {
            Serial.print("Processing cap injection at slot ");
            Serial.println(slots[i].getId());
            // Add cap injection logic
        }
        else if(slots[i].isAtBulbInjection()) {
            Serial.print("Processing bulb injection at slot ");
            Serial.println(slots[i].getId());
            // Add bulb injection logic
        }
        // Add other position handlers...
    }
}

void homeMachine() {
    Serial.println("Homing started...");
    
    // Set slow speed for homing
    digitalWrite(dirPin, HIGH); // Direction doesn't matter for homing
    unsigned long stepDelay = 5000; // Very slow speed (5000µs between steps)
    
    while(digitalRead(homeSensorPin) == HIGH) {
        // Move until sensor activates
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(10); // Short pulse
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelay);
    }
    
    // Sensor is now HIGH - we're home
    Serial.println("Homing complete - Slot 0 at Position 0");
    currentHomePosition = 0;
    //updateSlotPositions();
    
    // Optional: back off slightly if needed
    // for(int i=0; i<5; i++) { stepMotorReverse(); }
}

void stepMotor() {
    unsigned long currentTime = micros();
    
    if (isMoving) {
        // Calculate current speed profile
        unsigned long stepDelay;
        if (stepsTaken < ACCEL_STEPS) {
            float progress = (float)stepsTaken / ACCEL_STEPS;
            stepDelay = MAX_STEP_DELAY * pow((float)MIN_STEP_DELAY/MAX_STEP_DELAY, progress);
        } else {
            float progress = (float)(stepsTaken - ACCEL_STEPS) / DECEL_STEPS;
            stepDelay = MIN_STEP_DELAY * pow((float)MAX_STEP_DELAY/MIN_STEP_DELAY, progress);
        }

        stepDelay = constrain(stepDelay, MIN_STEP_DELAY, MAX_STEP_DELAY);

        if (currentTime - lastStepTime >= stepDelay) {
            if (!stepHigh) {
                digitalWrite(stepPin, HIGH);
                stepHigh = true;
            } else {
                digitalWrite(stepPin, LOW);
                stepHigh = false;
                stepsTaken++;
                
                if (stepsTaken >= TOTAL_STEPS) {
                    isMoving = false;
                    pauseStartTime = currentTime;
                    stepsTaken = 0;
                    
                    // Update to next position
                    currentHomePosition = (currentHomePosition + 1) % 16;
                    updateSlotPositions();
                    processAssembly();
                }
            }
            lastStepTime = currentTime;
        }
    } else {
        if (currentTime - pauseStartTime >= PAUSE_AFTER) {
            isMoving = true;
            lastStepTime = currentTime;
        }
    }
}

void setup() {
    Serial.begin(115200);
    
    // Initialize motor pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    pinMode(homeSensorPin, INPUT); // Home sensor input
    
    digitalWrite(enablePin, LOW);
    digitalWrite(dirPin, LOW); // CCW rotation
    digitalWrite(stepPin, LOW);
    
    // Initialize slot positions
    updateSlotPositions();
}

void loop() {
    homeMachine();
    //stepMotor();
    // Add other loop logic as needed
}