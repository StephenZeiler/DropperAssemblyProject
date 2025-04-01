#include <Arduino.h>
#include "SlotObject.h"
#include "MachineState.h"

// Motor pins
const int stepPin = 22;
const int dirPin = 23;
const int enablePin = 24;

// Button pins
const int startButtonPin = 10;
const int pauseButtonPin = 11;
const int stopButtonPin = 12;

// Movement parameters
const int TOTAL_STEPS = 100;
const int ACCEL_STEPS = 70;
const int DECEL_STEPS = 30;
const int MIN_STEP_DELAY = 100;   // microseconds
const int MAX_STEP_DELAY = 2000;  // microseconds
const unsigned long PAUSE_AFTER = 500000; // microseconds

// Sensor
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

MachineState machine;

void updateSlotPositions() {
    for(int i = 0; i < 16; i++) {
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
        }
        else if(slots[i].isAtBulbInjection()) {
            Serial.print("Processing bulb injection at slot ");
            Serial.println(slots[i].getId());
        }
    }
}

void homeMachine() {
    Serial.println("Homing started...");
    unsigned long stepDelay = 5000;
    unsigned long lastStep = micros();
    
    while(digitalRead(homeSensorPin) == HIGH) {
        if(micros() - lastStep >= stepDelay) {
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(10);
            digitalWrite(stepPin, LOW);
            lastStep = micros();
            
            if(digitalRead(stopButtonPin) == HIGH) {
                machine.stop();
                return;
            }
        }
    }
    
    currentHomePosition = 0;
    updateSlotPositions();
    machine.homingComplete();
    isMoving = true;
    lastStepTime = micros();
    Serial.println("Homing complete - Slot 0 at Position 0");
}

void stepMotor() {
    unsigned long currentTime = micros();
    
    if (isMoving) {
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

void handleButtons() {
    static unsigned long lastDebounceTime = 0;
    const unsigned long debounceDelay = 50;
    
    if (millis() - lastDebounceTime < debounceDelay) return;
    
    // Start button
    if (digitalRead(startButtonPin) == HIGH) {
        lastDebounceTime = millis();
        machine.start();
        Serial.println("Start command received");
    }
    
    // Pause button
    if (digitalRead(pauseButtonPin) == HIGH && !machine.isStopped) {
        lastDebounceTime = millis();
        machine.pause();
        isMoving = false;
        Serial.println("Pause command received");
    }
    
    // Stop button
    if (digitalRead(stopButtonPin) == HIGH) {
        lastDebounceTime = millis();
        machine.stop();
        isMoving = false;
        Serial.println("Stop command received");
    }
}

void setup() {
    Serial.begin(115200);
    
    // Initialize pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    pinMode(homeSensorPin, INPUT);
    pinMode(startButtonPin, OUTPUT);
    pinMode(pauseButtonPin, OUTPUT);
    pinMode(stopButtonPin, OUTPUT);

    digitalWrite(enablePin, LOW);
    digitalWrite(dirPin, LOW);
    digitalWrite(stepPin, LOW);
    
    updateSlotPositions();
}

void loop() {
    handleButtons();
    
    if (machine.isStopped) return;
    
    if (machine.needsHoming) {
        homeMachine();
        return;
    }
    
    if (machine.isPaused) return;
    
    if (machine.inProduction) {
        stepMotor();
    }
}