#include <Arduino.h>
#include "SlotObject.h"
#include "MachineState.h"

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


// Button pins
const int startButtonPin = 10;
const int pauseButtonPin = 11;
const int stopButtonPin = 12;

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

MachineState machine;

// Button states
bool lastStartButtonState;
bool lastPauseButtonState;
bool lastStopButtonState;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

void handleButtons() {
    unsigned long currentTime = millis();
    
    // Read buttons with debounce
    int startReading = digitalRead(startButtonPin);
    int pauseReading = digitalRead(pauseButtonPin);
    int stopReading = digitalRead(stopButtonPin);
    
    // Start button handling
    if (startReading != lastStartButtonState) {
        lastDebounceTime = currentTime;
    }
    
    if ((currentTime - lastDebounceTime) > debounceDelay) {
        // Start button pressed
        if (startReading == LOW && lastStartButtonState == HIGH) {
            if (machine.stopped()) {
                // Full startup with homing
                machine.startProduction();
            } else if (machine.paused()) {
                // Resume from pause
                machine.resumeFromPause();
            }
        }
        
        // Pause button pressed
        if (pauseReading == LOW && lastPauseButtonState == HIGH && !machine.stopped()) {
            machine.pauseProduction();
        }
        
        // Stop button pressed
        if (stopReading == LOW && lastStopButtonState == HIGH) {
            machine.stopProduction();
            isMoving = false; // Immediate stop
        }
    }
    
    // Update last button states
    lastStartButtonState = startReading;
    lastPauseButtonState = pauseReading;
    lastStopButtonState = stopReading;
}

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
    pinMode(startButtonPin, INPUT_PULLUP);
    pinMode(pauseButtonPin, INPUT_PULLUP);
    pinMode(stopButtonPin, INPUT_PULLUP);

    digitalWrite(enablePin, LOW);
    digitalWrite(dirPin, LOW); // CCW rotation
    digitalWrite(stepPin, LOW);
    
    // Initialize slot positions
    updateSlotPositions();
}

void loop() {
       handleButtons();
    
    if (machine.stopped()) {
        // Machine is stopped - do nothing
        return;
    }
    
    if (machine.requiresHoming()) {
        homeMachine();
        machine.completeHoming();
    }
    
    if (machine.paused()) {
        // Machine is paused - wait for resume
        return;
    }
    
    if (machine.inProductionMode()) {
        stepMotor();
    }
    // Add other loop logic as needed
}