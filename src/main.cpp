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
bool shouldStartMoving = false;
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
    // Simple debounce with immediate response
    static unsigned long lastButtonTime = 0;
    if(millis() - lastButtonTime < 200) return;
    
    // Start button (active LOW)
    if(digitalRead(startButtonPin) == LOW) {
        if(machine.stopped()) {
            machine.startProduction();
            Serial.println("Starting production...");
        } 
        else if(machine.paused()) {
            machine.resumeFromPause();
            isMoving = true;
            Serial.println("Resuming production...");
        }
        lastButtonTime = millis();
        return;
    }
    
    // Pause button
    if(digitalRead(pauseButtonPin) == LOW && !machine.stopped()) {
        machine.pauseProduction();
        isMoving = false;
        Serial.println("Production paused");
        lastButtonTime = millis();
        return;
    }
    
    // Stop button (immediate)
    if(digitalRead(stopButtonPin) == LOW) {
        machine.stopProduction();
        isMoving = false;
        Serial.println("EMERGENCY STOP");
        lastButtonTime = millis();
    }
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
    
    // Set slow speed for homing (5000µs between steps)
    const unsigned long homingSpeed = 5000;
    unsigned long lastStepTime = micros();
    
    // Move until sensor activates (goes HIGH)
    while(digitalRead(homeSensorPin) == LOW) {
        if(micros() - lastStepTime >= homingSpeed) {
            // Generate step pulse
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(10);
            digitalWrite(stepPin, LOW);
            lastStepTime = micros();
            
            // Emergency stop check
            if(digitalRead(stopButtonPin) == LOW) {
                machine.stopProduction();
                return;
            }
        }
    }
    
    // Found home position
    currentHomePosition = 0;
    updateSlotPositions();
    machine.completeHoming();
    isMoving = true; // Start normal movement
    lastStepTime = micros();
    
    Serial.println("Homing complete - Slot 0 at Position 0");
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
    // Always check buttons first
    handleButtons();
    
    // State machine logic
    if(machine.stopped()) {
        return; // Do nothing when stopped
    }
    
    if(machine.requiresHoming()) {
        homeMachine();
        return; // After homing, wait for next loop
    }
    
    if(machine.paused()) {
        return; // Do nothing when paused
    }
    
    if(machine.inProductionMode() && isMoving) {
        stepMotor();
    }
}