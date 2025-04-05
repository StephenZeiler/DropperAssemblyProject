#include <Arduino.h>
#include "SlotObject.h"
#include "MachineState.h"

// Motor pins
const int stepPin = 22;
const int dirPin = 23;
const int enablePin = 24;

// Button pins (configured as INPUT, not INPUT_PULLUP)
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
bool pauseRequested = false;
bool stopRequested = false;

// Bulb system pins
const int bulbRamHomeSensorPin = 33;
const int bulbPositionSensorPin = 26;
const int bulbAirPushPin = 41;
const int bulbSeparatorPin = 37;
const int bulbRamPin = 39;

//ejection pin
const int dropperEjectPin = 47;

//CAP injection
const int capInjectPin = 35;

// Pipet system pins
const int pipetRamPin = 43;
const int pipetTwisterPin = 45;
const int pipetTwisterHomeSensorPin = 49; // Add appropriate pin number

// Pipet system state
enum PipetState {
    PIPET_IDLE,
    PIPET_RAM_EXTENDING,
    PIPET_RAM_RETRACTING,
    PIPET_TWISTER_HOMING,
    PIPET_TWISTER_ACTIVE
};
PipetState currentPipetState = PIPET_IDLE;
unsigned long pipetStateStartTime = 0;
bool twisterAtHome = false;

// Dropper ejection system state
enum DropperState {
    DROPPER_IDLE,
    DROPPER_EJECTING,
    DROPPER_RETRACTING
};
DropperState currentDropperState = DROPPER_IDLE;

enum CapState {
    CAP_IDLE,
    CAP_INJECTING,
    CAP_RETRACTING
};
CapState currentCapState = CAP_IDLE;

unsigned long dropperStateStartTime = 0;

// Bulb system state
enum BulbState {
    BULB_IDLE,
    BULB_AIR_PUSHING,
    BULB_SEPARATING,
    BULB_RAM_EXTENDING,
    BULB_RAM_RETRACTING
};
BulbState currentBulbState = BULB_IDLE;
unsigned long bulbStateStartTime = 0;
bool motorDecelerated = false;

// Slot tracking
SlotObject slots[] = {
    SlotObject(0), SlotObject(1), SlotObject(2), SlotObject(3),
    SlotObject(4), SlotObject(5), SlotObject(6), SlotObject(7),
    SlotObject(8), SlotObject(9), SlotObject(10), SlotObject(11),
    SlotObject(12), SlotObject(13), SlotObject(14), SlotObject(15)
};
int currentHomePosition = 0;

MachineState machine;
void handlePipetSystem() {
    static bool lastMotorState = false;
    
    // Read sensor
    twisterAtHome = digitalRead(pipetTwisterHomeSensorPin);
    
    // Detect motor deceleration completion
    if (lastMotorState && !isMoving) {
        if (currentPipetState == PIPET_IDLE) {
            currentPipetState = PIPET_RAM_EXTENDING;
            pipetStateStartTime = micros();
            digitalWrite(pipetRamPin, HIGH);
            machine.setPipetSystemReady(false);
            
            // Start twister homing simultaneously (Step 1 & 2)
            digitalWrite(pipetTwisterPin, LOW); // Deactivate twister
        }
    }
    
    // Detect motor acceleration start
    if (!lastMotorState && isMoving) {
        if (currentPipetState == PIPET_IDLE) {
            pipetStateStartTime = micros();
            currentPipetState = PIPET_TWISTER_ACTIVE;
            digitalWrite(pipetTwisterPin, HIGH); // Activate twister
        }
    }
    lastMotorState = isMoving;
    
    // State machine transitions
    switch (currentPipetState) {
        case PIPET_RAM_EXTENDING:
            if (micros() - pipetStateStartTime >= 125000) { // 0.125s after stop
                // Twister should be homing now (already started)
                if (twisterAtHome) {
                    currentPipetState = PIPET_RAM_RETRACTING;
                }
            }
            if (micros() - pipetStateStartTime >= 250000) { // 0.25s after stop
                digitalWrite(pipetRamPin, LOW); // Retract ram
                currentPipetState = PIPET_IDLE;
                machine.setPipetSystemReady(true);
            }
            break;
            
        case PIPET_TWISTER_ACTIVE:
            if (micros() - pipetStateStartTime >= 125000) { // 0.125s after start
                digitalWrite(pipetTwisterPin, LOW); // Deactivate twister
                currentPipetState = PIPET_IDLE;
            }
            break;
            
        case PIPET_IDLE:
            // Waiting for motor state changes
            break;
    }
}

void handleBulbSystem() {
    static bool lastMotorState = false;
    
    // Detect motor deceleration completion
    if (lastMotorState && !isMoving) {
        motorDecelerated = true;
        if (currentBulbState == BULB_IDLE) {
            currentBulbState = BULB_AIR_PUSHING;
            bulbStateStartTime = micros();
            digitalWrite(bulbAirPushPin, HIGH);
            digitalWrite(bulbSeparatorPin, LOW);
            machine.setBulbSystemReady(false);
        }
    }
    lastMotorState = isMoving;
    
    // Read sensors
    bool ramHome = digitalRead(bulbRamHomeSensorPin); //high if home 
    bool bulbPresent = digitalRead(bulbPositionSensorPin); //hgih if present
    
    // State machine transitions
    switch (currentBulbState) {
        case BULB_AIR_PUSHING:
            if (bulbPresent) {
                currentBulbState = BULB_SEPARATING;
                digitalWrite(bulbAirPushPin, LOW);
                digitalWrite(bulbSeparatorPin, HIGH);
                bulbStateStartTime = micros();
            }
            break;
            
        case BULB_SEPARATING:
            if (micros() - bulbStateStartTime >= 125000) {
                currentBulbState = BULB_RAM_EXTENDING;
                digitalWrite(bulbRamPin, HIGH);
                bulbStateStartTime = micros();
            }
            break;
            
        case BULB_RAM_EXTENDING:
            if (micros() - bulbStateStartTime >= 300000) {
                currentBulbState = BULB_RAM_RETRACTING;
                digitalWrite(bulbRamPin, LOW);
            }
            break;
            
        case BULB_RAM_RETRACTING:
            if (ramHome) {
                currentBulbState = BULB_IDLE;
                digitalWrite(bulbSeparatorPin, LOW);
                machine.setBulbSystemReady(true);
                motorDecelerated = false;
            }
            break;
            
        case BULB_IDLE:
            // Waiting for motor to stop
            break;
    }
}

void handleDropperSystem() {
    static bool lastMotorState = false;
    
    // Detect motor deceleration completion
    if (lastMotorState && !isMoving) {
        if (currentDropperState == DROPPER_IDLE) {
            currentDropperState = DROPPER_EJECTING;
            digitalWrite(dropperEjectPin, HIGH);
            dropperStateStartTime = micros();
            machine.setDropperSystemReady(false);
        }
    }
    lastMotorState = isMoving;
    
    // State machine transitions
    switch (currentDropperState) {
        case DROPPER_EJECTING:
            if (micros() - dropperStateStartTime >= 250000) { // 0.125s
                currentDropperState = DROPPER_RETRACTING;
                digitalWrite(dropperEjectPin, LOW);
                machine.setDropperSystemReady(true);
                currentDropperState = DROPPER_IDLE;
            }
            break;
            
        case DROPPER_RETRACTING:
        case DROPPER_IDLE:
            // No action needed
            break;
    }
}

void handleCapInjection() {
    static bool lastMotorState = false;
    
    // Detect motor deceleration completion
    if (lastMotorState && !isMoving) {
        if (currentCapState == CAP_IDLE) {
            currentCapState = CAP_INJECTING;
            digitalWrite(capInjectPin, HIGH);
            dropperStateStartTime = micros();
            machine.setCapInjectionReady(false);
        }
    }
    lastMotorState = isMoving;
    
    // State machine transitions
    switch (currentCapState) {
        case CAP_INJECTING:
            if (micros() - dropperStateStartTime >= 250000) { // 0.125s
                currentCapState = CAP_RETRACTING;
                digitalWrite(capInjectPin, LOW);
                machine.setCapInjectionReady(true);
                currentCapState = CAP_IDLE;
            }
            break;
            
        case CAP_RETRACTING:
        case CAP_IDLE:
            // No action needed
            break;
    }
}


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
            
            if(stopRequested) {
                machine.stop();
                stopRequested = false;
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
                    machine.resetAllPneumatics();

                    updateSlotPositions();
                    processAssembly();
                    
                    // Check if pause was requested during movement
                    if(pauseRequested) {
                        machine.pause();
                        pauseRequested = false;
                        return;
                    }
                    
                    // Check if stop was requested
                    if(stopRequested && currentHomePosition == 0) {
                        machine.stop();
                        stopRequested = false;
                        return;
                    }
                }
            }
            lastStepTime = currentTime;
        }
    } else {
        if (machine.isReadyToMove() && (currentTime - pauseStartTime >= PAUSE_AFTER)) {
            isMoving = true;
            lastStepTime = currentTime;
        }
        //   if (!machine.isPaused && (currentTime - pauseStartTime >= PAUSE_AFTER)) {
        //     isMoving = true;
        //     lastStepTime = currentTime;
        // }
    }
}

void handleButtons() {
    static unsigned long lastDebounceTime = 0;
    const unsigned long debounceDelay = 50;
    static bool lastStartState = LOW;
    static bool lastPauseState = LOW;
    static bool lastStopState = LOW;
    
    bool startState = digitalRead(startButtonPin);
    bool pauseState = digitalRead(pauseButtonPin);
    bool stopState = digitalRead(stopButtonPin);
    
    // Only check buttons if debounce time has passed
    if (millis() - lastDebounceTime < debounceDelay) return;
    
    // Start button pressed (HIGH when pushed)
    if (startState == HIGH && lastStartState == LOW) {
        lastDebounceTime = millis();
        machine.start();
        pauseRequested = false;
        stopRequested = false;
        Serial.println("Start command received");
    }
    
    // Pause button pressed
    if (pauseState == HIGH && lastPauseState == LOW && !machine.isStopped && !machine.isPaused) {
        lastDebounceTime = millis();
        pauseRequested = true;
        Serial.println("Pause requested - will pause at next position");
    }
    
    // Stop button pressed
    if (stopState == HIGH && lastStopState == LOW && !machine.isStopped) {
        lastDebounceTime = millis();
        stopRequested = true;
        Serial.println("Stop requested - will stop when reaching home position");
    }
    
    // Update last states
    lastStartState = startState;
    lastPauseState = pauseState;
    lastStopState = stopState;
}
void setup() {
    Serial.begin(115200);
    
    // Initialize pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
   
    pinMode(startButtonPin, OUTPUT);
    pinMode(pauseButtonPin, OUTPUT);
    pinMode(stopButtonPin, OUTPUT);

    digitalWrite(enablePin, LOW);
    digitalWrite(dirPin, LOW);
    digitalWrite(stepPin, LOW);
    digitalWrite(dropperEjectPin, LOW);

    //Pneumatics
    pinMode(bulbAirPushPin, OUTPUT);
    pinMode(bulbRamPin, OUTPUT);
    pinMode(dropperEjectPin, OUTPUT);
    pinMode(pipetTwisterPin, OUTPUT);
    pinMode(pipetRamPin, OUTPUT);
    pinMode(capInjectPin, OUTPUT);
    //sensors
    pinMode(homeSensorPin, INPUT);
    pinMode(bulbRamHomeSensorPin, INPUT);
    pinMode(bulbPositionSensorPin, INPUT);

    updateSlotPositions();
}

void loop() {
//     handleButtons();
//    // handleBulbSystem();
//     handleCapInjection();
//     handleDropperSystem();  
//     if (machine.isStopped) return;
    
//     if (machine.needsHoming) {
//         homeMachine();
//         return;
//     }
    
//      if (machine.isPaused) return;
    
//     if (machine.inProduction) {
//         stepMotor();
//     }
  digitalWrite(dropperEjectPin, HIGH);
  digitalWrite(capInjectPin, HIGH);
  digitalWrite(bulbRamPin, HIGH);
  digitalWrite(pipetRamPin, HIGH);
  digitalWrite(pipetTwisterPin, HIGH);
  digitalWrite(bulbAirPushPin, HIGH);

    delay(2000);
  digitalWrite(dropperEjectPin, LOW);
  digitalWrite(capInjectPin, LOW);
  digitalWrite(bulbRamPin, LOW);
  digitalWrite(pipetRamPin, LOW);
  digitalWrite(pipetTwisterPin, LOW);
  digitalWrite(bulbAirPushPin, LOW);
  delay(2000);
}