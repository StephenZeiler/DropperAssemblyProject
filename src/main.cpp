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
const int pipetTwisterHomeSensorPin = 28;  // Changed to pin 28

// Pipet system state
enum PipetState {
    PIPET_HOMING,          // Initial homing state
    PIPET_HOMING_COMPLETE, // Twister is at home
    PIPET_RAM_EXTENDING,   // Ram is extending
    PIPET_RAM_RETRACTING,  // Ram is retracting
    PIPET_TWISTER_ACTIVE   // Twister is active
};
PipetState currentPipetState = PIPET_HOMING; // Start in homing state
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
    static bool homingComplete = false;
   bool twisterAtHome = digitalRead(pipetTwisterHomeSensorPin); // Sensor is HIGH when at home
   //bool twisterAtHome = HIGH; // test
    
    // Handle twister homing at startup
    if (!homingComplete) {
        if (!twisterAtHome) {
            digitalWrite(pipetTwisterPin, HIGH);  // Activate twister to move toward home
        } else {
            digitalWrite(pipetTwisterPin, LOW);   // Stop twister when home is reached
            homingComplete = true;
            currentPipetState = PIPET_HOMING_COMPLETE;
        }
        return;
    }

    // Only proceed if homing is complete
    if (currentPipetState == PIPET_HOMING_COMPLETE) {
        // Detect motor deceleration completion (transition from moving to stopped)
        if (lastMotorState && !isMoving) {
            // Step 1: Activate ram immediately
            currentPipetState = PIPET_RAM_EXTENDING;
            digitalWrite(pipetRamPin, HIGH);
            pipetStateStartTime = micros();
            machine.setPipetSystemReady(false);
            
            // Step 2: Schedule twister deactivation after 0.125s
            // This happens simultaneously with Step 1
        }
        
        // Detect motor start (transition from stopped to moving)
        if (!lastMotorState && isMoving) {
            // Step 4: Activate twister after 0.125s of motor start
            pipetStateStartTime = micros();
            currentPipetState = PIPET_TWISTER_ACTIVE;
        }
    }
    
    // State machine transitions
    switch (currentPipetState) {
        case PIPET_RAM_EXTENDING:
            // Step 2: After 0.125s, deactivate twister (move to home)
            if (micros() - pipetStateStartTime >= 125000) {
                digitalWrite(pipetTwisterPin, LOW);
            }
            
            // Step 3: After 0.25s, retract ram
            if (micros() - pipetStateStartTime >= 125000) { 
                digitalWrite(pipetRamPin, LOW);
                currentPipetState = PIPET_HOMING_COMPLETE;
                machine.setPipetSystemReady(true);
            }
            break;
            
        case PIPET_TWISTER_ACTIVE:
            // Step 4: After 0.125s of motor start, activate twister
            if (micros() - pipetStateStartTime >= 125000) {
                digitalWrite(pipetTwisterPin, HIGH);
                currentPipetState = PIPET_HOMING_COMPLETE;
            }
            break;
            
        case PIPET_HOMING_COMPLETE:
            // Waiting for motor state changes
            break;
    }
    
    lastMotorState = isMoving;
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
     pinMode(bulbSeparatorPin, OUTPUT);
pinMode(pipetTwisterHomeSensorPin, INPUT); // Use pullup if sensor is active LOW
    //sensors
    pinMode(homeSensorPin, INPUT);
    pinMode(bulbRamHomeSensorPin, INPUT);
    pinMode(bulbPositionSensorPin, INPUT);
    digitalWrite(pipetTwisterPin, LOW);  // Start with twister off
     currentPipetState = PIPET_HOMING;

    updateSlotPositions();
}

void loop() {
     handleButtons();
   // handleBulbSystem();
    handleCapInjection();
    handleDropperSystem();
    handlePipetSystem();  // Make sure this is uncommented
    
    if (machine.isStopped) return;
    if (machine.needsHoming) {
        homeMachine();
        return;
    }
    
    if (machine.inProduction) {
        stepMotor();
    }
//   digitalWrite(dropperEjectPin, HIGH);
//   digitalWrite(capInjectPin, HIGH);
//   digitalWrite(bulbRamPin, HIGH);
//   digitalWrite(pipetRamPin, HIGH);
//   digitalWrite(pipetTwisterPin, HIGH);
//   digitalWrite(bulbAirPushPin, HIGH);
//   digitalWrite(bulbSeparatorPin, HIGH);

//     delay(2000);
//   digitalWrite(dropperEjectPin, LOW);
//   digitalWrite(capInjectPin, LOW);
//   digitalWrite(bulbRamPin, LOW);
//   digitalWrite(pipetRamPin, LOW);
//   digitalWrite(pipetTwisterPin, LOW);
//   digitalWrite(bulbAirPushPin, LOW);
//   digitalWrite(bulbSeparatorPin, LOW);
//   delay(2000);
}