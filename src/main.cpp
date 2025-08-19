
#include <Arduino.h>
#include "SlotObject.h"
#include "MachineState.h"
#include "EasyNextionLibrary.h"

EasyNex myNex(Serial2); // Create an object of EasyNex class with the name < myNex >
long startTime;
// Motor pins
const int stepPin = 22;
const int dirPin = 23;
const int enablePin = 24;

// Button pins (configured as INPUT, not INPUT_PULLUP)
const int startButtonPin = 10;
const int pauseButtonPin = 11;
const int finishProductionButtonPin = 12;
const int speedButtonPin = 8;
const int emptySlotsButtonPin = 9;

// Movement parameters
const int TOTAL_STEPS = 200;  // Changed from 100 to 200
const int ACCEL_STEPS = 60;  // Changed from 70 to 140 (maintains same acceleration ratio) - was 140v
const int DECEL_STEPS = 20;   // Changed from 30 to 60 (maintains same deceleration ratio) - was 60
const int MIN_STEP_DELAY = 40;   // microseconds (keep same for max speed) - was 100
const int MAX_STEP_DELAY = 800;  // microseconds (keep same for start speed) - was 2000
unsigned long PAUSE_AFTER = 100000; // microseconds (keep same pause time)

// Fast values
#define PAUSE_AFTER_FAST     100000 //one second

#define PAUSE_AFTER_SLOW     1000000


// Sensor
const int homeSensorPin = 25;
 const int pipetTipSensor = 31;
// Motor state
unsigned long lastStepTime = 0;
unsigned long pauseStartTime = 0;
int stepsTaken = 0;
bool isMoving = false;
bool stepHigh = false;
bool pauseRequested = false;
bool emptySlotsRequested = false;
bool finsihProdRequested = false;

// Bulb system pins
const int bulbRamHomeSensorPin = 33;
const int bulbPositionSensorPin = 26;
const int bulbRevolverPositionDiscPin = 27; //low means home

//Revolver
const int revolverPUL = 50;
const int revolverDIR = 51;
const int revolverENA = 52;

const int revolverPreLoader = 37;
const int revolverLoader = 41;

//const int bulbAirPushPin = 41; removed
//const int bulbSeparatorPin = 37; removed
const int bulbRamPin = 39;
const int bulbInCapSensor = 30;

//ejection pin
const int dropperEjectPin = 47;

//junk ejector
const int junkEjectorPin = 49;

//CAP injection
const int capInjectPin = 35;
const int capInWheel = 29;

//Empty Slot Sensor
const int slotEmptySensor = 32;

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
float motorPausePercent;
BulbState currentBulbState = BULB_IDLE;
unsigned long bulbStateStartTime = 0;
bool motorDecelerated = false;

int motorSpeedMode = 0;       // 0 = slow, 1 = fast
int lastSw0Val = -1;          // Track last known switch state
const uint8_t SW0_PAGE_ID = 0; // Replace with actual page ID
const uint8_t SW0_COMPONENT_ID = 10; // Replace with actual component ID

// Slot tracking

SlotObject slots[] = {
    SlotObject(0), SlotObject(1), SlotObject(2), SlotObject(3),
    SlotObject(4), SlotObject(5), SlotObject(6), SlotObject(7),
    SlotObject(8), SlotObject(9), SlotObject(10), SlotObject(11),
    SlotObject(12), SlotObject(13), SlotObject(14), SlotObject(15)
};
int slotIdCapInWheelInjection;
int slotIdCapInWheelConfirm;
int slotIdBulbInjection;
int slotIdBulbInCapConfirm;
int slotIdPipetInjection;
int slotIdPipetConfirm;
int slotIdDropeprEjection;
int slotIdJunkEjection;
int slotIdJunkConfirm;
int slotIdFailedJunkEject;
void setSlotIdByPosition(SlotObject slots[])
{
     for(int i = 0; i < 16; i++) {
        if(slots[i].getPosition()==1){
            slotIdCapInWheelInjection= slots[i].getId();
        }
        if(slots[i].getPosition()==2){
            slotIdCapInWheelConfirm= slots[i].getId();
        }
         if(slots[i].getPosition()==5){
            slotIdBulbInjection= slots[i].getId();
        }
        if(slots[i].getPosition()==6){
            slotIdBulbInCapConfirm= slots[i].getId();
        }
        if(slots[i].getPosition()==9){
            slotIdPipetInjection= slots[i].getId();
        }
        if(slots[i].getPosition()==10){
            slotIdPipetConfirm= slots[i].getId();
        }
        if(slots[i].getPosition()==13){
            slotIdDropeprEjection= slots[i].getId();
        }
        if(slots[i].getPosition()==14){
            slotIdJunkEjection= slots[i].getId();
        }
        if(slots[i].getPosition()==15){
            slotIdJunkConfirm= slots[i].getId();
        }
        if(slots[i].getPosition()==0){
            slotIdFailedJunkEject= slots[i].getId();
        }
     }
}
void setSlotErrors(SlotObject slots[])
{
    for(int i = 0; i < 16; i++) {
        if(slots[i].hasJunk() || slots[i].hasMissingBulb() || slots[i].hasMissingCap() ){
            slots[i].setError(true);
        }
        else{
            slots[i].setError(false);
        }
    }
}

int currentHomePosition = 0;
MachineState machine;
// long prevRevolverMicros = 0;  
// int revolverStep = 1;

// void runRevolverMotor(long speed) {
//   digitalWrite(revolverDIR, LOW);
//   long currentMicros = micros(); // Update time inside the check
//   if ((currentMicros - prevRevolverMicros) > speed) {
//     if (revolverStep == 1) {
//       digitalWrite(revolverPUL, HIGH);
//       revolverStep = 2;
//     } 
//     else if (revolverStep == 2) {
//       digitalWrite(revolverPUL, LOW);
//       revolverStep = 1;
//     }
//     prevRevolverMicros = currentMicros;
//   }
// }

// Revolver motor control with acceleration
// unsigned long revolverStepInterval = 5000; // Start with a conservative slow speed (10000µs = 100Hz) 5000
// unsigned long minStepInterval = 300; // Your motor's maximum speed (100µs = 10kHz)
// int acceleration = 25; // How aggressively to accelerate (lower = faster acceleration) 25

void runRevolverMotor(long minStepInterval, int acceleration, long revolverStepInterval) {
  static long prevRevolverMicros = 0;
  static int revolverStepState = 1;
  unsigned long currentMicros = micros();
  
  // Only proceed if it's time for the next step
  if ((currentMicros - prevRevolverMicros) >= revolverStepInterval) {
    // Toggle the step pin
    digitalWrite(revolverPUL, (revolverStepState == 1) ? HIGH : LOW);
    revolverStepState = (revolverStepState == 1) ? 2 : 1;
    // Apply acceleration if not at max speed
    if (revolverStepInterval > minStepInterval) {
      // Reduce the interval (increase speed) based on acceleration factor
      // Using inverse relationship for proper acceleration curve
      revolverStepInterval = revolverStepInterval - (acceleration * revolverStepInterval) / (revolverStepInterval + acceleration);
      
      // Ensure we don't go below minimum interval
      if (revolverStepInterval < minStepInterval) {
        revolverStepInterval = minStepInterval;
      }
    }
    
    prevRevolverMicros = currentMicros;
  }
}

void handleRevolverSystem(){
    static bool lastMotorState = false;
    static unsigned long motorStopTime = 0;
        if (lastMotorState && !isMoving) {
            motorStopTime = micros(); // Record when motor stopped
            machine.setBulbSystemReady(false); // System not ready when motor stops
        }
        unsigned long stopDuration = micros() - motorStopTime;
        float pausePercent = (float)stopDuration / PAUSE_AFTER;

    if(digitalRead(bulbRevolverPositionDiscPin) == LOW){ //check if home
        if(!isMoving){ // addcheck if a bulb is in poisition for pre loading
            static bool preloaderActive = false;  // remembers value between calls
            static unsigned long preloaderTime = 0;

            if (!preloaderActive) {
                digitalWrite(revolverPreLoader, HIGH);
                preloaderTime = micros();
                preloaderActive = true;
            }

            if (preloaderActive && millis() - preloaderTime >= 20000) {
                digitalWrite(revolverLoader, HIGH);
                preloaderActive = false;  // reset for next trigger
            }
            if(pausePercent >= .6){
                digitalWrite(revolverLoader, LOW);
            }
            if(pausePercent >= .75){
                digitalWrite(revolverPreLoader, LOW);
            }
        } 
    }
 lastMotorState = isMoving;
}

void handlePipetSystem() {
    static bool lastMotorState = false;
    static bool homingComplete = false;
    static unsigned long motorStopTime = 0;
    static unsigned long motorStartTime = 0;
    bool twisterAtHome = digitalRead(pipetTwisterHomeSensorPin); // Sensor is HIGH when at home
    
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
    // Track motor state transitions
    if (lastMotorState && !isMoving) {
        motorStopTime = micros(); // Record when motor stopped
    }
    if (!lastMotorState && isMoving) {
        motorStartTime = micros(); // Record when motor started
    }
    lastMotorState = isMoving;

    // Only proceed if homing is complete
    if (currentPipetState == PIPET_HOMING_COMPLETE) {
        if(machine.canPipetProcessStart()){

        if (isMoving) {
            // Motor is moving - handle twister activation after 25% of movement
            unsigned long elapsedSteps = stepsTaken;
            
            // Calculate percentage of movement completed
            float movementPercent = (float)elapsedSteps / TOTAL_STEPS;
            
            // Activate twister after 25% of movement
            //if (movementPercent >= 0.25 && digitalRead(pipetTwisterPin) == LOW) { TEST Remove this comment if issues uncomment
            if (movementPercent >= 0.25 ) {
                digitalWrite(pipetTwisterPin, HIGH);
            }
        } else {
            // Motor is stopped - handle ram and twister timing based on pause duration
            unsigned long stopDuration = micros() - motorStopTime;
            float pausePercent = (float)stopDuration / PAUSE_AFTER;
            
            // Activate ram after 5% of pause time
            if (pausePercent >= 0.01 && pausePercent < 0.90 && digitalRead(pipetRamPin) == LOW) {
                if(!slots[slotIdPipetInjection].hasError() && !slots[slotIdPipetInjection].shouldFinishProduction()){
                //if(!slots[slotIdPipetInjection].hasMissingBulb()){

                    digitalWrite(pipetRamPin, HIGH);
                }
                machine.setPipetSystemReady(false);
            }
            
            // Deactivate ram after 90% of pause time
            //if (pausePercent >= 0.90 && digitalRead(pipetRamPin) == HIGH) { // TEST remove this comment if causing issues
            if (pausePercent >= 0.90) {
                digitalWrite(pipetRamPin, LOW);
                if(twisterAtHome){
                    machine.setPipetSystemReady(true);              
                }
            }
            // Deactivate twister after 75% of pause time
            //if (pausePercent >= 0.75 && digitalRead(pipetTwisterPin) == HIGH && !slots[slotIdPipetInjection].hasMissingBulb()) {
                if (pausePercent >= 0.75 && digitalRead(pipetTwisterPin) == HIGH && (!slots[slotIdPipetInjection].hasError() && !slots[slotIdPipetInjection].shouldFinishProduction())) {
                    digitalWrite(pipetTwisterPin, LOW);
                }
            }
            if(slots[slotIdPipetInjection].shouldFinishProduction()){
                digitalWrite(pipetTwisterPin, HIGH);
                if(twisterAtHome){
                 machine.setPipetSystemReady(true);
                }
            }
        }
        else{
            digitalWrite(pipetTwisterPin, HIGH);
            machine.setPipetSystemReady(true);
        }
    }
}
void motorPauseTime() {
    static bool lastMotorState = false;
    static unsigned long motorStopTime = 0;
    
    // Track motor state transitions
    if (lastMotorState && !isMoving) {
        motorStopTime = micros(); // Record when motor stopped
    }
    lastMotorState = isMoving;
    
    // Only calculate pause percent when motor is stopped
    if(!isMoving) {
        unsigned long stopDuration = micros() - motorStopTime;
        motorPausePercent = (float)stopDuration / PAUSE_AFTER;
    } else {
        motorPausePercent = 0; // Reset when motor starts moving
    }
}
void handleBulbSystem() {
    static bool lastMotorState = false;
    static unsigned long motorStopTime = 0;
    static unsigned long motorStartTime = 0;
    static bool ramExtended = false;
    static bool ramRetracted = true;
    
    // Track motor state transitions
    if (lastMotorState && !isMoving) {
        motorStopTime = micros(); // Record when motor stopped
        machine.setBulbSystemReady(false); // System not ready when motor stops
        ramRetracted = false; // Ram needs to retract again
    }
    if (!lastMotorState && isMoving) {
        motorStartTime = micros(); // Record when motor started
    }
    lastMotorState = isMoving;

    // Read sensors
    bool ramHome = digitalRead(bulbRamHomeSensorPin); // HIGH if home
    bool bulbPresent = digitalRead(bulbPositionSensorPin); // HIGH if present
    bool revolverSensor = digitalRead(bulbRevolverPositionDiscPin);
    
    if(machine.canBulbProcessStart()){
        if (isMoving) {
            // Handle revolver movement
            unsigned long elapsedSteps = stepsTaken;
           // unsigned long totalMovementTime = micros() - motorStartTime;
            
            // Calculate percentage of movement completed
            float movementPercent = (float)elapsedSteps / TOTAL_STEPS;
            if(machine.shouldRevolverMove() && movementPercent >= .01){
                //if(!slots[slotIdBulbInjection].hasMissingCap()){
                if(!slots[slotIdBulbInjection].hasError() && !slots[slotIdBulbInjection].shouldFinishProduction()){
                    runRevolverMotor(300,15,400);
                }
                //runRevolverMotor(500,30,700); faster but only for 400 steps/rev

            }
            if (revolverSensor == LOW && movementPercent >= .06){
                machine.setShouldRevolverMove(false); 
            }                     
        }        
        else {
            // Motor is stopped - handle timing based on pause duration
            unsigned long stopDuration = micros() - motorStopTime;
            float pausePercent = (float)stopDuration / PAUSE_AFTER;
            
            
            // Activate ram after 5% of pause time (only if bulb position sensor reads LOW)
            if (pausePercent >= 0.01 && pausePercent < 0.95 && !digitalRead(bulbRamPin) && bulbPresent && !revolverSensor) {
               // if(!slots[slotIdBulbInjection].hasMissingCap()){
                if(!slots[slotIdBulbInjection].hasError() && !slots[slotIdBulbInjection].shouldFinishProduction()){
                digitalWrite(bulbRamPin, HIGH);
                }
                ramExtended = true;
                ramRetracted = false;
                bulbPresent = true;
            }
            else if(pausePercent >= 0.01 && pausePercent < 0.95 && !digitalRead(bulbRamPin) && !bulbPresent){
                machine.bulbPresent = false;
            }
            
            // Deactivate ram after 95% of pause time
            if (pausePercent >= 0.95 && digitalRead(bulbRamPin)) {
                digitalWrite(bulbRamPin, LOW);
                machine.setShouldRevolverMove(true);
            }
            
            // Only set system ready when ram is confirmed home and retracted
            if (ramExtended && ramHome && !digitalRead(bulbRamPin)) {
                machine.setBulbSystemReady(true);
                ramExtended = false;
                ramRetracted = true;
            }
        }
    }
    else{
        machine.setBulbSystemReady(true);
    }
}



void updateSlotPositions() {
    for(int i = 0; i < 16; i++) {
        int relativePos = (currentHomePosition + slots[i].getId()) % 16;
        slots[i].setPosition(relativePos);
    }
    machine.IncrementPositionsMoved();
}

bool shouldRunTracker = true;
void machineTracker(){
    static bool lastMotorState = false;
    static unsigned long motorStopTime = 0;
     if (lastMotorState && !isMoving) {
        motorStopTime = micros(); // Record when motor stopped
    }
    lastMotorState = isMoving;
    unsigned long stopDuration = micros() - motorStopTime;

    if(!isMoving && shouldRunTracker){  
    shouldRunTracker = false;
    motorPauseTime();
    if(finsihProdRequested){
        // String test = (String)"Finish production active" ;
        // myNex.writeStr("errorTxt.txt+",  "Finish prod active \\r");
        slots[slotIdCapInWheelInjection].setFinsihProduction(true);
    }
    if(digitalRead(pipetTipSensor) == HIGH && machine.canPipetConfirmStart() && !slots[slotIdPipetConfirm].shouldFinishProduction()){
        slots[slotIdPipetConfirm].setJunk(true);        
        // String test = (String)slotIdPipetConfirm +   " slot has junk: "+ (String)slots[slotIdPipetConfirm].hasJunk() ;
        // myNex.writeStr("errorTxt.txt+", test +" \\r");
    }
    else{                
       slots[slotIdPipetConfirm].setJunk(false);
    }
    if(digitalRead(bulbInCapSensor) == LOW && machine.canBulbConfirmStart()&& !slots[slotIdBulbInCapConfirm].shouldFinishProduction()){
         slots[slotIdBulbInCapConfirm].setMissingBulb(true);  
    }
     else{                
       slots[slotIdBulbInCapConfirm].setMissingBulb(false);
    }
    if(digitalRead(capInWheel) == LOW && machine.canCapConfirmStart() && !slots[slotIdCapInWheelConfirm].shouldFinishProduction()){
         slots[slotIdCapInWheelConfirm].setMissingCap(true);  
    }
     else{                
       slots[slotIdCapInWheelConfirm].setMissingCap(false);
    }

    if(machine.canJunkEjectionStart()){
        if(stopDuration < 18000 && !isMoving){
            digitalWrite(junkEjectorPin, HIGH);
        }
        if(slots[slotIdJunkEjection].hasError()){
            machine.incrementErroredDroppers();
        }
    }
     
    if(machine.canDropperEjectionStart() && !slots[slotIdDropeprEjection].hasError() && !slots[slotIdDropeprEjection].shouldFinishProduction()){
        machine.incrementDroppersCompleted();
        if(stopDuration < 18000 && !isMoving){
            digitalWrite(dropperEjectPin, HIGH);
        }
        
    }

    // if(motorPausePercent>.4){
    //     //Shut off ejectors for junk etc.
    //     digitalWrite(junkEjectorPin, LOW);
    //     digitalWrite(dropperEjectPin, LOW);
    // }
    if(machine.canCheckForEmptyStart() && digitalRead(slotEmptySensor) == HIGH){
        slots[slotIdJunkConfirm].setFailedJunkEject(true);
        //         String test = (String)slotIdJunkConfirm +   " failed junk: "+ (String)slots[slotIdJunkConfirm].hasFailedJunkEject() ;
        // myNex.writeStr("errorTxt.txt+", test +" \\r");
    }
    else if(machine.canCheckForEmptyStart() && digitalRead(slotEmptySensor) == LOW){
        slots[slotIdJunkConfirm].setFailedJunkEject(false);
        slots[slotIdJunkConfirm].setJunk(false);
        slots[slotIdJunkConfirm].setMissingBulb(false);
        slots[slotIdJunkConfirm].setMissingCap(false);
        slots[slotIdJunkConfirm].setError(false);
    }
    
    
    if(slots[slotIdFailedJunkEject].hasFailedJunkEject()){
        machine.stop();
    }
    if(slots[slotIdJunkEjection].shouldFinishProduction()){ 
        machine.pause();
        digitalWrite(junkEjectorPin, LOW);
        digitalWrite(dropperEjectPin, LOW);
    }
}
if(isMoving || machine.isStopped || machine.isPaused){
        //Shut off ejectors for junk etc.
        digitalWrite(junkEjectorPin, LOW);
        digitalWrite(dropperEjectPin, LOW);
}
setSlotErrors(slots);
 if(stopDuration > 18000){
            digitalWrite(junkEjectorPin, LOW);
            digitalWrite(dropperEjectPin, LOW);
}
}
void handleCapInjection(){
if(machine.inProduction && !slots[slotIdCapInWheelInjection].hasError() && !slots[slotIdCapInWheelInjection].shouldFinishProduction()){
    digitalWrite(capInjectPin, HIGH);
}
else{
    digitalWrite(capInjectPin, LOW);
}
}
void homeMachine() {   
    unsigned long stepDelay = 5000;
    unsigned long lastStep = micros();
    while(digitalRead(homeSensorPin) == HIGH) {
        digitalWrite(capInjectPin, LOW);
        if(!digitalRead(pauseButtonPin)){
            machine.stop();
            machine.updateStatus( myNex, "Homing Stopped");
            break;
        }
        if(micros() - lastStep >= stepDelay) {
            //digitalWrite(junkEjectorPin, HIGH);
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(10);
            digitalWrite(stepPin, LOW);
            lastStep = micros();
            //TODO: replace with pause requested - made some changes to test
            // if(pauseRequested) {
            //     machine.stop();
            //     pauseRequested = false;
            //     return;
            // }
        }
    }
    if(digitalRead(homeSensorPin) == LOW ){ 
        machine.inProduction = true;    
        digitalWrite(junkEjectorPin, LOW);  
        digitalWrite(capInjectPin, HIGH);
        delay(500);
        digitalWrite(capInjectPin, LOW);
        delay(250);
    }
    
    currentHomePosition = 0;
    //updateSlotPositions();
    machine.homingComplete();
    isMoving = true;
    lastStepTime = micros();
    //Serial.println("Homing complete - Slot 0 at Position 0");
}
bool puasedStateProcessing = false;
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
                    shouldRunTracker = true;
                    isMoving = false;
                    pauseStartTime = currentTime;
                    stepsTaken = 0;
                    machine.resetAllPneumatics();
                   // if(puasedStateProcessing){
                        currentHomePosition = (currentHomePosition + 1) % 16;
                        updateSlotPositions();
                       // processAssembly();
                //         String test = "current position " + (String)currentHomePosition;
                // myNex.writeStr("errorTxt.txt+", test +" \\r");
                       // puasedStateProcessing= false;
                   // }
                    
                    // Check if pause was requested during movement

                    
                    // Check if stop was requested
                    // if(finsihProdRequested && currentHomePosition == 0) {
                    //     machine.stop();
                    //     finsihProdRequested = false;
                    //     return;
                    // }
                }
                else{
                    puasedStateProcessing = true;
                }
            }
            lastStepTime = currentTime;
        }
    } else {
            if(pauseRequested) {
                machine.pause();
                pauseRequested = false;
                return;
            }
        // Only start moving if ALL systems are ready AND pause time has elapsed
        if (machine.isReadyToMove() && (currentTime - pauseStartTime >= PAUSE_AFTER)) {
            // Additional safety check - confirm ram is home before moving
            if (digitalRead(bulbRamHomeSensorPin) && !digitalRead(bulbRamPin)) {
                isMoving = true;
                lastStepTime = currentTime;
            }
        }
    }
}
void fillRevolver() {
  static bool armed = true;

  const uint16_t debounceMs = 15;
  static bool     debouncedSensor = HIGH;   // HIGH = away, LOW = at index
  static bool     lastRawSensor   = HIGH;
  static uint32_t lastFlipMs      = 0;

  while (machine.revolverEmpty) {
    bool bulbPresent = (digitalRead(bulbPositionSensorPin) == HIGH);
    bool sensorRaw   = digitalRead(bulbRevolverPositionDiscPin); // LOW = at index

    uint32_t now = millis();
    if (sensorRaw != lastRawSensor) { lastRawSensor = sensorRaw; lastFlipMs = now; }
    else if ((now - lastFlipMs) >= debounceMs) { debouncedSensor = lastRawSensor; }

    bool atIndex = (debouncedSensor == LOW);

    if (bulbPresent && atIndex) { machine.revolverFilled(); break; }

    if (atIndex && armed) {
      digitalWrite(revolverPreLoader, HIGH); delay(20);
      digitalWrite(revolverLoader, HIGH);    delay(30);
      digitalWrite(revolverLoader, LOW);     delay(10);
      digitalWrite(revolverPreLoader, LOW);  delay(20);
      armed = false;
    } else {
      runRevolverMotor(1200, 25, 1400);
    }

    if (!atIndex) armed = true;
  }
}


void emptySlots() {
    machine.updateStatus( myNex, "Emptying Slots");
    const unsigned long stepDelay = 4000; // 5ms per step = 200 steps in ~1 second
    digitalWrite(pipetTwisterPin, LOW);
int i = 0;
    while(i<=3200){ //full rotation based on 16 slots 200 per slot
        if(!digitalRead(pauseButtonPin)){
            machine.stop();
            machine.updateStatus( myNex, "Emptying Stopped");
            break;
        }
        if(i%200 == 0){
            digitalWrite(junkEjectorPin, HIGH);
            delay(200);
            digitalWrite(junkEjectorPin, LOW);
            delay(200);
        }
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(10); // pulse width
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelay); // delay between steps
        ++i;
}
machine.updateStatus( myNex, "Emptying Completed");
    digitalWrite(junkEjectorPin, LOW);
    digitalWrite(pipetTwisterPin, HIGH);
    machine.stop();
}
void handleSpeedButton(){
    if (!digitalRead(speedButtonPin)) {
    PAUSE_AFTER = PAUSE_AFTER_SLOW;
  } else {
    PAUSE_AFTER = PAUSE_AFTER_FAST;
  }    
}
void handleEmptySlots(){
    if(!machine.inProduction && !digitalRead(emptySlotsButtonPin)){
        emptySlots();
    }
}
void handleButtons() {
    static unsigned long lastDebounceTime = 0;
    const unsigned long debounceDelay = 50;
    static bool lastStartState = LOW;
    static bool lastPauseState = LOW;
    static bool lastFinishState = LOW;
    
    bool startState = digitalRead(startButtonPin);
    bool pauseState = digitalRead(pauseButtonPin);
    bool finishState = digitalRead(finishProductionButtonPin);
    //bool finishState = digitalRead(singleStepButtonPin);
    
    // Only check buttons if debounce time has passed
    if (millis() - lastDebounceTime < debounceDelay) return;
    handleSpeedButton();
    handleEmptySlots();
    // Start button pressed (HIGH when pushed)
    if (startState == LOW && lastStartState == HIGH) {
        lastDebounceTime = millis();
        machine.start();
        pauseRequested = false;
        finsihProdRequested = false;
        machine.updateStatus(myNex,"In Production");
    }
    // Pause button pressed
    if (pauseState == LOW && lastPauseState == HIGH && !machine.isStopped && !machine.isPaused) {
        machine.updateStatus(myNex, "Paused");
        lastDebounceTime = millis();
        pauseRequested = true;
    }
    
    // finsih button pressed
    if (finishState == LOW && lastFinishState == HIGH && !machine.isStopped) {
        if(machine.inProduction){
            machine.updateStatus(myNex, "End Production");
        }
        lastDebounceTime = millis();
        finsihProdRequested = true;
    }
    
    // Update last states
    lastStartState = startState;
    lastPauseState = pauseState;
    lastFinishState = finishState;
}
// void trigger1() {//empty slots trigger
//     if(!machine.inProduction){
//         emptySlots();
//     }
// }
void setup() {
    //Serial.begin(115200);
     myNex.begin(115200); 
      
    // Initialize pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
   
    pinMode(startButtonPin, INPUT_PULLUP);
    pinMode(pauseButtonPin, INPUT_PULLUP);
    pinMode(finishProductionButtonPin, INPUT_PULLUP);
    pinMode(emptySlotsButtonPin, INPUT_PULLUP);
    pinMode(speedButtonPin, INPUT_PULLUP);

    digitalWrite(enablePin, LOW);
    digitalWrite(dirPin, LOW);
    digitalWrite(stepPin, LOW);
    digitalWrite(dropperEjectPin, LOW);
    //Revolver
    pinMode(revolverDIR, OUTPUT);
    pinMode(revolverENA, OUTPUT);
    pinMode(revolverPUL, OUTPUT);
    digitalWrite(revolverPUL, LOW);
    digitalWrite(revolverENA, LOW);
    digitalWrite(revolverDIR, HIGH);

    //Pneumatics
    delay(100);
    //pinMode(bulbAirPushPin, OUTPUT);
    pinMode(bulbRamPin, OUTPUT);
    pinMode(dropperEjectPin, OUTPUT);
    pinMode(pipetTwisterPin, OUTPUT);
    pinMode(pipetRamPin, OUTPUT);
    pinMode(capInjectPin, OUTPUT);
    pinMode(junkEjectorPin, OUTPUT);
    //pinMode(bulbSeparatorPin, OUTPUT);
    pinMode(pipetTwisterHomeSensorPin, INPUT); // Use pullup if sensor is active LOW
    //sensors
    pinMode(bulbRevolverPositionDiscPin, INPUT);
    pinMode(homeSensorPin, INPUT);
    pinMode(bulbRamHomeSensorPin, INPUT);
    pinMode(bulbPositionSensorPin, INPUT);
    pinMode(pipetTipSensor, INPUT);
    pinMode(bulbInCapSensor, INPUT);
    pinMode(capInWheel, INPUT);
    pinMode(slotEmptySensor, INPUT);

    digitalWrite(pipetTwisterPin, LOW);  // Start with twister off
    digitalWrite(bulbRamPin, LOW);
    digitalWrite(capInjectPin, LOW);
    digitalWrite(pipetRamPin, LOW);
    digitalWrite(junkEjectorPin, LOW);
    currentPipetState = PIPET_HOMING;
    updateSlotPositions();
    
}


int i = 0;

void loop() {
    
    // handleButtons();
    // handleCapInjection();
    // setSlotIdByPosition(slots);
    // machineTracker();

    // startTime = millis();
    // motorPauseTime();
    // if(!isMoving && motorPausePercent>.90){
    //    machine.updateMachineDisplayInfo(myNex, startTime, slots);
    // }
    
    // handleBulbSystem();
    // handlePipetSystem();  // Make sure this is uncommented
    
    // if (machine.isStopped) return;
    // if (machine.needsHoming || machine.revolverEmpty) {
    //     if(machine.revolverEmpty){
    //         machine.updateStatus(myNex,"Revolver homing");
    //         fillRevolver();
    //     }
    //     if(machine.needsHoming){
    //         machine.updateStatus(myNex,"Motor Homing");
    //         homeMachine();
    //     }
    //     if(!machine.needsHoming && !machine.isPaused  && !machine.isStopped){
    //         machine.updateStatus(myNex,"In Production");
    //     }
    //     return;
    // }
    
    // if (machine.inProduction) {
    //     stepMotor();
    // }

fillRevolver();





// if(digitalRead(finishProductionButtonPin)){
// digitalWrite(capInjectPin, HIGH);
// }
// else{
// digitalWrite(capInjectPin, LOW);
// }
// i++;
//myNex.writeStr("cautiontTxt.txt+", (String)i+"\\r");
// myNex.writeStr("logTxt.txt", "test1\\r");
// delay(2950);
// String time = msToHMS(currentUptime);
// myNex.writeStr("t2.txt", time);
}
