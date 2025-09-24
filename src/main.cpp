
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

//Production Supply Indicators
const int pipetSupplySensorPin = 34; //Signal goes high when no pipet
const int bulbSupplySensorPin = 40; //Signal goes high when no bulb
const int capSupplySensorPin = 38; //Signal goes high when no cap

// Movement parameters
const int TOTAL_STEPS = 200;  // Changed from 100 to 200

//30percentincrease
int MIN_STEP_DELAY = 31;   // 40 * 0.77 ≈ 31
int MAX_STEP_DELAY = 616;  // 800 * 0.77 ≈ 616
int ACCEL_STEPS   = 46;    // 60 * 0.77 ≈ 46
int DECEL_STEPS   = 15;    // 20 * 0.77 ≈ 15

// const int MIN_STEP_DELAY = 27;   // was 40
// const int MAX_STEP_DELAY = 533;  // was 800
// const int ACCEL_STEPS   = 40;    // was 60
// const int DECEL_STEPS   = 13;    // was 20


unsigned long PAUSE_AFTER = 90000; // microseconds (keep same pause time)

//Potentiameter low and high range
const unsigned long PAUSE_MIN_US = 90000UL;  // fastest allowed pause
const unsigned long PAUSE_MAX_US = 400000UL; // slowest allowed pause
// Fast values

const unsigned long PAUSE_LOW_SUPPLY = 2000000UL;
//Potentiameter
const int speedPotPin = A1;                   // wire your pot wiper here

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
const int bulbInPreLoadPosSensorPin = 26; //HIGH = something blocking sensor
const int preLoadCylinderHomeSensorPin = 53; //HIGH = something blocking sensor, LOW when home

const int bulbPreLoadCylinder = 37;
//const unsigned long PRELOAD_PULSE_US = 20000;
const unsigned long PRELOAD_PULSE_US = 160000;
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

//Low Air Sensor 
const int lowAirSensorPin = 2; //Reads HIGH when no air.

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
int slotIdBulbPreLoad;
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
            slotIdBulbPreLoad= slots[i].getId();
        }
         if(slots[i].getPosition()==6){
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

bool handleLowSupplies(){
    if(digitalRead(capSupplySensorPin)== HIGH){
        return true;
    }
    if(digitalRead(bulbSupplySensorPin)== HIGH){
        return true;
    }
    if(digitalRead(pipetSupplySensorPin)== HIGH){
        return true;
    }
}
int currentHomePosition = 0;
MachineState machine;

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

    // NEW: preloader state (fires once per stop)
    static bool preloadArmed = false;            // becomes true when we detect motor stopped
    static bool preloadFiredThisStop = false;    // ensure only one fire per stop
    static unsigned long preloadPulseStart = 0;  // for fast retract timing

    // Track motor state transitions
    if (lastMotorState && !isMoving) {
        motorStopTime = micros(); // Record when motor stopped
        machine.setBulbSystemReady(false); // System not ready when motor stops
        ramRetracted = false; // Ram needs to retract again

        // NEW: arm preloader for this stop
        preloadArmed = true;
        preloadFiredThisStop = false;
    }
    if (!lastMotorState && isMoving) {
        motorStartTime = micros(); // Record when motor started

        // NEW: disarm on movement; will re-arm at next stop
        preloadArmed = false;
    }
    lastMotorState = isMoving;

    // Read sensors
    bool ramHome = digitalRead(bulbRamHomeSensorPin); // HIGH if home
    bool bulbInPreload = digitalRead(bulbInPreLoadPosSensorPin); // HIGH if present
    bool bulbInCap = digitalRead(bulbInCapSensor);
    bool preLoadCylinderHome = digitalRead(preLoadCylinderHomeSensorPin);

    // ===================== NEW: Preloader one-shot =====================
    // Condition: machine stopped, allowed to preload, sensor HIGH, and not yet fired this stop
    if(!machine.canPreLoadBulbProcessStart()){
       machine.setBulbPreLoadReady(true); 
    }
    if (!isMoving && preloadArmed && !preloadFiredThisStop
        && machine.canPreLoadBulbProcessStart() && bulbInPreload) {
        // Extend preloader (fire) and immediately start retract timing
        machine.setBulbPreLoadReady(false);
        if(machine.inProduction && !slots[slotIdBulbPreLoad].hasError() && !slots[slotIdBulbPreLoad].shouldFinishProduction()){
            digitalWrite(bulbPreLoadCylinder, HIGH);
        }
        preloadPulseStart = micros();
        preloadFiredThisStop = true; // ensure only once per stop
    }

    // Fast retract: end the pulse as soon as we've met the minimum actuation time
    if (preloadFiredThisStop) {
        unsigned long stopDuration = micros() - motorStopTime;
        float pausePercent = (float)stopDuration / PAUSE_AFTER;
        //if (micros() - preloadPulseStart >= PRELOAD_PULSE_US) {
            if(pausePercent >= 0.95){
            digitalWrite(bulbPreLoadCylinder, LOW); // retract ASAP
            if(digitalRead(preLoadCylinderHomeSensorPin) == LOW){ //Is home
                machine.setBulbPreLoadReady(true);
            }
        }
    }
    // If it hasn’t fired yet this stop, keep preload "not ready"
if (!preloadFiredThisStop && machine.canPreLoadBulbProcessStart()) {
    machine.setBulbPreLoadReady(false);
}
    // ===================================================================

    if(machine.canBulbProcessStart()){
        if(!isMoving) {
            // Motor is stopped - handle timing based on pause duration
            unsigned long stopDuration = micros() - motorStopTime;
            float pausePercent = (float)stopDuration / PAUSE_AFTER;

            // Activate ram after 5% of pause time (only if bulb position sensor reads LOW)
            if (pausePercent >= 0.01 && pausePercent < 0.95 && !digitalRead(bulbRamPin) && bulbInCap) {
               // if(!slots[slotIdBulbInjection].hasMissingCap()){
                if(!slots[slotIdBulbInjection].hasError() && !slots[slotIdBulbInjection].shouldFinishProduction()){
                    digitalWrite(bulbRamPin, HIGH);
                }
                ramExtended = true;
                ramRetracted = false;
            }
            else if(pausePercent >= 0.01 && pausePercent < 0.95 && !digitalRead(bulbRamPin) && !bulbInCap){
                machine.bulbPresent = false;
                //TODO: DOES THIS ADD VALUE?
            }

            // Deactivate ram after 95% of pause time
            if (pausePercent >= 0.95 && digitalRead(bulbRamPin)) {
                digitalWrite(bulbRamPin, LOW);
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

    if(handleLowSupplies()){
        MIN_STEP_DELAY = 31 * 8;   // ≈ 248
        MAX_STEP_DELAY = 616 * 8;  // ≈ 4928
    }
    else{
        MIN_STEP_DELAY = 31;   // 40 * 0.77 ≈ 31
        MAX_STEP_DELAY = 616;  // 800 * 0.77 ≈ 616
    }
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
    //handleSpeedButton();
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
// ---- Config ----



// Optional smoothing/hysteresis
const uint8_t POT_EMA_ALPHA_NUM   = 1;  // EMA alpha = 1/8 (lower = smoother)
const uint8_t POT_EMA_ALPHA_DEN   = 8;
const unsigned long PAUSE_DEADBAND_US = 2000UL; // ignore tiny changes (<2 ms)

// Call this once per loop (or before you start a new index)
void updatePauseAfterFromPot() {
    // if(handleLowSupplies()){
    //     PAUSE_AFTER = PAUSE_LOW_SUPPLY;
    // }
    //else{
  static int ema = -1;  // -1 = uninitialized
  static unsigned long lastPause = PAUSE_AFTER;

  int raw = analogRead(speedPotPin);     // 0..1023

  // Exponential moving average to reduce jitter
  if (ema < 0) {
    ema = raw;
  } else {
    ema = (int)((long)ema * (POT_EMA_ALPHA_DEN - POT_EMA_ALPHA_NUM) +
                (long)raw * POT_EMA_ALPHA_NUM) / POT_EMA_ALPHA_DEN;
  }

  // Map pot to pause: higher pot -> smaller pause (faster)
  // If your knob works opposite, swap PAUSE_MIN_US and PAUSE_MAX_US in this formula.
  unsigned long span = (PAUSE_MAX_US - PAUSE_MIN_US);
  unsigned long mappedPause =
      PAUSE_MAX_US - ( (unsigned long)ema * span ) / 1023UL;

  // Clamp just in case
  if (mappedPause < PAUSE_MIN_US) mappedPause = PAUSE_MIN_US;
  if (mappedPause > PAUSE_MAX_US) mappedPause = PAUSE_MAX_US;

  // Apply only if change is meaningful (deadband)
  if ( (mappedPause > lastPause + PAUSE_DEADBAND_US) ||
       (mappedPause + PAUSE_DEADBAND_US < lastPause) ) {
    PAUSE_AFTER = mappedPause;
    lastPause = mappedPause;
    // Optional: debug
    // Serial.print("PAUSE_AFTER set to "); Serial.println(PAUSE_AFTER);
  }
    //}
}

void handleLowAirPressure(){ //When low air pressure is detected, pause machine and wait for start button
    if(digitalRead(lowAirSensorPin)== HIGH){
        machine.pause();
        machine.updateStatus(myNex, "Low Air - Pause");
    }
}

void setup() {
    //Serial.begin(115200);
     myNex.begin(115200); 
      
    // Initialize pins
    pinMode(speedPotPin, INPUT); 
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
    pinMode(preLoadCylinderHomeSensorPin, INPUT);
    //sensors
    pinMode(homeSensorPin, INPUT);
    pinMode(bulbRamHomeSensorPin, INPUT);
    pinMode(pipetTipSensor, INPUT);
    pinMode(bulbInCapSensor, INPUT);
    pinMode(capInWheel, INPUT);
    pinMode(slotEmptySensor, INPUT);
    pinMode(pipetSupplySensorPin, INPUT);
    pinMode(bulbSupplySensorPin, INPUT);
    pinMode(capSupplySensorPin, INPUT);

    digitalWrite(pipetTwisterPin, LOW);  // Start with twister off
    digitalWrite(bulbRamPin, LOW);
    digitalWrite(capInjectPin, LOW);
    digitalWrite(pipetRamPin, LOW);
    digitalWrite(junkEjectorPin, LOW);
    digitalWrite(bulbPreLoadCylinder, LOW);
    currentPipetState = PIPET_HOMING;
    updateSlotPositions();
    
}


int i = 0;

void loop() {
    // handleLowAirPressure();
    // updatePauseAfterFromPot(); 
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
    // if (machine.needsHoming) {
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



//if(handleLowSupplies()){
    if(handleLowSupplies()){
 // delay between steps
}
else{
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(10); // pulse width
        digitalWrite(stepPin, LOW);
        delayMicroseconds(4000);
}


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