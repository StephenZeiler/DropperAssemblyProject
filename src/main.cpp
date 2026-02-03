#include <Arduino.h>
#include "SlotObject.h"
#include "MachineState.h"
#include "EasyNextionLibrary.h"

MachineState machine;
EasyNex myNex(Serial2); // Create an object of EasyNex class with the name < myNex >
long startTime;

// ============================================================================
// TEENSY COMMUNICATION PINS - NEW ADDITIONS
// ============================================================================
const int teensyWheelReadyPin = 50;        // INPUT from Teensy - wheel ready (Teensy pin 26)
const int teensyOverrunAlarmPin = 23;      // INPUT from Teensy - overrun alarm (Teensy pin 12)
const int teensyWheelPosAlarmPin = 24;     // INPUT from Teensy - wheel pos alarm (Teensy pin 27)
const int teensyTrollHomePin = 27;         // OUTPUT - relay to Teensy when to troll home (Teensy pin 24)

// ============================================================================
// EXISTING PINS - Some repurposed for Teensy communication
// ============================================================================
// Pin 22 (stepPin) - NOW: Pulse to move wheel (Teensy pin 25)
// Pin 25 (homeSensorPin) - STAYS INPUT from physical sensor (unchanged!)
// Pin 27 (teensyTrollHomePin) - NEW: OUTPUT to Teensy to relay home status (Teensy pin 24)
// Pin 33 (bulbRamHomeSensorPin) - NOW: INPUT from Teensy (Teensy pin 11)
// Pin 39 (bulbRamPin) - NOW: OUTPUT to Teensy (Teensy pin 10) - logic unchanged!

// Motor pins - stepPin repurposed, others commented out
const int stepPin = 22;      // NOW sends pulse to Teensy instead of motor driver
// const int dirPin = 23;    // No longer needed
// const int enablePin = 24; // No longer needed

// Button pins (configured as INPUT, not INPUT_PULLUP)
const int startButtonPin = 10;
const int pauseButtonPin = 11;
const int finishProductionButtonPin = 12;
const int speedButtonPin = 8;
const int emptySlotsButtonPin = 9;

// Production Supply Indicators
const int pipetSupplySensorPin = 34; // Signal goes high when no pipet
const int bulbSupplySensorPin = 40;  // Signal goes high when no bulb
const int capSupplySensorPin = 38;   // Signal goes high when no cap
const int pipetLowSupplyLight = 42;
const int bulbLowSupplyLight = 44;
const int capLowSupplyLight = 46;
const int lowSupplyBuzzer = 48;
const int startUpBuzzerPin = 41;      // Safety buzzer for machine start warning

// Movement parameters - NO LONGER USED (Teensy handles this)
// const int TOTAL_STEPS = 200;
// int MIN_STEP_DELAY = 31;
// int MAX_STEP_DELAY = 616;
// int ACCEL_STEPS = 46;
// int DECEL_STEPS = 15;

unsigned long PAUSE_AFTER = 90000; // microseconds (keep same pause time)

// Potentiameter low and high range
const unsigned long PAUSE_MIN_US = 90000UL;  // fastest allowed pause
const unsigned long PAUSE_MAX_US = 400000UL; // slowest allowed pause
const unsigned long PAUSE_LOW_SUPPLY = 2000000UL;

// Potentiameter
const int speedPotPin = A1; // wire your pot wiper here

// Sensor
const int homeSensorPin = 25;        // NOW: OUTPUT to Teensy (was INPUT from sensor)
const int pipetTipSensor = 31;

// Motor state - simplified since Teensy handles movement
unsigned long pauseStartTime = 0;
bool isMoving = false;
bool pauseRequested = false;
bool emptySlotsRequested = false;
bool finsihProdRequested = false;

// Bulb system pins
const int bulbRamHomeSensorPin = 33; // NOW: INPUT from Teensy (was physical sensor)
const int bulbInPreLoadPosSensorPin = 26;    // HIGH = something blocking sensor
const int preLoadCylinderHomeSensorPin = 53; // HIGH = something blocking sensor, LOW when home

const int bulbPreLoadCylinder = 37;
const unsigned long PRELOAD_PULSE_US = 160000;
const int bulbRamPin = 39;           // NOW: OUTPUT to Teensy (logic unchanged!)
const int bulbInCapSensor = 30;

// ejection pin
const int dropperEjectPin = 47;

// junk ejector
const int junkEjectorPin = 49;

// CAP injection
const int capInjectPin = 35;
const int capInWheel = 29;
const int capPositionSensorPin = 52;

// Empty Slot Sensor
const int slotEmptySensor = 32;

// Pipet system pins
const int pipetRamPin = 43;
const int pipetTwisterPin = 45;
const int pipetTwisterHomeSensorPin = 28;

// Low Air Sensor
const int lowAirSensorPin = 2; // Reads HIGH when no air.

// Pipet system state
enum PipetState
{
    PIPET_HOMING,
    PIPET_HOMING_COMPLETE,
    PIPET_RAM_EXTENDING,
    PIPET_RAM_RETRACTING,
    PIPET_TWISTER_ACTIVE
};
PipetState currentPipetState = PIPET_HOMING;
unsigned long pipetStateStartTime = 0;
bool twisterAtHome = false;

// Dropper ejection system state
enum DropperState
{
    DROPPER_IDLE,
    DROPPER_EJECTING,
    DROPPER_RETRACTING
};
DropperState currentDropperState = DROPPER_IDLE;

enum CapState
{
    CAP_IDLE,
    CAP_INJECTING,
    CAP_RETRACTING
};
CapState currentCapState = CAP_IDLE;

unsigned long dropperStateStartTime = 0;

// Bulb system state
enum BulbState
{
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

int motorSpeedMode = 0;
int lastSw0Val = -1;
const uint8_t SW0_PAGE_ID = 0;
const uint8_t SW0_COMPONENT_ID = 10;

// Slot tracking
SlotObject slots[] = {
    SlotObject(0), SlotObject(1), SlotObject(2), SlotObject(3),
    SlotObject(4), SlotObject(5), SlotObject(6), SlotObject(7),
    SlotObject(8), SlotObject(9), SlotObject(10), SlotObject(11),
    SlotObject(12), SlotObject(13), SlotObject(14), SlotObject(15)};
int slotIdCapInWheelInjection;
int slotIdCapInWheelConfirm;
int slotIdBulbPreLoad;
int test;
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
    for (int i = 0; i < 16; i++)
    {
        if (slots[i].getPosition() == 1)
        {
            slotIdCapInWheelInjection = slots[i].getId();
        }
        if (slots[i].getPosition() == 2)
        {
            slotIdCapInWheelConfirm = slots[i].getId();
        }
        if (slots[i].getPosition() == 4)
        {
            test = slots[i].getId();
        }
        if (slots[i].getPosition() == 5)
        {
            slotIdBulbPreLoad = slots[i].getId();
        }
        if (slots[i].getPosition() == 6)
        {
            slotIdBulbInjection = slots[i].getId();
        }
        if (slots[i].getPosition() == 6)
        {
            slotIdBulbInCapConfirm = slots[i].getId();
        }
        if (slots[i].getPosition() == 9)
        {
            slotIdPipetInjection = slots[i].getId();
        }
        if (slots[i].getPosition() == 10)
        {
            slotIdPipetConfirm = slots[i].getId();
        }
        if (slots[i].getPosition() == 13)
        {
            slotIdDropeprEjection = slots[i].getId();
        }
        if (slots[i].getPosition() == 14)
        {
            slotIdJunkEjection = slots[i].getId();
        }
        if (slots[i].getPosition() == 15)
        {
            slotIdJunkConfirm = slots[i].getId();
        }
        if (slots[i].getPosition() == 0)
        {
            slotIdFailedJunkEject = slots[i].getId();
        }
    }
}

bool hasConsecutiveErrors() {
    int slotCount = sizeof(slots) / sizeof(slots[0]);
    for (int i = 0; i < slotCount; ++i) {
        int i1 = (i + 1) % slotCount;
        int i2 = (i + 2) % slotCount;

        bool threeMissingCaps =
            slots[i].hasMissingCap() &&
            slots[i1].hasMissingCap() &&
            slots[i2].hasMissingCap();

        bool threeMissingPipets =
            slots[i].hasJunk() &&
            slots[i1].hasJunk() &&
            slots[i2].hasJunk();

        bool threeMissingBulbs =
            slots[i].hasMissingBulb() &&
            slots[i1].hasMissingBulb() &&
            slots[i2].hasMissingBulb();

        if (threeMissingCaps || threeMissingPipets || threeMissingBulbs) {
            if (threeMissingCaps){
                 slots[i].setMissingCap(false);
                slots[i1].setMissingCap(false);
                slots[i2].setMissingCap(false);
                machine.hasConsecutiveCapErrors = true;
            }
            if (threeMissingPipets){
                 slots[i].setJunk(false);
                slots[i1].setJunk(false);
                slots[i2].setJunk(false);
                machine.hasConsecutivePipetErrors = true;
            }
            if (threeMissingBulbs){
                slots[i].setMissingBulb(false);
                slots[i1].setMissingBulb(false);
                slots[i2].setMissingBulb(false);
                machine.hasConsecutiveBulbErrors = true;
            }
                slots[i].setError(true);
                slots[i1].setError(true);
                slots[i2].setError(true);
            return true;
        }
    }
    machine.hasConsecutiveBulbErrors = false;
    machine.hasConsecutiveCapErrors = false;
    machine.hasConsecutivePipetErrors = false;
    return false;
}

void setSlotErrors(SlotObject slots[])
{
    for (int i = 0; i < 16; i++)
    {
        if (slots[i].hasJunk() || slots[i].hasMissingBulb() || slots[i].hasMissingCap())
        {
            slots[i].setError(true);
        }
    }
}

bool handleLowSupplies()
{
    if (digitalRead(capSupplySensorPin) == HIGH)
    {
        return true;
    }
    else if (digitalRead(bulbSupplySensorPin) == HIGH)
    {
        return true;
    }
    else if (digitalRead(pipetSupplySensorPin) == HIGH)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void handleSupplyAlert()
{
    bool capLow = (digitalRead(capSupplySensorPin) == HIGH);
    bool bulbLow = (digitalRead(bulbSupplySensorPin) == HIGH);
    bool pipetLow = (digitalRead(pipetSupplySensorPin) == HIGH);

    digitalWrite(capLowSupplyLight, capLow ? LOW : HIGH);
    digitalWrite(bulbLowSupplyLight, bulbLow ? LOW : HIGH);
    digitalWrite(pipetLowSupplyLight, pipetLow ? LOW : HIGH);

    bool anyLow = capLow || bulbLow || pipetLow;
    digitalWrite(lowSupplyBuzzer, anyLow ? LOW : HIGH);
}

int currentHomePosition = 0;

void handlePipetSystem()
{
    static bool lastMotorState = false;
    static bool homingComplete = false;
    static unsigned long motorStopTime = 0;
    static unsigned long motorStartTime = 0;
    bool twisterAtHome = digitalRead(pipetTwisterHomeSensorPin);

    if (!homingComplete)
    {
        if (!twisterAtHome)
        {
            digitalWrite(pipetTwisterPin, HIGH);
        }
        else
        {
            digitalWrite(pipetTwisterPin, LOW);
            homingComplete = true;
            currentPipetState = PIPET_HOMING_COMPLETE;
        }
        return;
    }

    if (lastMotorState && !isMoving)
    {
        motorStopTime = micros();
    }
    if (!lastMotorState && isMoving)
    {
        motorStartTime = micros();
    }
    lastMotorState = isMoving;

    if (currentPipetState == PIPET_HOMING_COMPLETE)
    {
        if (machine.canPipetProcessStart())
        {
            if (isMoving)
            {
                // Motor is moving - Teensy handles wheel movement now
                // Just activate twister at appropriate time
                unsigned long elapsedTime = micros() - motorStartTime;
                // Estimate 25% based on time (wheel takes ~1 second to move)
                if (elapsedTime >= 50000)  // 50ms
                {
                    digitalWrite(pipetTwisterPin, HIGH);
                }
                machine.setPipetSystemReady(false);
            }
            else
            {
                unsigned long stopDuration = micros() - motorStopTime;
                float pausePercent = (float)stopDuration / PAUSE_AFTER;

                if (pausePercent >= 0.00 && pausePercent < 0.90 && digitalRead(pipetRamPin) == LOW)
                {
                    if (!slots[slotIdPipetInjection].hasError() && !slots[slotIdPipetInjection].shouldFinishProduction())
                    {
                        digitalWrite(pipetRamPin, HIGH);
                    }
                }

                if (pausePercent >= 0.90)
                {
                    digitalWrite(pipetRamPin, LOW);
                    if (twisterAtHome)
                    {
                        machine.setPipetSystemReady(true);
                    }
                }

                if (pausePercent >= 0.75 && digitalRead(pipetTwisterPin) == HIGH && (!slots[slotIdPipetInjection].hasError() && !slots[slotIdPipetInjection].shouldFinishProduction()))
                {
                    digitalWrite(pipetTwisterPin, LOW);
                }
            }
            if (slots[slotIdPipetInjection].shouldFinishProduction() || slots[slotIdPipetInjection].hasError())
            {
                machine.setPipetSystemReady(true);
            }
        }
        else
        {
            digitalWrite(pipetTwisterPin, HIGH);
            machine.setPipetSystemReady(true);
        }
    }
}

void motorPauseTime()
{
    static bool lastMotorState = false;
    static unsigned long motorStopTime = 0;

    if (lastMotorState && !isMoving)
    {
        motorStopTime = micros();
    }
    lastMotorState = isMoving;

    if (!isMoving)
    {
        unsigned long stopDuration = micros() - motorStopTime;
        motorPausePercent = (float)stopDuration / PAUSE_AFTER;
    }
    else
    {
        motorPausePercent = 0;
    }
}

// ============================================================================
// handleBulbSystem() - UNCHANGED! Still uses pins 39 and 33
// ============================================================================
void handleBulbSystem()
{
    static bool lastMotorState = false;
    static unsigned long motorStopTime = 0;
    static unsigned long motorStartTime = 0;
    static bool ramExtended = false;
    static bool ramRetracted = true;

    static bool preloadArmed = false;
    static bool preloadFiredThisStop = false;
    static unsigned long preloadPulseStart = 0;

    if (lastMotorState && !isMoving)
    {
        motorStopTime = micros();
        ramRetracted = false;
        preloadArmed = true;
        preloadFiredThisStop = false;
    }
    if (!lastMotorState && isMoving)
    {
        motorStartTime = micros();
        machine.setBulbSystemReady(false);
        machine.setBulbPreLoadReady(false);
        preloadArmed = false;
    }
    lastMotorState = isMoving;

    bool ramHome = digitalRead(bulbRamHomeSensorPin);  // NOW reading from Teensy!
    bool bulbInPreload = digitalRead(bulbInPreLoadPosSensorPin);
    bool bulbInCap = digitalRead(bulbInCapSensor);
    bool preLoadCylinderHome = digitalRead(preLoadCylinderHomeSensorPin);

    if (!machine.canPreLoadBulbProcessStart())
    {
        machine.setBulbPreLoadReady(true);
    }
    if (!isMoving && preloadArmed && !preloadFiredThisStop && machine.canPreLoadBulbProcessStart() && bulbInPreload)
    {
        if (machine.inProduction && !slots[slotIdBulbPreLoad].hasError() && !slots[slotIdBulbPreLoad].shouldFinishProduction())
        {
            digitalWrite(bulbPreLoadCylinder, HIGH);
        }
        preloadPulseStart = micros();
        preloadFiredThisStop = true;
    }
    if (!isMoving && digitalRead(preLoadCylinderHomeSensorPin) == LOW && (slots[slotIdBulbPreLoad].hasError() || slots[slotIdBulbPreLoad].shouldFinishProduction()))
    {
        machine.setBulbPreLoadReady(true);
    }

    if (preloadFiredThisStop)
    {
        unsigned long stopDuration = micros() - motorStopTime;
        float pausePercent = (float)stopDuration / PAUSE_AFTER;
        if (pausePercent >= 0.95)
        {
            digitalWrite(bulbPreLoadCylinder, LOW);
            if (digitalRead(preLoadCylinderHomeSensorPin) == LOW)
            {
                machine.setBulbPreLoadReady(true);
            }
        }
    }

    if (machine.canBulbProcessStart())
    {
        if (!isMoving)
        {
            unsigned long stopDuration = micros() - motorStopTime;
            float pausePercent = (float)stopDuration / PAUSE_AFTER;

            if (pausePercent >= 0.00 && pausePercent < 0.95 && !digitalRead(bulbRamPin) && bulbInCap)
            {
                if (!slots[slotIdBulbInjection].hasError() && !slots[slotIdBulbInjection].shouldFinishProduction())
                {
                    digitalWrite(bulbRamPin, HIGH);  // Sends to Teensy now!
                }
                ramExtended = true;
                ramRetracted = false;
            }
            else if (pausePercent >= 0.01 && pausePercent < 0.95 && !digitalRead(bulbRamPin) && !bulbInCap)
            {
                machine.bulbPresent = false;
            }
            if (ramHome && (slots[slotIdBulbInjection].hasError() || slots[slotIdBulbInjection].shouldFinishProduction()))
            {
                machine.setBulbSystemReady(true);
            }

            if (pausePercent >= 0.95 && digitalRead(bulbRamPin))
            {
                digitalWrite(bulbRamPin, LOW);  // Sends to Teensy now!
            }

            if (ramExtended && ramHome && !digitalRead(bulbRamPin))
            {
                machine.setBulbSystemReady(true);
                ramExtended = false;
                ramRetracted = true;
            }
        }
    }
    else
    {
        machine.setBulbSystemReady(true);
    }
}

void updateSlotPositions()
{
    for (int i = 0; i < 16; i++)
    {
        int relativePos = (currentHomePosition + slots[i].getId()) % 16;
        slots[i].setPosition(relativePos);
    }
    machine.IncrementPositionsMoved();
}

bool shouldRunTracker = true;

void machineTracker()
{
    static bool lastMotorState = false;
    static unsigned long motorStopTime = 0;
    if (lastMotorState && !isMoving)
    {
        motorStopTime = micros();
    }
    lastMotorState = isMoving;
    unsigned long stopDuration = micros() - motorStopTime;

    if (!isMoving && shouldRunTracker)
    {
        shouldRunTracker = false;
        motorPauseTime();
        
        if (finsihProdRequested)
        {
            slots[slotIdFailedJunkEject].setFinsihProduction(true);
        }
        
        if (digitalRead(pipetTipSensor) == HIGH && machine.canPipetConfirmStart() && !slots[slotIdPipetConfirm].shouldFinishProduction() && !slots[slotIdPipetConfirm].hasError())
        {
            slots[slotIdPipetConfirm].setJunk(true);
        }
        else
        {
            slots[slotIdPipetConfirm].setJunk(false);
        }
        
        if (digitalRead(bulbInCapSensor) == LOW && machine.canBulbConfirmStart() && !slots[slotIdBulbInCapConfirm].shouldFinishProduction() && !slots[slotIdBulbInCapConfirm].hasError())
        {
            slots[slotIdBulbInCapConfirm].setMissingBulb(true);
        }
        else
        {
            slots[slotIdBulbInCapConfirm].setMissingBulb(false);
        }
        
        if (digitalRead(capInWheel) == LOW && machine.canCapConfirmStart() && !slots[slotIdCapInWheelConfirm].shouldFinishProduction() && !slots[slotIdCapInWheelConfirm].hasError())
        {
            slots[slotIdCapInWheelConfirm].setMissingCap(true);
        }
        else
        {
            slots[slotIdCapInWheelConfirm].setMissingCap(false);
        }

        if (machine.canJunkEjectionStart())
        {
            if (stopDuration < 18000 && !isMoving)
            {
                digitalWrite(junkEjectorPin, HIGH);
            }
            if (slots[slotIdJunkEjection].hasError())
            {
                machine.incrementErroredDroppers();
            }
        }

        if (machine.canDropperEjectionStart() && !slots[slotIdDropeprEjection].hasError() && !slots[slotIdDropeprEjection].shouldFinishProduction())
        {
            machine.incrementDroppersCompleted();
            if (stopDuration < 18000 && !isMoving)
            {
                digitalWrite(dropperEjectPin, HIGH);
            }
        }

        if (machine.canCheckForEmptyStart() && digitalRead(slotEmptySensor) == HIGH)
        {
            slots[slotIdJunkConfirm].setFailedJunkEject(true);
        }
        else if (machine.canCheckForEmptyStart() && digitalRead(slotEmptySensor) == LOW)
        {
            slots[slotIdJunkConfirm].setFailedJunkEject(false);
            slots[slotIdJunkConfirm].setJunk(false);
            slots[slotIdJunkConfirm].setMissingBulb(false);
            slots[slotIdJunkConfirm].setMissingCap(false);
            slots[slotIdJunkConfirm].setError(false);
        }
        
        setSlotErrors(slots);
        
        if (hasConsecutiveErrors())
        {
            machine.pause(junkEjectorPin, dropperEjectPin);
        }

        if (slots[slotIdFailedJunkEject].hasFailedJunkEject())
        {
            machine.pause(junkEjectorPin, dropperEjectPin);
        }
        
        if (slots[slotIdJunkEjection].shouldFinishProduction())
        {
            machine.pause(junkEjectorPin, dropperEjectPin);
        }
    }
    
    if (isMoving || machine.isStopped || machine.isPaused)
    {
        digitalWrite(junkEjectorPin, LOW);
        digitalWrite(dropperEjectPin, LOW);
    }
    
    if (stopDuration > 18000)
    {
        digitalWrite(junkEjectorPin, LOW);
        digitalWrite(dropperEjectPin, LOW);
    }
}

void handleCapInjection()
{
    if (machine.inProduction && !slots[slotIdFailedJunkEject].hasError() && !slots[slotIdFailedJunkEject].shouldFinishProduction())
    {
        digitalWrite(capInjectPin, HIGH);
    }
    else
    {
        digitalWrite(capInjectPin, LOW);
    }
    
    if(!isMoving && digitalRead(capPositionSensorPin) == HIGH ){
        machine.capInjectionReady = true;
    }
    
    if(slots[slotIdFailedJunkEject].hasError() || slots[slotIdFailedJunkEject].shouldFinishProduction()){
        machine.capInjectionReady = true;
    }
    
    if(isMoving){
        machine.capInjectionReady = false;
    }
}

// ============================================================================
// homeMachine() - MODIFIED to send signals to Teensy
// ============================================================================
void homeMachine()
{
    digitalWrite(capInjectPin, LOW);
    
    // Send HIGH to Teensy to start trolling home
    digitalWrite(teensyTrollHomePin, HIGH);  // Pin 27 -> Teensy pin 24
    
    // Wait for Teensy to signal that ram is home (pin 33 from Teensy pin 11)
    while (digitalRead(bulbRamHomeSensorPin) == LOW)
    {
        // Read physical home sensor (pin 25) and relay to Teensy via pin 27
        // When pin 25 is LOW = at home, send LOW to Teensy
        // When pin 25 is HIGH = not home, send HIGH to Teensy
        bool sensorValue = digitalRead(homeSensorPin);  // Read pin 25
        digitalWrite(teensyTrollHomePin, sensorValue);  // Send to Teensy via pin 27
        
        if (!digitalRead(pauseButtonPin))
        {
            digitalWrite(teensyTrollHomePin, LOW);  // Stop trolling
            machine.stop();
            machine.updateStatus(myNex, "Homing Stopped");
            return;
        }
        delay(10);
    }
    
    // Teensy has signaled home - stop trolling
    digitalWrite(teensyTrollHomePin, LOW);
    
    if (digitalRead(bulbRamHomeSensorPin) == HIGH)
    {
        machine.inProduction = true;
        digitalWrite(junkEjectorPin, LOW);
        digitalWrite(capInjectPin, HIGH);
        delay(500);
        digitalWrite(capInjectPin, LOW);
        delay(250);
    }

    currentHomePosition = 0;
    machine.homingComplete();
    isMoving = true;
    pauseStartTime = micros();
}

bool puasedStateProcessing = false;

// ============================================================================
// stepMotor() - MODIFIED to send pulse to Teensy
// ============================================================================
void stepMotor()
{
    unsigned long currentTime = micros();

    if (isMoving)
    {
        // Send pulse to Teensy to move wheel one slot
        digitalWrite(stepPin, HIGH);  // Pin 22 -> Teensy pin 25
        delay(10);  // Short pulse
        digitalWrite(stepPin, LOW);
        
        // Wait for Teensy to signal wheel is ready
        isMoving = false;
        unsigned long waitStart = millis();
        
        while (digitalRead(teensyWheelReadyPin) == LOW)  // Pin 50 <- Teensy pin 26
        {
            delay(10);
            
            // Timeout after 5 seconds
            if (millis() - waitStart > 5000)
            {
                machine.pause(junkEjectorPin, dropperEjectPin);
                machine.updateStatus(myNex, "Wheel Move Timeout");
                return;
            }
        }
        
        // Wheel has moved successfully
        shouldRunTracker = true;
        pauseStartTime = currentTime;
        machine.resetAllPneumatics();
        currentHomePosition = (currentHomePosition + 1) % 16;
        updateSlotPositions();
    }
    else
    {
        if (pauseRequested)
        {
            machine.pause(junkEjectorPin, dropperEjectPin);
            pauseRequested = false;
            return;
        }
        
        // Check if ready to move (now includes Teensy ready signals)
        bool teensyReady = (digitalRead(teensyWheelReadyPin) == HIGH) &&
                          (digitalRead(bulbRamHomeSensorPin) == HIGH);
        
        if (machine.isReadyToMove() && teensyReady)
        {
            // Additional safety check - confirm ram is home
            if (digitalRead(bulbRamHomeSensorPin))  // Reading from Teensy now
            {
                isMoving = true;
            }
        }
    }
}

void emptySlots()
{
    machine.updateStatus(myNex, "Emptying Slots");
    const unsigned long stepDelay = 4000;
    digitalWrite(pipetTwisterPin, LOW);
    
    int i = 0;
    while (i <= 3200)
    {
        if (!digitalRead(pauseButtonPin))
        {
            machine.stop();
            machine.updateStatus(myNex, "Emptying Stopped");
            break;
        }
        
        if (i % 200 == 0)
        {
            digitalWrite(junkEjectorPin, HIGH);
            delay(200);
            digitalWrite(junkEjectorPin, LOW);
            delay(200);
        }
        
        // Send pulse to Teensy to move wheel
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelay);
        ++i;
    }
    
    machine.updateStatus(myNex, "Emptying Completed");
    digitalWrite(junkEjectorPin, LOW);
    digitalWrite(pipetTwisterPin, HIGH);
    machine.stop();
}

void handleEmptySlots()
{
    if (!machine.inProduction && !digitalRead(emptySlotsButtonPin))
    {
        emptySlots();
    }
}

void handleButtons()
{
    static unsigned long lastDebounceTime = 0;
    const unsigned long debounceDelay = 50;
    static bool lastStartState = LOW;
    static bool lastPauseState = LOW;
    static bool lastFinishState = LOW;

    bool startState = digitalRead(startButtonPin);
    bool pauseState = digitalRead(pauseButtonPin);
    bool finishState = digitalRead(finishProductionButtonPin);

    if (millis() - lastDebounceTime < debounceDelay)
        return;
    
    handleEmptySlots();

    if (startState == LOW && lastStartState == HIGH)
    {
        lastDebounceTime = millis();

        // Safety buzzer - alert for 5 seconds before starting
        machine.updateStatus(myNex, "Starting in 5s...");
        digitalWrite(startUpBuzzerPin, HIGH);

        unsigned long buzzerStartTime = millis();
        bool startCancelled = false;

        // Sound buzzer for 5 seconds, but allow pause to cancel
        while (millis() - buzzerStartTime < 5000)
        {
            if (digitalRead(pauseButtonPin) == LOW)
            {
                startCancelled = true;
                machine.updateStatus(myNex, "Start Cancelled");
                break;
            }
            delay(10);
        }

        digitalWrite(startUpBuzzerPin, LOW);

        // Only start if not cancelled
        if (!startCancelled)
        {
            machine.start();
            pauseRequested = false;
            finsihProdRequested = false;
            machine.updateStatus(myNex, "In Production");
        }
    }

    if (pauseState == LOW && lastPauseState == HIGH && !machine.isStopped && !machine.isPaused)
    {
        machine.updateStatus(myNex, "Paused");
        lastDebounceTime = millis();
        pauseRequested = true;
    }

    if (finishState == LOW && lastFinishState == HIGH && !machine.isStopped)
    {
        if (machine.inProduction)
        {
            machine.updateStatus(myNex, "End Production");
        }
        lastDebounceTime = millis();
        finsihProdRequested = true;
    }

    lastStartState = startState;
    lastPauseState = pauseState;
    lastFinishState = finishState;
}

const uint8_t POT_EMA_ALPHA_NUM = 1;
const uint8_t POT_EMA_ALPHA_DEN = 8;
const unsigned long PAUSE_DEADBAND_US = 2000UL;

void updatePauseAfterFromPot()
{
    static int ema = -1;
    static unsigned long lastPause = PAUSE_AFTER;

    int raw = analogRead(speedPotPin);

    if (ema < 0)
    {
        ema = raw;
    }
    else
    {
        ema = (int)((long)ema * (POT_EMA_ALPHA_DEN - POT_EMA_ALPHA_NUM) +
                    (long)raw * POT_EMA_ALPHA_NUM) /
              POT_EMA_ALPHA_DEN;
    }

    unsigned long span = (PAUSE_MAX_US - PAUSE_MIN_US);
    unsigned long mappedPause =
        PAUSE_MAX_US - ((unsigned long)ema * span) / 1023UL;

    if (mappedPause < PAUSE_MIN_US)
        mappedPause = PAUSE_MIN_US;
    if (mappedPause > PAUSE_MAX_US)
        mappedPause = PAUSE_MAX_US;

    if ((mappedPause > lastPause + PAUSE_DEADBAND_US) ||
        (mappedPause + PAUSE_DEADBAND_US < lastPause))
    {
        PAUSE_AFTER = mappedPause;
        lastPause = mappedPause;
    }
}

void systemNotReadyTimeout() {
    static unsigned long notReadySince = 0;
    static bool tracking = false;

    const unsigned long TIMEOUT_MS = 2000UL;
    unsigned long now = millis();
    
    if(machine.isReadyToMove()){
        machine.timeoutMachine = false;
    }
    
    bool condition =
        (!isMoving) &&
        (!machine.isReadyToMove()) &&
        (!machine.isPaused) &&
        (!machine.isStopped);
        
    if (condition) {
        if (!tracking) {
            tracking = true;
            notReadySince = now;
        } else if (now - notReadySince >= TIMEOUT_MS) {
            machine.pause(junkEjectorPin, dropperEjectPin);
            tracking = false;
            machine.timeoutMachine = true;
        }
    } else {
        tracking = false;
    }
}

void handleLowAirPressure()
{
    if (digitalRead(lowAirSensorPin) == LOW)
    {
        machine.hasLowAirPressure = true;
        machine.pause(junkEjectorPin, dropperEjectPin);
        machine.updateStatus(myNex, "Low Air - Pause");
    }
    else{
        machine.hasLowAirPressure = false;
    }
}

// ============================================================================
// NEW FUNCTION: Handle Teensy Alarms
// ============================================================================
void handleTeensyAlarms() {
    static bool lastOverrunAlarm = LOW;
    static bool lastWheelPosAlarm = LOW;
    
    bool currentOverrunAlarm = digitalRead(teensyOverrunAlarmPin);
    bool currentWheelPosAlarm = digitalRead(teensyWheelPosAlarmPin);
    
    // Check for RAM OVERRUN alarm (rising edge)
    if (currentOverrunAlarm == HIGH && lastOverrunAlarm == LOW) {
        machine.pause(junkEjectorPin, dropperEjectPin);
        machine.updateStatus(myNex, "RAM OVERRUN ERROR");
        machine.hasTeensyRamError = true;  // Set flag for logging
    }
    
    // Check for WHEEL POSITION alarm (rising edge)
    if (currentWheelPosAlarm == HIGH && lastWheelPosAlarm == LOW) {
        machine.pause(junkEjectorPin, dropperEjectPin);
        machine.updateStatus(myNex, "WHEEL POSITION ERROR");
        machine.hasTeensyWheelError = true;  // Set flag for logging
    }
    
    lastOverrunAlarm = currentOverrunAlarm;
    lastWheelPosAlarm = currentWheelPosAlarm;
}

void setup()
{
    myNex.begin(115200);

    // Initialize pins
    pinMode(speedPotPin, INPUT);
    pinMode(stepPin, OUTPUT);         // Now sends pulse to Teensy
    // pinMode(dirPin, OUTPUT);       // No longer needed
    // pinMode(enablePin, OUTPUT);    // No longer needed

    pinMode(startButtonPin, INPUT_PULLUP);
    pinMode(pauseButtonPin, INPUT_PULLUP);
    pinMode(finishProductionButtonPin, INPUT_PULLUP);
    pinMode(emptySlotsButtonPin, INPUT_PULLUP);
    pinMode(speedButtonPin, INPUT_PULLUP);

    // digitalWrite(enablePin, LOW);  // No longer needed
    // digitalWrite(dirPin, LOW);     // No longer needed
    digitalWrite(stepPin, LOW);
    digitalWrite(dropperEjectPin, LOW);

    // ========================================================================
    // Pin 25 stays INPUT - reads physical home sensor (NO CHANGE!)
    // ========================================================================
    pinMode(homeSensorPin, INPUT);
    
    // ========================================================================
    // NEW: Pin 27 OUTPUT to relay home status to Teensy
    // ========================================================================
    pinMode(teensyTrollHomePin, OUTPUT);
    digitalWrite(teensyTrollHomePin, LOW);
    
    // ========================================================================
    // NEW: Teensy communication pins
    // ========================================================================
    pinMode(teensyWheelReadyPin, INPUT);
    pinMode(teensyOverrunAlarmPin, INPUT);
    pinMode(teensyWheelPosAlarmPin, INPUT);

    // Pneumatics
    delay(100);
    pinMode(bulbRamPin, OUTPUT);           // Still OUTPUT (now to Teensy)
    pinMode(dropperEjectPin, OUTPUT);
    pinMode(pipetTwisterPin, OUTPUT);
    pinMode(pipetRamPin, OUTPUT);
    pinMode(capInjectPin, OUTPUT);
    pinMode(junkEjectorPin, OUTPUT);
    pinMode(pipetTwisterHomeSensorPin, INPUT);
    pinMode(preLoadCylinderHomeSensorPin, INPUT);
    
    // sensors
    pinMode(bulbRamHomeSensorPin, INPUT);  // Still INPUT (now from Teensy)
    pinMode(pipetTipSensor, INPUT);
    pinMode(bulbInCapSensor, INPUT);
    pinMode(capInWheel, INPUT);
    pinMode(slotEmptySensor, INPUT);
    pinMode(pipetSupplySensorPin, INPUT);
    pinMode(bulbSupplySensorPin, INPUT);
    pinMode(capSupplySensorPin, INPUT);
    pinMode(capPositionSensorPin, INPUT);
    
    pinMode(capLowSupplyLight, OUTPUT);
    pinMode(bulbLowSupplyLight, OUTPUT);
    pinMode(pipetLowSupplyLight, OUTPUT);
    pinMode(lowSupplyBuzzer, OUTPUT);
    pinMode(startUpBuzzerPin, OUTPUT);
    digitalWrite(startUpBuzzerPin, LOW);

    digitalWrite(pipetTwisterPin, LOW);
    digitalWrite(bulbRamPin, LOW);
    digitalWrite(capInjectPin, LOW);
    digitalWrite(pipetRamPin, LOW);
    digitalWrite(junkEjectorPin, LOW);
    digitalWrite(bulbPreLoadCylinder, LOW);
    
    currentPipetState = PIPET_HOMING;
    updateSlotPositions();
}

int i = 0;

void loop()
{
    handleLowAirPressure();
    handleTeensyAlarms();        // NEW: Monitor Teensy alarm signals
    updatePauseAfterFromPot();
    handleButtons();
    handleSupplyAlert();
    setSlotIdByPosition(slots);
    
    startTime = millis();
    motorPauseTime();
    
    if ((!isMoving && motorPausePercent > .90) || machine.isPaused)
    {
        machine.updateMachineDisplayInfo(myNex, startTime, slots);
    }
    
    machineTracker();
    handleCapInjection();
    handleBulbSystem();      // UNCHANGED - still uses pins 39 and 33
    handlePipetSystem();
    systemNotReadyTimeout();

    if (machine.isStopped)
        return;
        
    if (machine.needsHoming)
    {
        if (machine.needsHoming)
        {
            machine.updateStatus(myNex, "Motor Homing");
            homeMachine();
        }
        if (!machine.needsHoming && !machine.isPaused && !machine.isStopped)
        {
            machine.updateStatus(myNex, "In Production");
        }
        return;
    }

    if (machine.inProduction)
    {
        stepMotor();
    }
}
