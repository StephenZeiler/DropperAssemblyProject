#include <Arduino.h>
#include "SlotObject.h"
#include "MachineState.h"
#include "EasyNextionLibrary.h"

// ============================================================================
// DEBUG LOGGING - Set to 1 to enable Serial debug output, 0 to disable
// ============================================================================
#define DEBUG_MODE 1

#if DEBUG_MODE
  #define DBG_INIT() Serial.begin(115200); while(!Serial && millis() < 3000){}
  #define DBG(x) Serial.print(F(x))
  #define DBG_VAL(x) Serial.print(x)
  #define DBGLN(x) Serial.println(F(x))
  #define DBGLN_VAL(x) Serial.println(x)
#else
  #define DBG_INIT()
  #define DBG(x)
  #define DBG_VAL(x)
  #define DBGLN(x)
  #define DBGLN_VAL(x)
#endif

MachineState machine;
EasyNex myNex(Serial2); // Create an object of EasyNex class with the name < myNex >
long startTime;

// ============================================================================
// TEENSY COMMUNICATION PINS - NEW ADDITIONS
// ============================================================================
const int teensyWheelReadyPin = 50;        // INPUT from Teensy - wheel ready (Teensy pin 26)
const int teensyOverrunAlarmPin = 23;      // INPUT from Teensy - overrun alarm (Teensy pin 12)
const int teensyTrollHomePin = 27;         // OUTPUT - relay to Teensy when to troll home (Teensy pin 24)
const int wheelPositionSensorPin = 18;     // INPUT - wheel position sensor (moved from Teensy pin 31)

// ============================================================================
// EXISTING PINS - Some repurposed for Teensy communication
// ============================================================================
// Pin 18 (wheelPositionSensorPin) - INPUT from wheel position sensor (moved from Teensy pin 31)
// Pin 22 (stepPin) - NOW: Pulse to move wheel (Teensy pin 25)
// Pin 25 (homeSensorPin) - STAYS INPUT from physical sensor (unchanged!)
// Pin 27 (teensyTrollHomePin) - OUTPUT to Teensy to relay home status (Teensy pin 24)
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
unsigned long motorStopTime = 0;  // Synchronized with pauseStartTime for accurate ejector timing
bool isMoving = false;
bool pauseRequested = false;
bool emptySlotsRequested = false;
bool finsihProdRequested = false;
int finishProdEjectCount = 0;

// Wheel movement state machine for non-blocking Teensy control
enum WheelMoveState
{
    WHEEL_IDLE,
    WHEEL_PULSE_SENT,
    WHEEL_WAITING_FOR_READY
};
WheelMoveState wheelState = WHEEL_IDLE;
unsigned long wheelPulseSentTime = 0;
const unsigned long WHEEL_TIMEOUT_MS = 5000;

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

void printFinishProdSlotStatus() {
    DBGLN("=== FINISH PROD - SLOT STATUS ===");
    for (int i = 0; i < 16; i++) {
        DBG("Slot "); DBG_VAL(slots[i].getId());
        DBG(" pos="); DBG_VAL(slots[i].getPosition());
        DBG(" err="); DBG_VAL(slots[i].hasError());
        DBG(" junk="); DBG_VAL(slots[i].hasJunk());
        DBG(" noBulb="); DBG_VAL(slots[i].hasMissingBulb());
        DBG(" noCap="); DBG_VAL(slots[i].hasMissingCap());
        DBG(" junkFail="); DBG_VAL(slots[i].hasFailedJunkEject());
        DBG(" finProd="); DBGLN_VAL(slots[i].shouldFinishProduction());
    }
    DBG("Ejecting slot (pos 13): "); DBGLN_VAL(slotIdDropeprEjection);
    DBG("Junk eject slot (pos 14): "); DBGLN_VAL(slotIdJunkEjection);
    DBG("Ejected since end prod: "); DBGLN_VAL(finishProdEjectCount);
    DBGLN("=================================");
}

bool handleLowSupplies()
{
    static unsigned long lowSinceTime = 0;
    static bool wasLow = false;
    const unsigned long DEBOUNCE_MS = 500; // sensor must read low-supply for 0.5s

    bool anyLow = (digitalRead(capSupplySensorPin) == HIGH) ||
                  (digitalRead(bulbSupplySensorPin) == HIGH) ||
                  (digitalRead(pipetSupplySensorPin) == HIGH);

    if (anyLow) {
        if (!wasLow) {
            lowSinceTime = millis();
            wasLow = true;
        }
        return (millis() - lowSinceTime >= DEBOUNCE_MS);
    } else {
        wasLow = false;
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
    static bool lastPipetReady = true;
    static bool lastRamState = false;
    static bool lastTwisterState = false;
    bool twisterAtHome = digitalRead(pipetTwisterHomeSensorPin);

    if (!homingComplete)
    {
        if (!twisterAtHome)
        {
            digitalWrite(pipetTwisterPin, HIGH);
        }
        else
        {
            DBGLN("[PIPET] Homing complete");
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
                if (elapsedTime >= 50000)  // 0.25 seconds
                {
                    if (digitalRead(pipetTwisterPin) == LOW) {
                        DBGLN("[PIPET] Twister ON (during move)");
                    }
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
                        DBG("[PIPET] RAM FIRING - pause%="); DBGLN_VAL(pausePercent);
                        digitalWrite(pipetRamPin, HIGH);
                    }
                }

                if (pausePercent >= 0.90)
                {
                    if (digitalRead(pipetRamPin) == HIGH) {
                        DBGLN("[PIPET] RAM RETRACTING (pause >= 90%)");
                    }
                    digitalWrite(pipetRamPin, LOW);
                    if (twisterAtHome)
                    {
                        machine.setPipetSystemReady(true);
                    }
                }

                if (pausePercent >= 0.75 && digitalRead(pipetTwisterPin) == HIGH && (!slots[slotIdPipetInjection].hasError() && !slots[slotIdPipetInjection].shouldFinishProduction()))
                {
                    DBGLN("[PIPET] Twister OFF (pause >= 75%)");
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

    // Log state changes
    if (machine.pipetSystemReady != lastPipetReady) {
        DBG("[PIPET] pipetSystemReady: "); DBGLN_VAL(machine.pipetSystemReady);
        lastPipetReady = machine.pipetSystemReady;
    }
    bool currentRam = digitalRead(pipetRamPin);
    if (currentRam != lastRamState) {
        DBG("[PIPET] ramPin: "); DBGLN_VAL(currentRam);
        lastRamState = currentRam;
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

    // Track state changes for logging
    static bool lastRamExtended = false;
    static bool lastBulbReady = true;
    static bool lastPreloadReady = true;

    if (lastMotorState && !isMoving)
    {
        DBGLN("[BULB] Motor stopped - arming preload");
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
            DBGLN("[BULB] PRELOAD CYLINDER FIRING");
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
            if (digitalRead(bulbPreLoadCylinder) == HIGH) {
                DBGLN("[BULB] PRELOAD CYLINDER RETRACTING");
            }
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
                    DBG("[BULB] RAM FIRING - bulbInCap="); DBG_VAL(bulbInCap);
                    DBG(" ramHome="); DBG_VAL(ramHome);
                    DBG(" pause%="); DBGLN_VAL(pausePercent);
                    digitalWrite(bulbRamPin, HIGH);  // Sends to Teensy now!
                }
                ramExtended = true;
                ramRetracted = false;
            }
            else if (pausePercent >= 0.01 && pausePercent < 0.95 && !digitalRead(bulbRamPin) && !bulbInCap)
            {
                if (machine.bulbPresent) {
                    DBGLN("[BULB] No bulb in cap detected!");
                }
                machine.bulbPresent = false;
            }
            if (ramHome && (slots[slotIdBulbInjection].hasError() || slots[slotIdBulbInjection].shouldFinishProduction()))
            {
                machine.setBulbSystemReady(true);
            }

            if (pausePercent >= 0.95 && digitalRead(bulbRamPin))
            {
                DBGLN("[BULB] RAM RETRACTING (pause >= 95%)");
                digitalWrite(bulbRamPin, LOW);  // Sends to Teensy now!
            }

            if (ramExtended && ramHome && !digitalRead(bulbRamPin))
            {
                if (!machine.bulbSystemReady) {
                    DBGLN("[BULB] Ram cycle complete - system ready");
                }
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

    // Log state changes
    if (ramExtended != lastRamExtended) {
        DBG("[BULB] ramExtended: "); DBGLN_VAL(ramExtended);
        lastRamExtended = ramExtended;
    }
    if (machine.bulbSystemReady != lastBulbReady) {
        DBG("[BULB] bulbSystemReady: "); DBGLN_VAL(machine.bulbSystemReady);
        lastBulbReady = machine.bulbSystemReady;
    }
    if (machine.bulbPreLoadReady != lastPreloadReady) {
        DBG("[BULB] bulbPreLoadReady: "); DBGLN_VAL(machine.bulbPreLoadReady);
        lastPreloadReady = machine.bulbPreLoadReady;
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
            if (finsihProdRequested)
            {
                finishProdEjectCount++;
            }
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
            // Print final status before resetting
            printFinishProdSlotStatus();

            machine.stop();
            machine.updateStatus(myNex, "End Prod Complete");

            // Reset all finish production flags so machine can start fresh
            for (int i = 0; i < 16; i++) {
                slots[i].setFinsihProduction(false);
            }
            finsihProdRequested = false;
            finishProdEjectCount = 0;
        }

        if (finsihProdRequested)
        {
            printFinishProdSlotStatus();
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
    static bool lastCapReady = true;

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

    // Log state changes
    if (machine.capInjectionReady != lastCapReady) {
        DBG("[CAP] capInjectionReady: "); DBGLN_VAL(machine.capInjectionReady);
        lastCapReady = machine.capInjectionReady;
    }
}

// ============================================================================
// homeMachine() - MODIFIED to send signals to Teensy
// ============================================================================
void homeMachine()
{
    DBGLN("[HOME] Starting homing sequence");
    digitalWrite(capInjectPin, LOW);

    // Send HIGH to Teensy to start trolling home
    DBGLN("[HOME] Sending troll home signal to Teensy");
    digitalWrite(teensyTrollHomePin, HIGH);  // Pin 27 -> Teensy pin 24

    // Wait for Teensy to finish both ram homing and wheel trolling (pin 50 from Teensy pin 26)
    DBGLN("[HOME] Waiting for Teensy ready signal...");
    unsigned long homeStartTime = millis();
    while (digitalRead(teensyWheelReadyPin) == LOW)
    {
        // Read physical home sensor (pin 25) and relay to Teensy via pin 27
        // Teensy uses this during trollWheelToHome() to know when wheel is at home
        bool sensorValue = digitalRead(homeSensorPin);  // Read pin 25
        digitalWrite(teensyTrollHomePin, sensorValue);  // Send to Teensy via pin 27

        if (!digitalRead(pauseButtonPin))
        {
            DBGLN("[HOME] Homing cancelled by user");
            digitalWrite(teensyTrollHomePin, LOW);  // Stop trolling
            machine.stop();
            machine.updateStatus(myNex, "Homing Stopped");
            return;
        }

        // Log progress every second
        static unsigned long lastHomeLog = 0;
        if (millis() - lastHomeLog > 1000) {
            DBG("[HOME] Still homing... homeSensor="); DBG_VAL(sensorValue);
            DBG(" elapsed="); DBG_VAL((millis() - homeStartTime) / 1000); DBGLN("s");
            lastHomeLog = millis();
        }
        delay(10);
    }

    // Teensy has signaled both motors homed - stop relaying
    digitalWrite(teensyTrollHomePin, LOW);
    DBG("[HOME] Teensy ready! Homing took "); DBG_VAL((millis() - homeStartTime)); DBGLN("ms");

    if (digitalRead(teensyWheelReadyPin) == HIGH)
    {
        DBGLN("[HOME] Initializing production state");
        machine.inProduction = true;
        digitalWrite(junkEjectorPin, LOW);
        digitalWrite(capInjectPin, HIGH);
        delay(500);
        digitalWrite(capInjectPin, LOW);
        delay(250);
    }

    currentHomePosition = 0;
    machine.homingComplete();
    isMoving = false;  // Not moving after homing - ready to start pause cycle
    pauseStartTime = micros();
    motorStopTime = pauseStartTime;  // Sync for accurate ejector timing
    DBGLN("[HOME] Homing complete! Ready for production");
}

bool puasedStateProcessing = false;

// ============================================================================
// stepMotor() - MODIFIED to send pulse to Teensy
// ============================================================================
void stepMotor()
{
    static unsigned long movePulseTime = 0;      // Stopwatch: when pulse was sent
    static bool waitingForAck = false;            // Waiting for Teensy to pull pin LOW
    static bool sawSensorHigh = false;            // Tracks if wheel left previous slot during move
    static bool lastIsMoving = false;             // For state change detection
    unsigned long currentTime = micros();

    // Log state transitions
    if (isMoving != lastIsMoving) {
        DBG("[STEP] isMoving: "); DBG_VAL(lastIsMoving); DBG(" -> "); DBGLN_VAL(isMoving);
        lastIsMoving = isMoving;
    }

    if (isMoving)
    {
        // First iteration after isMoving=true: send pulse and start stopwatch
        if (movePulseTime == 0)
        {
            DBGLN("[STEP] Sending pulse to Teensy");
            digitalWrite(stepPin, HIGH);  // Send pulse to Teensy
            delay(10);
            digitalWrite(stepPin, LOW);
            movePulseTime = micros();  // Start stopwatch
            waitingForAck = true;      // Wait for Teensy to acknowledge (pin LOW)
            sawSensorHigh = false;     // Reset - must see HIGH during this move
            DBG("[STEP] Pulse sent, waiting for ACK. WheelReady="); DBGLN_VAL(digitalRead(teensyWheelReadyPin));
        }

        // Poll wheel position sensor throughout the move to confirm the wheel left the slot
        if (digitalRead(wheelPositionSensorPin) == HIGH)
        {
            if (!sawSensorHigh) {
                DBGLN("[STEP] Wheel pos sensor HIGH (wheel left slot)");
            }
            sawSensorHigh = true;
        }

        // Wait for Teensy to acknowledge by pulling ready pin LOW
        if (waitingForAck)
        {
            if (digitalRead(teensyWheelReadyPin) == LOW)
            {
                DBGLN("[STEP] Teensy ACK received (pin LOW)");
                waitingForAck = false;  // Teensy acknowledged, now wait for HIGH
            }
            // Check for timeout while waiting for acknowledgment
            unsigned long elapsed = micros() - movePulseTime;
            if (elapsed > 5000000)
            {
                DBGLN("[STEP] ERROR: Teensy ACK timeout!");
                isMoving = false;
                movePulseTime = 0;
                waitingForAck = false;
                sawSensorHigh = false;
                machine.pause(junkEjectorPin, dropperEjectPin);
                machine.updateStatus(myNex, "Teensy No ACK");
                return;
            }
            return;  // Don't check for HIGH until we get LOW
        }

        // Now wait for Teensy to finish (wheelReady goes HIGH)
        if (digitalRead(teensyWheelReadyPin) == HIGH)
        {
            DBGLN("[STEP] Teensy ready HIGH (move complete signal)");

            // Verify the wheel actually left the previous slot during the move
            if (!sawSensorHigh)
            {
                DBGLN("[STEP] ERROR: Never saw wheel leave slot!");
                isMoving = false;
                movePulseTime = 0;
                waitingForAck = false;
                sawSensorHigh = false;
                machine.pause(junkEjectorPin, dropperEjectPin);
                machine.updateStatus(myNex, "WHEEL DID NOT MOVE");
                machine.hasWheelPositionError = true;
                return;
            }

            // Move complete - check wheel position sensor (LOW = correct position)
            if (digitalRead(wheelPositionSensorPin) == HIGH)
            {
                DBGLN("[STEP] ERROR: Wheel pos sensor still HIGH after move!");
                // Wheel not in correct position
                isMoving = false;
                movePulseTime = 0;
                waitingForAck = false;
                sawSensorHigh = false;
                machine.pause(junkEjectorPin, dropperEjectPin);
                machine.updateStatus(myNex, "WHEEL POSITION ERROR");
                machine.hasWheelPositionError = true;
                return;
            }

            DBG("[STEP] Move OK! Pos "); DBG_VAL(currentHomePosition); DBG(" -> "); DBGLN_VAL((currentHomePosition + 1) % 16);
            isMoving = false;
            shouldRunTracker = true;
            pauseStartTime = currentTime;
            motorStopTime = currentTime;
            machine.resetAllPneumatics();
            currentHomePosition = (currentHomePosition + 1) % 16;
            updateSlotPositions();
            movePulseTime = 0;
            waitingForAck = false;
            sawSensorHigh = false;
            return;
        }

        // Timeout check (5 seconds) - only if move hasn't completed
        unsigned long elapsed = micros() - movePulseTime;
        if (elapsed > 5000000)
        {
            DBGLN("[STEP] ERROR: Move timeout (5s)!");
            isMoving = false;
            movePulseTime = 0;
            sawSensorHigh = false;
            machine.pause(junkEjectorPin, dropperEjectPin);
            machine.updateStatus(myNex, "Wheel Move Timeout");
        }
    }
    else
    {
        // Not moving - check if should start
        if (pauseRequested)
        {
            DBGLN("[STEP] Pause requested");
            machine.pause(junkEjectorPin, dropperEjectPin);
            pauseRequested = false;
            return;
        }

        // Only start move if Teensy is ready (wheelReady HIGH)
        bool teensyWheelReady = digitalRead(teensyWheelReadyPin) == HIGH;
        bool bulbRamHome = digitalRead(bulbRamHomeSensorPin) == HIGH;
        bool teensyReady = teensyWheelReady && bulbRamHome;
        bool machineReady = machine.isReadyToMove();
        bool pauseElapsed = (currentTime - pauseStartTime >= PAUSE_AFTER);

        // Log why we can't move (only periodically to avoid spam)
        static unsigned long lastNotReadyLog = 0;
        if (!machineReady || !teensyReady || !pauseElapsed) {
            if (currentTime - lastNotReadyLog > 1000000) { // Log every 1 second max
                lastNotReadyLog = currentTime;
                if (!machineReady) {
                    DBG("[STEP] NOT READY - Machine: bulb="); DBG_VAL(machine.bulbSystemReady);
                    DBG(" cap="); DBG_VAL(machine.capInjectionReady);
                    DBG(" preload="); DBG_VAL(machine.bulbPreLoadReady);
                    DBG(" pipet="); DBGLN_VAL(machine.pipetSystemReady);
                }
                if (!teensyWheelReady) {
                    DBGLN("[STEP] NOT READY - Teensy wheel not ready (pin LOW)");
                }
                if (!bulbRamHome) {
                    DBGLN("[STEP] NOT READY - Bulb ram not home");
                }
                if (!pauseElapsed) {
                    unsigned long remaining = (PAUSE_AFTER - (currentTime - pauseStartTime)) / 1000;
                    DBG("[STEP] NOT READY - Pause remaining: "); DBG_VAL(remaining); DBGLN("ms");
                }
            }
        }

        if (machineReady && teensyReady && pauseElapsed)
        {
            if (bulbRamHome)
            {
                // If supplies are low, delay and count cycles
                static int lowSupplyCycleCount = 0;
                if (handleLowSupplies()) {
                    lowSupplyCycleCount++;
                    machine.hasLowSupplySlowing = true;
                    if (lowSupplyCycleCount >= 8) {
                        DBGLN("[STEP] LOW SUPPLY for 8 cycles - pausing machine");
                        machine.hasLowSupplyPaused = true;
                        machine.hasLowSupplySlowing = false;
                        machine.pause(junkEjectorPin, dropperEjectPin);
                        machine.updateStatus(myNex, "Low Supply - Pause");
                        lowSupplyCycleCount = 0;
                        return;
                    }
                    DBGLN("[STEP] LOW SUPPLY - delaying 2s before move");
                    delay(2000);
                } else {
                    lowSupplyCycleCount = 0;
                    machine.hasLowSupplySlowing = false;
                    machine.hasLowSupplyPaused = false;
                }
                DBGLN("[STEP] Starting move!");
                isMoving = true;
                movePulseTime = 0;  // Will send pulse on next iteration
            }
        }
    }
}

void emptySlots()
{
    machine.updateStatus(myNex, "Emptying Slots");
    digitalWrite(pipetTwisterPin, LOW);

    const int SLOTS_PER_REVOLUTION = 16;
    const unsigned long TEENSY_TIMEOUT = 5000;  // 5 second timeout per slot

    for (int slot = 0; slot < SLOTS_PER_REVOLUTION; slot++)
    {
        if (!digitalRead(pauseButtonPin))
        {
            machine.stop();
            machine.updateStatus(myNex, "Emptying Stopped");
            break;
        }

        // Send pulse to Teensy to move one slot
        digitalWrite(stepPin, HIGH);
        delay(10);
        digitalWrite(stepPin, LOW);

        // Wait for Teensy ACK (teensyWheelReadyPin goes LOW)
        unsigned long startTime = millis();
        while (digitalRead(teensyWheelReadyPin) == HIGH)
        {
            if (millis() - startTime > TEENSY_TIMEOUT)
            {
                machine.updateStatus(myNex, "Empty Timeout ACK");
                machine.stop();
                return;
            }
        }

        // Wait for Teensy complete (teensyWheelReadyPin goes HIGH)
        startTime = millis();
        while (digitalRead(teensyWheelReadyPin) == LOW)
        {
            if (millis() - startTime > TEENSY_TIMEOUT)
            {
                machine.updateStatus(myNex, "Empty Timeout Done");
                machine.stop();
                return;
            }
        }

        // Slot move complete - fire junk ejector
        digitalWrite(junkEjectorPin, HIGH);
        delay(200);
        digitalWrite(junkEjectorPin, LOW);
        delay(200);
    }

    machine.updateStatus(myNex, "Emptying Completed");
    digitalWrite(junkEjectorPin, LOW);
    digitalWrite(pipetTwisterPin, HIGH);
    machine.stop();
}

void handleEmptySlots()
{
    // Only allow empty slots when Teensy is ready (HIGH) and button pressed
    if (!machine.inProduction &&
        !digitalRead(emptySlotsButtonPin) &&
        digitalRead(teensyWheelReadyPin) == HIGH)
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

        // Safety buzzer - alert for 3 seconds before starting
        machine.updateStatus(myNex, "Starting in 3s...");
        digitalWrite(startUpBuzzerPin, HIGH);

        unsigned long buzzerStartTime = millis();
        bool startCancelled = false;

        // Sound buzzer for 3 seconds, but allow pause to cancel
        while (millis() - buzzerStartTime < 3000)
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
            finishProdEjectCount = 0;
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
            DBG("[TIMEOUT] Started tracking not-ready state. bulb="); DBG_VAL(machine.bulbSystemReady);
            DBG(" cap="); DBG_VAL(machine.capInjectionReady);
            DBG(" preload="); DBG_VAL(machine.bulbPreLoadReady);
            DBG(" pipet="); DBGLN_VAL(machine.pipetSystemReady);
        } else if (now - notReadySince >= TIMEOUT_MS) {
            DBGLN("[TIMEOUT] TIMEOUT! Pausing machine - subsystem not ready for 2s");
            DBG("[TIMEOUT] Final state: bulb="); DBG_VAL(machine.bulbSystemReady);
            DBG(" cap="); DBG_VAL(machine.capInjectionReady);
            DBG(" preload="); DBG_VAL(machine.bulbPreLoadReady);
            DBG(" pipet="); DBGLN_VAL(machine.pipetSystemReady);
            machine.pause(junkEjectorPin, dropperEjectPin);
            tracking = false;
            machine.timeoutMachine = true;
        }
    } else {
        if (tracking) {
            DBGLN("[TIMEOUT] Tracking stopped - system ready");
        }
        tracking = false;
    }
}

void handleLowAirPressure()
{
    static bool lastLowAir = false;

    if (digitalRead(lowAirSensorPin) == HIGH)
    {
        if (!lastLowAir) {
            DBGLN("[AIR] LOW AIR PRESSURE DETECTED!");
        }
        machine.hasLowAirPressure = true;
        machine.pause(junkEjectorPin, dropperEjectPin);
        machine.updateStatus(myNex, "Low Air - Pause");
        lastLowAir = true;
    }
    else{
        if (lastLowAir) {
            DBGLN("[AIR] Air pressure restored");
        }
        machine.hasLowAirPressure = false;
        lastLowAir = false;
    }
}

// Periodic status dump for debugging
void debugStatusDump() {
#if DEBUG_MODE
    static unsigned long lastDump = 0;
    const unsigned long DUMP_INTERVAL = 3000; // Every 3 seconds

    if (millis() - lastDump < DUMP_INTERVAL) return;
    if (!machine.inProduction || machine.isPaused) return; // Only during active production

    lastDump = millis();

    DBGLN("=== STATUS DUMP ===");
    DBG("  pos="); DBG_VAL(currentHomePosition);
    DBG(" moved="); DBG_VAL(machine.positionsMoved);
    DBG(" isMoving="); DBGLN_VAL(isMoving);

    DBG("  Teensy: wheelReady="); DBG_VAL(digitalRead(teensyWheelReadyPin));
    DBG(" overrun="); DBG_VAL(digitalRead(teensyOverrunAlarmPin));
    DBG(" bulbRamHome="); DBGLN_VAL(digitalRead(bulbRamHomeSensorPin));

    DBG("  Sensors: wheelPos="); DBG_VAL(digitalRead(wheelPositionSensorPin));
    DBG(" home="); DBG_VAL(digitalRead(homeSensorPin));
    DBG(" bulbInCap="); DBG_VAL(digitalRead(bulbInCapSensor));
    DBG(" capPos="); DBGLN_VAL(digitalRead(capPositionSensorPin));

    DBG("  Ready: bulb="); DBG_VAL(machine.bulbSystemReady);
    DBG(" cap="); DBG_VAL(machine.capInjectionReady);
    DBG(" preload="); DBG_VAL(machine.bulbPreLoadReady);
    DBG(" pipet="); DBGLN_VAL(machine.pipetSystemReady);

    DBG("  Rams: bulb="); DBG_VAL(digitalRead(bulbRamPin));
    DBG(" pipet="); DBG_VAL(digitalRead(pipetRamPin));
    DBG(" preload="); DBGLN_VAL(digitalRead(bulbPreLoadCylinder));

    DBG("  PAUSE_AFTER="); DBG_VAL(PAUSE_AFTER / 1000); DBGLN("ms");
    DBGLN("===================");
#endif
}

// ============================================================================
// NEW FUNCTION: Handle Teensy Alarms
// ============================================================================
void handleTeensyAlarms() {
    static bool lastOverrunAlarm = LOW;

    bool currentOverrunAlarm = digitalRead(teensyOverrunAlarmPin);

    // Check for RAM OVERRUN alarm (rising edge)
    if (currentOverrunAlarm == HIGH && lastOverrunAlarm == LOW) {
        DBGLN("[ALARM] TEENSY RAM OVERRUN DETECTED!");
        machine.pause(junkEjectorPin, dropperEjectPin);
        machine.updateStatus(myNex, "RAM OVERRUN ERROR");
        machine.hasTeensyRamError = true;
    }

    lastOverrunAlarm = currentOverrunAlarm;
}

void setup()
{
    DBG_INIT();
    DBGLN("=== DROPPER MACHINE STARTING ===");
    DBGLN("Debug logging ENABLED - watching for decisions");
    DBG("  stepPin="); DBG_VAL(stepPin);
    DBG(" teensyWheelReady="); DBG_VAL(teensyWheelReadyPin);
    DBG(" wheelPosSensor="); DBGLN_VAL(wheelPositionSensorPin);
    DBG("  bulbRamPin="); DBG_VAL(bulbRamPin);
    DBG(" bulbRamHome="); DBG_VAL(bulbRamHomeSensorPin);
    DBG(" pipetRamPin="); DBGLN_VAL(pipetRamPin);

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
    pinMode(wheelPositionSensorPin, INPUT);

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
    // Track state changes for logging
    static bool lastInProduction = false;
    static bool lastIsPaused = false;
    static bool lastIsStopped = true;
    static bool lastNeedsHoming = true;
    static unsigned long lastStateLog = 0;

    // Log major state changes
    if (machine.inProduction != lastInProduction) {
        DBG("[LOOP] inProduction: "); DBGLN_VAL(machine.inProduction);
        lastInProduction = machine.inProduction;
    }
    if (machine.isPaused != lastIsPaused) {
        DBG("[LOOP] isPaused: "); DBGLN_VAL(machine.isPaused);
        lastIsPaused = machine.isPaused;
    }
    if (machine.isStopped != lastIsStopped) {
        DBG("[LOOP] isStopped: "); DBGLN_VAL(machine.isStopped);
        lastIsStopped = machine.isStopped;
    }
    if (machine.needsHoming != lastNeedsHoming) {
        DBG("[LOOP] needsHoming: "); DBGLN_VAL(machine.needsHoming);
        lastNeedsHoming = machine.needsHoming;
    }

    handleLowAirPressure();
    handleTeensyAlarms();        // NEW: Monitor Teensy alarm signals
    updatePauseAfterFromPot();
    handleButtons();
    handleSupplyAlert();
    setSlotIdByPosition(slots);
    debugStatusDump();           // Periodic status dump for debugging

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

    if (machine.inProduction && !machine.isPaused)
    {
        stepMotor();
    }
    else
    {
        // Log why we're not calling stepMotor (only every 2 seconds)
        if (millis() - lastStateLog > 2000) {
            lastStateLog = millis();
            if (!machine.inProduction) {
                DBGLN("[LOOP] Not calling stepMotor: inProduction=false");
            }
            if (machine.isPaused) {
                DBGLN("[LOOP] Not calling stepMotor: isPaused=true");
            }
        }
    }
}
