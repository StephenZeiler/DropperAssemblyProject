
#ifndef MACHINE_STATE_H
#define MACHINE_STATE_H
#include "EasyNextionLibrary.h"

class MachineState {
public:
    bool isPaused = false;
    bool isStopped = true;
    bool inProduction = false;
    bool needsHoming = true;
    bool revolverEmpty = true;
    int positionsMoved = 0;
    bool bulbSystemReady = true;
    bool dropperSystemReady = true;  // Add this line
    bool capInjectionReady = true;
    bool pipetSystemReady = true;  // Add this line
    long lastErrorResetTime = 0;
    long lastCautionResetTime = 0;
    long lastDropperCompleteResetTime = 0;
    long lastrunTimeResetTime = 0;
    bool printErrorLogs;
    bool printCautionLogs;
    bool revolverAtHome = false;
    bool revolverShouldMove = true;
    int totalDroppersComplete = 0;
void incrementDroppersCompleted(){
    totalDroppersComplete = totalDroppersComplete++;
}
int getCompletedDropperCnt(){
    return totalDroppersComplete;
}
bool shouldRevolverMove(){
    return revolverShouldMove;
}
void setShouldRevolverMove(bool shouldMove){
    revolverShouldMove = shouldMove;
}
bool isRevolverAtHome(){
    return revolverAtHome;
}
void setRevolverPosition(bool isAtHome){
    revolverAtHome = isAtHome;
}

    // Add more system flags here as needed:
    // bool capSystemReady = true;
    // bool pipetSystemReady = true;
void IncrementPositionsMoved(){
    positionsMoved++;
}

void ResetPositionsMoved(){
    positionsMoved = 0;
}
bool canCapConfirmStart(){
    return positionsMoved > 1;
}
bool canBulbProcessStart(){
    return positionsMoved > 4;
}
bool canBulbConfirmStart(){
    return positionsMoved > 5;
}
bool canPipetProcessStart(){
    return positionsMoved > 8;
}
bool canPipetConfirmStart(){
    return positionsMoved > 9;
}
bool canDropperEjectionStart(){
    return positionsMoved > 12;
}
bool canJunkEjectionStart(){
    return positionsMoved > 13;
}
bool canCheckForEmptyStart(){
    return positionsMoved > 14;
}
    void start() {
        if (isStopped) {
            needsHoming = true;
            revolverEmpty = true;
            isStopped = false;
            inProduction = true;
        } else if (isPaused) {
            isPaused = false;
            inProduction = true;
        }
    }
    
    void pause() {
        if (!isStopped) {
            isPaused = true;
            inProduction = false;
        }
    }
    
    void stop() {
        isStopped = true;
        isPaused = false;
        inProduction = false;
        needsHoming = true;
        revolverEmpty = true;
        
    }
    void finishProduction() {
        
    }
    
    void homingComplete() {
        needsHoming = false;
    }
    void revolverFilled() {
        revolverEmpty = false;
    }
    // Check if all pneumatics are ready
    bool isReadyToMove() {
        return 
        bulbSystemReady &&  // Add other systems here with &&
        //dropperSystemReady &&  // Add this
        //capInjectionReady &&
        pipetSystemReady &&
        !needsHoming && 
        !revolverEmpty &&
        !isPaused && 
        !isStopped;
    }

    // Add this setter
    void setPipetSystemReady(bool ready) {
        pipetSystemReady = ready;
    }
    
    void setDropperSystemReady(bool ready) {
        dropperSystemReady = ready;
    }
    void setCapInjectionReady(bool ready) {
        capInjectionReady = ready;
    }
    // Set individual system readiness
    void setBulbSystemReady(bool ready) {
        bulbSystemReady = ready;
    }
    
    // Add similar setters for other systems:
    // void setCapSystemReady(bool ready) {
    //     capSystemReady = ready;
    // }
    
    // Reset all pneumatics to not ready
    void resetAllPneumatics() {
        bulbSystemReady = false;
        dropperSystemReady = false;  // Add this
        //capInjectionReady = false;
        pipetSystemReady = false;
        // capSystemReady = false;
        // etc...
    }
//_____________LOGGING________________
//
//
//
//ERRORS
bool bulbPresent = true;

//
//
//
//Cautions


void updateMachineDisplayInfo(EasyNex myNex, long currentMilliTime, SlotObject slots[]) {
    static uint8_t displayCounter = 0;  
    // Stagger the updates (Option 3)
    switch(displayCounter) {
        case 0: 
            setRunTimeDisplay(myNex, currentMilliTime); 
            break;
        case 1: 
            setDropperCntDisplay(myNex, currentMilliTime); 
            break;
        case 2: 
            setErrorLogs(myNex, currentMilliTime); 
            break;
        case 3: 
            setCautionLogs(myNex, currentMilliTime, slots); 
            break;
    }
    
    displayCounter = (displayCounter + 1) % 4;
}

void setRunTimeDisplay(EasyNex myNex, long currentMilliTime) {
    if((currentMilliTime - lastrunTimeResetTime) >= 60000) {
        // Using character buffers (Option 1)
        unsigned long currentUptime = millis() - currentMilliTime;
        unsigned long totalSeconds = currentUptime / 1000;
        unsigned long hours = totalSeconds / 3600;
        unsigned int minutes = (totalSeconds % 3600) / 60;
        
        char timeString[32]; // Enough for "XXXXX hours XX minutes"
        snprintf(timeString, sizeof(timeString), "%lu hours %02d minutes", hours, minutes);
        
        lastrunTimeResetTime = currentMilliTime;
        myNex.writeStr("t2.txt", timeString);
    }
}

void setDropperCntDisplay(EasyNex myNex, long currentMilliTime) {
    if((currentMilliTime - lastDropperCompleteResetTime) >= 500) {
        // Using character buffers (Option 1)
        char countStr[12]; // Enough for 10 digits plus null terminator
        snprintf(countStr, sizeof(countStr), "%d", getCompletedDropperCnt());
        
        lastDropperCompleteResetTime = currentMilliTime;
        myNex.writeStr("t4.txt", countStr);
    }
}

void setErrorLogs(EasyNex myNex, long currentMilliTime) {
    if((currentMilliTime - lastErrorResetTime) >= 500) {
        // Using character buffers (Option 1)
        char fullLog[256] = {0}; // Adjust size as needed
        int pos = 0;
        
        if(!bulbPresent) {
            pos += snprintf(fullLog + pos, sizeof(fullLog) - pos, "No bulb detected for injection!\\r");
        }
        
        // Only update if there's something to display
        if(pos > 0) {
            lastErrorResetTime = currentMilliTime;
            myNex.writeStr("errorTxt.txt", fullLog);
        }
    }
}

void setCautionLogs(EasyNex myNex, long currentMilliTime, SlotObject slots[]) {
    if((currentMilliTime - lastCautionResetTime) >= 500) {
        // Using character buffers (Option 1)
        char fullLog[1024] = {0}; // Adjust size based on maximum expected log size
        int pos = 0;
        
        for(int i = 0; i < 16; i++) {
            if(slots[i].hasMissingCap()) {
                pos += snprintf(fullLog + pos, sizeof(fullLog) - pos, "Slot %d has missing cap.\\r", i);
            }
            if(slots[i].hasMissingBulb()) {
                pos += snprintf(fullLog + pos, sizeof(fullLog) - pos, "Slot %d has missing bulb.\\r", i);
            }
            if(slots[i].hasJunk()) {
                pos += snprintf(fullLog + pos, sizeof(fullLog) - pos, "Slot %d has broken/missing pipet.\\r", i);
            }
            if(slots[i].hasFailedJunkEject()) {
                pos += snprintf(fullLog + pos, sizeof(fullLog) - pos, "Slot %d failed to eject junk.\\r", i);
            }
            
            // Prevent buffer overflow
            if(pos >= (int)sizeof(fullLog) - 64) {
                break;
            }
        }
        
        // Only update if there's something to display
        if(pos > 0) {
            lastCautionResetTime = currentMilliTime;
            myNex.writeStr("cautionTxt.txt", fullLog);
        }
    }
}



/////

//OLD

////
// void updateMachineDisplayInfo(EasyNex myNex, long currentMilliTime, SlotObject slots[]){
//     setRunTimeDisplay(myNex, currentMilliTime);
//     setDropperCntDisplay (myNex, currentMilliTime);
//     setErrorLogs(myNex, currentMilliTime);
//     setCautionLogs(myNex, currentMilliTime, slots);
// }
// void setRunTimeDisplay(EasyNex myNex, long currentMilliTime){
//      unsigned long currentUptime = millis() - currentMilliTime;
//     // Convert to total seconds
//     unsigned long totalSeconds = currentUptime / 1000;
    
//     // Calculate time components
//     unsigned long hours = totalSeconds / 3600;
//     unsigned int minutes = (totalSeconds % 3600) / 60;
    
//     // Format as "hours minutes" with hours up to 5 digits
//     char timeString[20]; // Enough for 5-digit hours + " hours " + 2-digit minutes + " minutes" + null
//     sprintf(timeString, "%lu hours %02d minutes", hours, minutes);
//     String display = String(timeString);
//     if((currentMilliTime-lastrunTimeResetTime) >= 500){
//         lastrunTimeResetTime=currentMilliTime;
//         myNex.writeStr("t2.txt", display);
//     }
// }

// void setDropperCntDisplay(EasyNex myNex, long currentMilliTime){
//     String display = (String)getCompletedDropperCnt();
//     if((currentMilliTime-lastDropperCompleteResetTime) >= 500){
//         lastDropperCompleteResetTime=currentMilliTime;
//         myNex.writeStr("t4.txt", display);
//     }
// }

// void setErrorLogs(EasyNex myNex, long currentMilliTime){
//     String fullLog = "";
//     if(!bulbPresent){
//         fullLog = fullLog + "No bulb detected for injection!\\r";
//     }
//     printErrorLogs = false;
//     if((currentMilliTime-lastErrorResetTime) >= 500){
//         lastErrorResetTime=currentMilliTime;
//         myNex.writeStr("errorTxt.txt", fullLog);
//     }
// }

// void setCautionLogs(EasyNex myNex, long currentMilliTime, SlotObject slots[]){
//     String fullLog = "";
//     for(int i = 0; i < 16; i++) {
//         if(slots[i].hasMissingCap()){
//             fullLog = fullLog + "Slot " + i + " has missing cap.\\r";
//         }
//         if(slots[i].hasMissingBulb()){
//              fullLog = fullLog + "Slot " + i + " has missing bulb.\\r";
//         }
//         if(slots[i].hasJunk()){
//              fullLog = fullLog + "Slot " + i + " has broken/missing pipet.\\r";
//         }
//         if(slots[i].hasFailedJunkEject()){
//              fullLog = fullLog + "Slot " + i + " failed to eject junk.\\r";
//         }
//     }
//         printCautionLogs = false;
//     if((currentMilliTime-lastCautionResetTime) >= 500){
//         lastCautionResetTime=currentMilliTime;
//        myNex.writeStr("cautionTxt.txt", fullLog);
//     }
// }
// bool setBackgroundColorError(EasyNex myNex){
    //     String stringFromNextion;
    //     myNex.NextionListen();
    //     stringFromNextion = myNex.readStr("errorTxt.txt");
    //     if(stringFromNextion!=""){
        //         myNex.writeNum("Logs.bco", 63488);
        //     }
        //     else{
            //         myNex.writeNum("Logs.bco", 50712);
//     }
//     //Yellow 65504
//     //Grey 50712
//     //red 63488
// }
};

#endif