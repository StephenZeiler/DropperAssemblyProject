
#ifndef MACHINE_STATE_H
#define MACHINE_STATE_H
#include "EasyNextionLibrary.h"

class MachineState {
public:
    bool isPaused = false;
    bool isStopped = true;
    bool inProduction = false;
    bool needsHoming = true;

    int positionsMoved = 0;
    bool bulbSystemReady = true;
    bool dropperSystemReady = true;  // Add this line
    bool capInjectionReady = true;
    bool bulbPreLoadReady = true;
    bool pipetSystemReady = true;  // Add this line
    long lastErrorResetTime = 0;
    long lastCautionResetTime = 0;
    long lastDropperCompleteResetTime = 0;
    long lastrunTimeResetTime = 0;
    bool printErrorLogs;
    bool printCautionLogs;
    int totalDroppersComplete = 0;
    int totalErroredDroppers = 0;
    int timeLoggingDelay = 230;
    bool statusStateChange = false;
    String status = "";

void incrementDroppersCompleted(){
    totalDroppersComplete++;
}
void incrementErroredDroppers(){
    totalErroredDroppers++;
}
int getCompletedDropperCnt(){
    return totalDroppersComplete;
}
int getErrorDropperCnt(){
    return totalErroredDroppers;
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
bool canCapInjectStart(){
    return positionsMoved >=0;
}
bool canCapConfirmStart(){
    return positionsMoved > 1;
}
bool canPreLoadBulbProcessStart(){
    return positionsMoved > 4;
}
bool canBulbProcessStart(){
    return positionsMoved > 5;
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
    }
    void finishProduction() {
        
    }
    
    void homingComplete() {
        needsHoming = false;
    }
    // Check if all pneumatics are ready
    bool isReadyToMove() {
        return 
        bulbSystemReady &&  // Add other systems here with &&
        //dropperSystemReady &&  // Add this
        //capInjectionReady &&
        bulbPreLoadReady &&
        pipetSystemReady &&
        !needsHoming && 
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
    void setBulbPreLoadReady(bool ready) {
        bulbPreLoadReady = ready;
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
        bulbPreLoadReady = false;
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

void updateStatus(EasyNex myNex, String newStatus){
    status=newStatus;
    statusStateChange = true;
    setStatusDisplay(myNex);
}
void updateMachineDisplayInfo(EasyNex myNex, long currentMilliTime, SlotObject slots[]) {
    static uint8_t displayCounter = 0;  
    // Stagger the updates (Option 3)
    switch(displayCounter) {
        case 0: 
            //setRunTimeDisplay(myNex, currentMilliTime); 
            break;
        case 1: 
            setDropperCntDisplay(myNex, currentMilliTime); 
            break;
        case 2: 
            //setErrorLogs(myNex, currentMilliTime); 
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
    if((currentMilliTime - lastDropperCompleteResetTime) >= 60000) {
        // Using character buffers (Option 1)
        char countCompletedStr[12]; // Enough for 10 digits plus null terminator
        snprintf(countCompletedStr, sizeof(countCompletedStr), "%d", getCompletedDropperCnt());
        myNex.writeStr("completedTxt.txt", countCompletedStr);

        char countErrorStr[12];
        snprintf(countErrorStr, sizeof(countErrorStr), "%d", getErrorDropperCnt());
        myNex.writeStr("junkTxt.txt", countErrorStr);
        lastDropperCompleteResetTime = currentMilliTime;
        //snprintf(countStr, sizeof(countStr), "%d", getErrorDropperCnt());
    }
}
void setStatusDisplay(EasyNex myNex) {
        // Using character buffers (Option 1)
    if(statusStateChange){
        char fullLog[1024] = {0}; // Buffer for full log
        int pos = 0;

             String msg = status;
                pos += snprintf(fullLog + pos, sizeof(fullLog) - pos, "%s", msg.c_str());
            
            myNex.writeStr("statusTxt.txt", fullLog);
    }
    statusStateChange = false;
}
void setErrorLogs(EasyNex myNex, long currentMilliTime) {
    if((currentMilliTime - lastErrorResetTime) >= timeLoggingDelay) {
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
    if ((currentMilliTime - lastCautionResetTime) >= timeLoggingDelay) {
        char fullLog[1024] = {0}; // Buffer for full log
        int pos = 0;

        for (int i = 0; i < 16; i++) {
            //test
            if (slots[i].isAtPipetInjection() && slots[i].shouldFinishProduction() && !pipetSystemReady){
                String msg = String(slots[i].getId()) + " pipet system not ready\r\n";
            }
            if (slots[i].isAtBulbInjection() && slots[i].shouldFinishProduction() && !bulbSystemReady){
                String msg = String(slots[i].getId()) + " bulb system not ready\r\n";
            }
            if (slots[i].isAtBulbPreLoad() && slots[i].shouldFinishProduction() && !bulbPreLoadReady){
                String msg = String(slots[i].getId()) + " bulb preload system not ready\r\n";
            }

            //test
            if (slots[i].hasMissingCap()) {
                String msg = "Slot " + String(slots[i].getId()) + " has missing cap.\r\n";
                pos += snprintf(fullLog + pos, sizeof(fullLog) - pos, "%s", msg.c_str());
            }

            if (slots[i].hasMissingBulb()) {
                String msg = "Slot " + String(slots[i].getId()) + " has missing bulb.\r\n";
                pos += snprintf(fullLog + pos, sizeof(fullLog) - pos, "%s", msg.c_str());
            }

            if (slots[i].hasJunk()) {
                String msg = "Slot " + String(slots[i].getId()) + " has broken/missing pipet.\r\n";
                pos += snprintf(fullLog + pos, sizeof(fullLog) - pos, "%s", msg.c_str());
            }

            if (slots[i].hasFailedJunkEject()) {
                String msg = "Slot " + String(slots[i].getId()) + " failed to eject junk.\r\n";
                pos += snprintf(fullLog + pos, sizeof(fullLog) - pos, "%s", msg.c_str());
            }

            // Prevent buffer overflow
            if (pos >= (int)sizeof(fullLog) - 64) {
                break;
            }
        }

        if (pos > 0) {
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