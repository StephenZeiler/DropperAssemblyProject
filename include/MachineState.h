
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
    bool pipetSystemReady = true;  // Add this line
    long lastErrorResetTime = 0;
    long lastCautionResetTime = 0;
    bool printErrorLogs = false;

    // Add more system flags here as needed:
    // bool capSystemReady = true;
    // bool pipetSystemReady = true;
void IncrementPositionsMoved(){
    positionsMoved++;
}

void ResetPositionsMoved(){
    positionsMoved = 0;
}
bool canPipetProcessStart(){
    return positionsMoved > 8;
}
bool canBulbProcessStart(){
    return positionsMoved > 4;
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
    
    void homingComplete() {
        needsHoming = false;
    }
    // Check if all pneumatics are ready
    bool isReadyToMove() {
        return 
        bulbSystemReady &&  // Add other systems here with &&
        dropperSystemReady &&  // Add this
        //capInjectionReady &&
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
void setCautionLogs(EasyNex myNex){
    //myNex.writeStr("cautiontTxt.txt", "");
    //myNex.writeStr("cautiontTxt.txt", (String)i+"\\r");
}

void setErrorLogs(EasyNex myNex, long currentMilliTime){
    if((currentMilliTime-lastErrorResetTime) >= 500){
        printErrorLogs = true;
        lastErrorResetTime=currentMilliTime;
        myNex.writeStr("errorTxt.txt", "");
    }
    if(printErrorLogs == true){

        if(!bulbPresent){
            myNex.writeStr("errorTxt.txt+", "No bulb detected for injection!\\r");
        }
        printErrorLogs = false;
    }
    //myNex.writeStr("cautiontTxt.txt", (String)i+"\\r");
}
bool setBackgroundColorError(EasyNex myNex){
    String stringFromNextion;
    myNex.NextionListen();
    stringFromNextion = myNex.readStr("errorTxt.txt");
    if(stringFromNextion!=""){
        myNex.writeNum("Logs.bco", 63488);
    }
    else{
        myNex.writeNum("Logs.bco", 50712);
    }
    //Yellow 65504
    //Grey 50712
    //red 63488
}
};

#endif
