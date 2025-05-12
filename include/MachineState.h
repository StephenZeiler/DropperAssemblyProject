
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
    myNex.writeStr("cautiontTxt.txt", "");
    //myNex.writeStr("cautiontTxt.txt", (String)i+"\\r");
}


};

#endif
