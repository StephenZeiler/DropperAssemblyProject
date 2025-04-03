#ifndef MACHINE_STATE_H
#define MACHINE_STATE_H

class MachineState {
public:
    bool isPaused = false;
    bool isStopped = true;
    bool inProduction = false;
    bool needsHoming = true;
    
    bool bulbSystemReady = true;
        bool dropperSystemReady = true;  // Add this line
    // Add more system flags here as needed:
    // bool capSystemReady = true;
    // bool pipetSystemReady = true;

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
        //  bulbSystemReady &&  // Add other systems here with &&
        //        dropperSystemReady &&  // ejection
               !needsHoming && 
               !isPaused && 
               !isStopped;
    }

    // Add this setter
    void setDropperSystemReady(bool ready) {
        dropperSystemReady = ready;
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
        // capSystemReady = false;
        // etc...
    }
};

#endif