#ifndef MACHINE_STATE_H
#define MACHINE_STATE_H

class MachineState {
private:
    bool isHomed;
    bool isPaused;
    bool isStopped;
    bool inProduction;
    bool needsHoming;

public:
    MachineState() : 
        isHomed(false),
        isPaused(false),
        isStopped(true),
        inProduction(false),
        needsHoming(true) {}

    // State checks
    bool homed() const { return isHomed; }
    bool paused() const { return isPaused; }
    bool stopped() const { return isStopped; }
    bool inProductionMode() const { return inProduction; }
    bool requiresHoming() const { return needsHoming; }

    // State transitions
    void startProduction() { 
        isStopped = false;
        isPaused = false;
        inProduction = true;
    }
    
    void pauseProduction() {
        isPaused = true;
        inProduction = false;
    }
    
    void stopProduction() {
        isStopped = true;
        isPaused = false;
        inProduction = false;
        needsHoming = true;
    }
    
    void completeHoming() {
        isHomed = true;
        needsHoming = false;
    }
    
    void resumeFromPause() {
        isPaused = false;
        inProduction = true;
        needsHoming = false;
    }
};

#endif