#ifndef SLOT_OBJECT_H
#define SLOT_OBJECT_H

class SlotObject {
private:
    int slotId;      // Permanent ID (1-16)
    bool hasError;
    int currentPosition;   // Dynamic position (0-15)
    bool capInjected;
    bool bulbInjected;
    bool pipetInjected;
    
public:
    SlotObject(int id);
    
    // Getters
    int getId() const { return slotId; }
    int getPosition() const { return currentPosition; }
    bool getError() const { return hasError; }
    bool isCapInjected() const { return capInjected; }
    bool isBulbInjected() const { return bulbInjected; }
    bool isPipetInjected() const { return pipetInjected; }
    
    // Setters
    void setPosition(int position) { currentPosition = position % 16; }
    void setError(bool error) { hasError = error; }
    void setCapInjected(bool injected) { capInjected = injected; }
    void setBulbInjected(bool injected) { bulbInjected = injected; }
    void setPipetInjected(bool injected) { pipetInjected = injected; }
    
    // Position checks
    bool isAtHome() const { return currentPosition == 0; }
    bool isAtCapInjection() const { return currentPosition == 1; }
    bool isAtCapConfirm() const { return currentPosition == 2; }
    bool isAtBulbInjection() const { return currentPosition == 5; }
    bool isAtBulbConfirm() const { return currentPosition == 6; }
    bool isAtPipetInjection() const { return currentPosition == 9; }
    bool isAtPipetConfirm() const { return currentPosition == 10; }
    bool isAtCompletedEjection() const { return currentPosition == 13; }
    bool isAtJunkEjection() const { return currentPosition == 14; }
    bool isAtEmptyConfirm() const { return currentPosition == 15; }
    
    // Reset all injection flags
    void resetInjectionFlags() {
        capInjected = false;
        bulbInjected = false;
        pipetInjected = false;
    }
};

#endif