#ifndef SLOT_OBJECT_H
#define SLOT_OBJECT_H

class SlotObject {
private:
    const int slotId;      // Permanent ID (1-16)
    bool errorInSlot;
    int currentPosition;   // Dynamic position (0-15)
    bool isJunk;
    bool isMissingBulb;
    bool isMissingCap;
    bool isJunkEjectFail;
    bool shouldFinishProd;
    
public:
    SlotObject(int id);
    
    // Getters
    int getId() const { return slotId; }
    int getPosition() const { return currentPosition; }
    bool hasError() const { return errorInSlot; }
    bool hasJunk() const {return isJunk;}
    bool hasMissingBulb() const {return isMissingBulb;}
    bool hasMissingCap() const {return isMissingCap;}
    bool hasFailedJunkEject() const {return isJunkEjectFail;}
    bool shouldFinishProduction() const {return shouldFinishProd;}

    // Setters
    void setPosition(int position) { currentPosition = position % 16; }
    void setError(bool error) { errorInSlot = error; }
    void setJunk(bool junk) {isJunk = junk;}
    void setMissingBulb(bool missingBulb) {isMissingBulb = missingBulb;}
    void setMissingCap(bool missingCap) {isMissingCap = missingCap;}
    void setFailedJunkEject(bool failedEject) {isJunkEjectFail = failedEject;}
    void setFinsihProduction(bool finish) {shouldFinishProd = finish;}
    
    // Position checks
    bool isAtFailedJunkEject() const { return currentPosition == 0; }
    bool isAtCapInjection() const { return currentPosition == 1; }
    bool isAtCapConfirm() const { return currentPosition == 2; }
    bool isAtBulbInjection() const { return currentPosition == 5; }
    bool isAtBulbConfirm() const { return currentPosition == 6; }
    bool isAtPipetInjection() const { return currentPosition == 9; }
    bool isAtPipetConfirm() const { return currentPosition == 10; }
    bool isAtCompletedEjection() const { return currentPosition == 13; }
    bool isAtJunkEjection() const { return currentPosition == 14; }
    bool isAtJunkEjectConfirm() const { return currentPosition == 15; }
};

#endif