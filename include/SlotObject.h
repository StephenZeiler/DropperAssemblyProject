#ifndef SLOT_OBJECT_H
#define SLOT_OBJECT_H

class SlotObject {
private:
    const int slotId;      // Permanent ID (1-16)
    bool hasError;
    int currentPosition;   // Dynamic position (0-15)
    bool isJunk;
    
public:
    SlotObject(int id);
    
    // Getters
    int getId() const { return slotId; }
    int getPosition() const { return currentPosition; }
    bool getError() const { return hasError; }
    bool hasJunk() const {return isJunk;}

    // Setters
    void setPosition(int position) { currentPosition = position % 16; }
    void setError(bool error) { hasError = error; }
    void setJunk(bool junk) {isJunk = junk;}
    
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

    int slotNumberByID(SlotObject slots[], int position)
{
     for(int i = 0; i < 16; i++) {
        if(slots[i].getPosition()==position){
            return slots[i].getId();
        }
     }
}
};

#endif