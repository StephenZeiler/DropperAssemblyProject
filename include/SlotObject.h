// In SlotObject.h
class SlotObject {
private:
    const int slotId;
    bool hasError;
    int currentPosition;
    bool firstRotationComplete;  // Track if first rotation is done
    int assemblyStep;            // Current assembly step (0-15)
    
public:
    SlotObject(int id);
    
    // Getters
    int getId() const { return slotId; }
    int getPosition() const { return currentPosition; }
    bool getError() const { return hasError; }
    bool isFirstRotationComplete() const { return firstRotationComplete; }
    int getAssemblyStep() const { return assemblyStep; }
    
    // Setters
    void setPosition(int position) { 
        currentPosition = position % 16;
        if (currentPosition == 0 && !firstRotationComplete) {
            firstRotationComplete = true;
        }
    }
    void setError(bool error) { hasError = error; }
    void incrementAssemblyStep() { assemblyStep++; }
    
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
    
    // Should perform action based on current assembly step
    bool shouldPerformAction() const {
        if (firstRotationComplete) return true;
        return assemblyStep == currentPosition;
    }
};