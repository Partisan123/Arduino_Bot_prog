class RampGenerator {
private:
    int currentPower;
    int targetPower;
    int stepSize; 
    int brakeStepSize; // Amount of power to change during braking
    unsigned long lastUpdateTime;
    unsigned long updateInterval; 
    bool brakeMode; 

public:
    RampGenerator(int initialPower = 0, int step = 3, int brakeStep = 10, unsigned long interval = 20)
        : currentPower(initialPower), targetPower(initialPower), stepSize(step), brakeStepSize(brakeStep), lastUpdateTime(0), updateInterval(interval), brakeMode(false) {}

    void setTargetPower(int power) {
        targetPower = power;
        if(power == 0) { // If target power is 0, activate brake mode
            brakeMode = true;
        } else {
            brakeMode = false;
        }
    }

    int update() {
        unsigned long currentTime = millis();
        if (currentTime - lastUpdateTime >= updateInterval) {
            lastUpdateTime = currentTime;
            
            int effectiveStep = brakeMode ? brakeStepSize : stepSize;

            if (currentPower < targetPower) {
                currentPower += effectiveStep;
                if (currentPower > targetPower) {
                    currentPower = targetPower;
                }
            } else if (currentPower > targetPower) {
                currentPower -= effectiveStep;
                if (currentPower < targetPower) {
                    currentPower = targetPower;
                }
            }
        }
        return currentPower;
    }

    // Call this method to activate the brake mode manually
    void applyBrake() {
        setTargetPower(0);
    }
};
