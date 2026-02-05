/**
 * @file power_manager.h
 * @brief Pre-charge circuit and contactor management
 * 
 * Handles the safety-critical power sequencing:
 * - Pre-charge circuit timing
 * - Contactor control (mutually exclusive with pre-charge)
 * - Emergency stop response
 */

#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>
#include "config.h"

// Power system states
enum PowerState {
    POWER_IDLE,          // System off, waiting for start
    POWER_PRECHARGING,   // Pre-charge active, contactor off
    POWER_READY,         // Pre-charge off, contactor on (normal operation)
    POWER_EMERGENCY      // Emergency stop - both off
};

class PowerManager {
public:
    PowerManager();
    
    void begin();
    void update();
    
    // State control
    void startPrecharge();
    void emergencyStop();
    void reset();
    
    // State queries
    PowerState getState() const { return currentState; }
    bool isReady() const { return currentState == POWER_READY; }
    bool isEmergency() const { return currentState == POWER_EMERGENCY; }
    bool isPrecharging() const { return currentState == POWER_PRECHARGING; }
    
    // Get state as string for display/logging
    const char* getStateString() const;
    
    // Get pre-charge progress (0-100%)
    uint8_t getPrechargeProgress() const;
    
    // Direct control (use with caution)
    void setPrecharge(bool state);
    void setContactor(bool state);
    
    // Status
    bool isPrechargeOn() const { return prechargeOn; }
    bool isContactorOn() const { return contactorOn; }
    
private:
    PowerState currentState;
    PowerState previousState;
    
    // Hardware states
    bool prechargeOn;
    bool contactorOn;
    
    // Timing
    unsigned long prechargeStartTime;
    unsigned long stateChangeTime;
    
    // Internal state machine
    void transitionTo(PowerState newState);
    void handlePrecharging();
    void handleReady();
    void handleEmergency();
    
    // Safety checks
    void ensureMutualExclusion();
};

// Global power manager instance
extern PowerManager powerManager;

#endif // POWER_MANAGER_H

