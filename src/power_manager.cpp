/**
 * @file power_manager.cpp
 * @brief Pre-charge circuit and contactor management implementation
 * 
 * State machine (non-blocking):
 * IDLE -> PRECHARGING -> CONTACTOR_DELAY -> READY
 *                    \-> EMERGENCY (on E-stop)
 */

#include "power_manager.h"

// Internal sub-states for non-blocking transitions
enum PrechargeSubState {
    PRECHARGE_WAITING,         // Waiting for precharge duration
    PRECHARGE_CONTACTOR_ON,    // Contactor just turned on, waiting to settle
    PRECHARGE_COMPLETE         // Ready to transition to READY
};

static PrechargeSubState prechargeSubState = PRECHARGE_WAITING;

// ============================================================================
// Constructor
// ============================================================================

PowerManager::PowerManager() {
    currentState = POWER_IDLE;
    previousState = POWER_IDLE;
    prechargeOn = false;
    contactorOn = false;
    prechargeStartTime = 0;
    stateChangeTime = 0;
}

// ============================================================================
// Initialization
// ============================================================================

void PowerManager::begin() {
    DEBUG_PRINTLN("    [PWR] Configuring pins...");
    // Configure pins
    pinMode(PRECHARGE_PIN, OUTPUT);
    pinMode(CONTACTOR_PIN, OUTPUT);
    
    // Ensure safe initial state - both OFF
    DEBUG_PRINTLN("    [PWR] Setting outputs LOW...");
    digitalWrite(PRECHARGE_PIN, LOW);
    digitalWrite(CONTACTOR_PIN, LOW);
    prechargeOn = false;
    contactorOn = false;
    
    currentState = POWER_IDLE;
    stateChangeTime = millis();
    DEBUG_PRINTLN("    [PWR] Power manager initialized - state: IDLE");
}

// ============================================================================
// Main Update - Call frequently in main loop (NON-BLOCKING)
// ============================================================================

void PowerManager::update() {
    // Always ensure mutual exclusion safety
    ensureMutualExclusion();
    
    // Handle current state
    switch (currentState) {
        case POWER_IDLE:
            // Waiting for startPrecharge() call
            break;
            
        case POWER_PRECHARGING:
            handlePrecharging();
            break;
            
        case POWER_READY:
            handleReady();
            break;
            
        case POWER_EMERGENCY:
            handleEmergency();
            break;
    }
}

// ============================================================================
// State Handlers (NON-BLOCKING)
// ============================================================================

void PowerManager::handlePrecharging() {
    unsigned long now = millis();
    unsigned long elapsed = now - prechargeStartTime;
    
    switch (prechargeSubState) {
        case PRECHARGE_WAITING:
            // Wait for pre-charge duration to complete
            if (elapsed >= PRECHARGE_DURATION_MS) {
                // Pre-charge complete - turn on contactor
                setContactor(true);
                stateChangeTime = now;  // Record when contactor turned on
                prechargeSubState = PRECHARGE_CONTACTOR_ON;
            }
            break;
            
        case PRECHARGE_CONTACTOR_ON:
            // Wait for contactor to settle (non-blocking)
            if (now - stateChangeTime >= CONTACTOR_SETTLE_MS) {
                // Contactor settled - turn off pre-charge
                setPrecharge(false);
                prechargeSubState = PRECHARGE_COMPLETE;
                // Transition to ready state
                transitionTo(POWER_READY);
            }
            break;
            
        case PRECHARGE_COMPLETE:
            // Should have already transitioned, but just in case
            transitionTo(POWER_READY);
            break;
    }
}

void PowerManager::handleReady() {
    // Normal operation - contactor should be ON, pre-charge OFF
    // This state is maintained until emergency stop or shutdown
}

void PowerManager::handleEmergency() {
    // Emergency state - ensure both are OFF
    if (prechargeOn || contactorOn) {
        setPrecharge(false);
        setContactor(false);
    }
}

// ============================================================================
// State Control Functions
// ============================================================================

void PowerManager::startPrecharge() {
    DEBUG_PRINTLN("[PWR] startPrecharge() called");
    
    if (currentState == POWER_EMERGENCY) {
        DEBUG_PRINTLN("[PWR] Cannot start - in EMERGENCY state");
        return;
    }
    
    // Ensure contactor is off before starting pre-charge
    setContactor(false);
    
    // Start pre-charge
    DEBUG_PRINTLN("[PWR] Turning ON pre-charge relay...");
    setPrecharge(true);
    DEBUG_PRINTLN("[PWR] Pre-charge relay ON");
    
    prechargeStartTime = millis();
    prechargeSubState = PRECHARGE_WAITING;
    
    transitionTo(POWER_PRECHARGING);
    DEBUG_PRINTLN("[PWR] startPrecharge() complete");
}

void PowerManager::emergencyStop() {
    // IMMEDIATE shutdown - no delays, direct GPIO control
    digitalWrite(CONTACTOR_PIN, LOW);
    contactorOn = false;
    
    digitalWrite(PRECHARGE_PIN, LOW);
    prechargeOn = false;
    
    prechargeSubState = PRECHARGE_WAITING;
    transitionTo(POWER_EMERGENCY);
}

void PowerManager::reset() {
    // Reset from emergency state - goes back to idle
    // Pre-charge must be manually started after reset
    
    // Ensure everything is off
    setPrecharge(false);
    setContactor(false);
    prechargeSubState = PRECHARGE_WAITING;
    
    transitionTo(POWER_IDLE);
}

// ============================================================================
// State Transition
// ============================================================================

void PowerManager::transitionTo(PowerState newState) {
    previousState = currentState;
    currentState = newState;
    stateChangeTime = millis();
}

// ============================================================================
// Direct Hardware Control
// ============================================================================

void PowerManager::setPrecharge(bool state) {
    #if DEBUG_MODE
        Serial.print("[PWR] setPrecharge(");
        Serial.print(state ? "ON" : "OFF");
        Serial.println(")");
    #endif
    
    prechargeOn = state;
    digitalWrite(PRECHARGE_PIN, state ? HIGH : LOW);
    delay(50);  // Give hardware time to settle
}

void PowerManager::setContactor(bool state) {
    #if DEBUG_MODE
        Serial.print("[PWR] setContactor(");
        Serial.print(state ? "ON" : "OFF");
        Serial.println(")");
    #endif
    
    contactorOn = state;
    digitalWrite(CONTACTOR_PIN, state ? HIGH : LOW);
}

void PowerManager::ensureMutualExclusion() {
    // In emergency, both must be off
    if (currentState == POWER_EMERGENCY) {
        if (prechargeOn || contactorOn) {
            digitalWrite(PRECHARGE_PIN, LOW);
            digitalWrite(CONTACTOR_PIN, LOW);
            prechargeOn = false;
            contactorOn = false;
        }
    }
}

// ============================================================================
// Status Functions
// ============================================================================

const char* PowerManager::getStateString() const {
    switch (currentState) {
        case POWER_IDLE:        return "IDLE";
        case POWER_PRECHARGING: return "PRECHARGING";
        case POWER_READY:       return "READY";
        case POWER_EMERGENCY:   return "EMERGENCY";
        default:                return "UNKNOWN";
    }
}

uint8_t PowerManager::getPrechargeProgress() const {
    if (currentState == POWER_READY) {
        return 100;
    }
    
    if (currentState != POWER_PRECHARGING) {
        return 0;
    }
    
    // During precharging, calculate progress
    unsigned long elapsed = millis() - prechargeStartTime;
    
    // If in contactor settling phase, show 100%
    if (prechargeSubState == PRECHARGE_CONTACTOR_ON || 
        prechargeSubState == PRECHARGE_COMPLETE) {
        return 100;
    }
    
    // Calculate percentage (cap at 99% during charging, 100% when done)
    if (elapsed >= PRECHARGE_DURATION_MS) {
        return 100;
    }
    
    uint8_t progress = (uint8_t)((elapsed * 100UL) / PRECHARGE_DURATION_MS);
    return progress;
}
