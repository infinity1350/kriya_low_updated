/**
 * @file safety_monitor.cpp
 * @brief Safety monitoring with interrupt-based button handling
 */

#include "safety_monitor.h"
#include "power_manager.h"

// External instances
extern PowerManager powerManager;

// Static member initialization
volatile bool SafetyMonitor::emergencyStopActive = false;
volatile bool SafetyMonitor::button1State = false;
volatile bool SafetyMonitor::button2State = false;
volatile bool SafetyMonitor::button3State = false;

DebouncedSwitch SafetyMonitor::eStopSwitch = {false, false, 0, false};
DebouncedSwitch SafetyMonitor::button1Switch = {false, false, 0, false};
DebouncedSwitch SafetyMonitor::button2Switch = {false, false, 0, false};
DebouncedSwitch SafetyMonitor::button3Switch = {false, false, 0, false};

// ============================================================================
// Constructor and Initialization
// ============================================================================

SafetyMonitor::SafetyMonitor() {
    currentAlert = ALERT_NONE;
    memset(alertMessage, 0, sizeof(alertMessage));
    eStopAcknowledged = false;
    
    lastHeartbeatToggle = 0;
    heartbeatState = false;
    
    sirenCycleStart = 0;
    sirenActive = false;
    
    lastUpdate = 0;
    
    button1LEDState = false;
    button2LEDState = false;
    button3LEDState = false;
}

void SafetyMonitor::begin() {
    // Configure input pins with internal pull-ups
    pinMode(E_STOP_PIN, INPUT_PULLUP);
    pinMode(BUTTON1_IN_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_IN_PIN, INPUT_PULLUP);
    pinMode(BUTTON3_IN_PIN, INPUT_PULLUP);
    
    // Configure output pins
    pinMode(BUTTON1_OUT_PIN, OUTPUT);
    pinMode(BUTTON2_OUT_PIN, OUTPUT);
    pinMode(BUTTON3_OUT_PIN, OUTPUT);
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    pinMode(SIREN_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    
    // Initialize outputs to LOW
    digitalWrite(BUTTON1_OUT_PIN, LOW);
    digitalWrite(BUTTON2_OUT_PIN, LOW);
    digitalWrite(BUTTON3_OUT_PIN, LOW);
    digitalWrite(ONBOARD_LED_PIN, LOW);
    digitalWrite(SIREN_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    
    // Read initial states
    emergencyStopActive = !digitalRead(E_STOP_PIN);  // Active LOW
    button1State = !digitalRead(BUTTON1_IN_PIN);
    button2State = !digitalRead(BUTTON2_IN_PIN);
    button3State = !digitalRead(BUTTON3_IN_PIN);
    
    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(E_STOP_PIN), emergencyStopISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BUTTON1_IN_PIN), button1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BUTTON2_IN_PIN), button2ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BUTTON3_IN_PIN), button3ISR, CHANGE);
}

// ============================================================================
// Interrupt Service Routines
// ============================================================================

void SafetyMonitor::emergencyStopISR() {
    uint32_t currentTime = millis();
    if (currentTime - eStopSwitch.lastDebounceTime > EMERGENCY_DEBOUNCE_MS) {
        bool newState = !digitalRead(E_STOP_PIN);  // Active LOW
        if (newState != eStopSwitch.currentState) {
            eStopSwitch.lastState = eStopSwitch.currentState;
            eStopSwitch.currentState = newState;
            eStopSwitch.changed = true;
            emergencyStopActive = newState;
        }
        eStopSwitch.lastDebounceTime = currentTime;
    }
}

void SafetyMonitor::button1ISR() {
    uint32_t currentTime = millis();
    if (currentTime - button1Switch.lastDebounceTime > DEBOUNCE_DELAY_MS) {
        bool newState = !digitalRead(BUTTON1_IN_PIN);  // Active LOW
        if (newState != button1Switch.currentState) {
            button1Switch.lastState = button1Switch.currentState;
            button1Switch.currentState = newState;
            button1Switch.changed = true;
            button1State = newState;
        }
        button1Switch.lastDebounceTime = currentTime;
    }
}

void SafetyMonitor::button2ISR() {
    uint32_t currentTime = millis();
    if (currentTime - button2Switch.lastDebounceTime > DEBOUNCE_DELAY_MS) {
        bool newState = !digitalRead(BUTTON2_IN_PIN);  // Active LOW
        if (newState != button2Switch.currentState) {
            button2Switch.lastState = button2Switch.currentState;
            button2Switch.currentState = newState;
            button2Switch.changed = true;
            button2State = newState;
        }
        button2Switch.lastDebounceTime = currentTime;
    }
}

void SafetyMonitor::button3ISR() {
    uint32_t currentTime = millis();
    if (currentTime - button3Switch.lastDebounceTime > DEBOUNCE_DELAY_MS) {
        bool newState = !digitalRead(BUTTON3_IN_PIN);  // Active LOW
        if (newState != button3Switch.currentState) {
            button3Switch.lastState = button3Switch.currentState;
            button3Switch.currentState = newState;
            button3Switch.changed = true;
            button3State = newState;
        }
        button3Switch.lastDebounceTime = currentTime;
    }
}

// ============================================================================
// Main Update Function
// ============================================================================

void SafetyMonitor::update() {
    unsigned long now = millis();
    lastUpdate = now;
    
    // Evaluate safety conditions
    evaluateSafetyConditions();
    
    // Handle emergency stop
    if (emergencyStopActive) {
        // Trigger power manager emergency stop
        if (!powerManager.isEmergency()) {
            powerManager.emergencyStop();
        }
    }
    
    // Update siren pattern
    updateSiren();
}

// ============================================================================
// Edge Detection
// ============================================================================

bool SafetyMonitor::eStopRisingEdge() const {
    return eStopSwitch.changed && eStopSwitch.currentState;
}

bool SafetyMonitor::eStopFallingEdge() const {
    return eStopSwitch.changed && !eStopSwitch.currentState;
}

bool SafetyMonitor::button1RisingEdge() const {
    return button1Switch.changed && button1Switch.currentState;
}

bool SafetyMonitor::button1FallingEdge() const {
    return button1Switch.changed && !button1Switch.currentState;
}

bool SafetyMonitor::button2RisingEdge() const {
    return button2Switch.changed && button2Switch.currentState;
}

bool SafetyMonitor::button2FallingEdge() const {
    return button2Switch.changed && !button2Switch.currentState;
}

bool SafetyMonitor::button3RisingEdge() const {
    return button3Switch.changed && button3Switch.currentState;
}

bool SafetyMonitor::button3FallingEdge() const {
    return button3Switch.changed && !button3Switch.currentState;
}

void SafetyMonitor::clearChangedFlags() {
    eStopSwitch.changed = false;
    button1Switch.changed = false;
    button2Switch.changed = false;
    button3Switch.changed = false;
}

// ============================================================================
// LED Control
// ============================================================================

void SafetyMonitor::setButton1LED(bool state) {
    button1LEDState = state;
    digitalWrite(BUTTON1_OUT_PIN, state ? HIGH : LOW);
}

void SafetyMonitor::setButton2LED(bool state) {
    button2LEDState = state;
    digitalWrite(BUTTON2_OUT_PIN, state ? HIGH : LOW);
}

void SafetyMonitor::setButton3LED(bool state) {
    button3LEDState = state;
    digitalWrite(BUTTON3_OUT_PIN, state ? HIGH : LOW);
}

void SafetyMonitor::setHeartbeatLED(bool state) {
    heartbeatState = state;
    digitalWrite(ONBOARD_LED_PIN, state ? HIGH : LOW);
}

// ============================================================================
// Siren/Buzzer Control
// ============================================================================

void SafetyMonitor::setSiren(bool state) {
    sirenActive = state;
    digitalWrite(SIREN_PIN, state ? HIGH : LOW);
}

void SafetyMonitor::setBuzzer(bool state) {
    digitalWrite(BUZZER_PIN, state ? HIGH : LOW);
}

void SafetyMonitor::updateSiren() {
    if (emergencyStopActive) {
        uint32_t currentMillis = millis();
        
        // Reset cycle if needed
        if (currentMillis - sirenCycleStart >= SIREN_CYCLE_DURATION_MS) {
            sirenCycleStart = currentMillis;
        }
        
        // Siren ON for first SIREN_ON_DURATION_MS, OFF for rest
        bool sirenOn = (currentMillis - sirenCycleStart < SIREN_ON_DURATION_MS);
        digitalWrite(SIREN_PIN, sirenOn ? HIGH : LOW);
        digitalWrite(BUZZER_PIN, sirenOn ? HIGH : LOW);
    } else {
        // Ensure siren/buzzer are off when not in emergency
        digitalWrite(SIREN_PIN, LOW);
        digitalWrite(BUZZER_PIN, LOW);
        // Reset for next trigger
        sirenCycleStart = millis() - SIREN_CYCLE_DURATION_MS;
    }
}

// ============================================================================
// Heartbeat
// ============================================================================

void SafetyMonitor::updateHeartbeat() {
    heartbeatState = !heartbeatState;
    setHeartbeatLED(heartbeatState);
}

// ============================================================================
// Safety Evaluation
// ============================================================================

void SafetyMonitor::evaluateSafetyConditions() {
    if (emergencyStopActive) {
        currentAlert = ALERT_CRITICAL;
        strncpy(alertMessage, "EMERGENCY STOP ACTIVE", sizeof(alertMessage) - 1);
        return;
    }
    
    // All clear
    currentAlert = ALERT_NONE;
    alertMessage[0] = '\0';
}

bool SafetyMonitor::isSafeToOperate() const {
    return !emergencyStopActive && currentAlert < ALERT_CRITICAL;
}

SafetyAlert SafetyMonitor::getCurrentAlertLevel() const {
    return currentAlert;
}

const char* SafetyMonitor::getAlertMessage() const {
    return alertMessage;
}

// ============================================================================
// Alert Management
// ============================================================================

void SafetyMonitor::clearAlerts() {
    if (!emergencyStopActive) {
        currentAlert = ALERT_NONE;
        alertMessage[0] = '\0';
    }
}

void SafetyMonitor::acknowledgeEStop() {
    eStopAcknowledged = true;
    currentAlert = ALERT_NONE;
    alertMessage[0] = '\0';
}
