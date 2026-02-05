/**
 * @file buzzer_controller.cpp
 * @brief Buzzer and siren control implementation
 */

#include "buzzer_controller.h"

// ============================================================================
// Constructor and Initialization
// ============================================================================

BuzzerController::BuzzerController() {
    currentPattern = BUZZER_OFF;
    currentType = LOW_BUZZER;
    playing = false;
    muted = false;
    intensity = 255;
    
    patternStartTime = 0;
    lastToggleTime = 0;
    beepCount = 0;
    targetBeepCount = 0;
    beepDuration = 0;
    beepInterval = 0;
    buzzerState = false;
    
    toneStartTime = 0;
    toneDuration = 0;
    toneActive = false;
}

void BuzzerController::begin() {
    pinMode(LOW_BUZZ_PIN, OUTPUT);
    pinMode(HIGH_BUZZ_PIN, OUTPUT);
    
    setLowBuzzer(false);
    setHighBuzzer(false);
}

// ============================================================================
// Main Update Function
// ============================================================================

void BuzzerController::update() {
    if (muted || !playing) {
        setLowBuzzer(false);
        setHighBuzzer(false);
        return;
    }
    
    unsigned long now = millis();
    
    // Handle tone generation timeout
    if (toneActive && (now - toneStartTime >= toneDuration)) {
        noTone(currentType);
    }
    
    // Execute current pattern
    switch (currentPattern) {
        case BUZZER_OFF:
            setLowBuzzer(false);
            setHighBuzzer(false);
            playing = false;
            break;
            
        case BUZZER_SINGLE_BEEP:
            patternSingleBeep();
            break;
            
        case BUZZER_DOUBLE_BEEP:
            patternDoubleBeep();
            break;
            
        case BUZZER_TRIPLE_BEEP:
            patternTripleBeep();
            break;
            
        case BUZZER_CONTINUOUS:
            patternContinuous();
            break;
            
        case BUZZER_CONTINUOUS_HIGH:
            setHighBuzzer(true);
            setLowBuzzer(false);
            break;
            
        case BUZZER_INTERMITTENT:
            patternIntermittent();
            break;
            
        case BUZZER_FAST_BEEP:
            patternFastBeep();
            break;
            
        case BUZZER_SLOW_BEEP:
            patternSlowBeep();
            break;
            
        case BUZZER_WARNING:
            patternWarning();
            break;
            
        case BUZZER_ERROR:
            patternError();
            break;
            
        case BUZZER_SUCCESS:
            patternSuccess();
            break;
            
        case BUZZER_STARTUP:
            patternStartup();
            break;
            
        case BUZZER_SHUTDOWN:
            patternShutdown();
            break;
            
        default:
            setLowBuzzer(false);
            setHighBuzzer(false);
            playing = false;
            break;
    }
}

// ============================================================================
// Pattern Control
// ============================================================================

void BuzzerController::playPattern(BuzzerPattern pattern, BuzzerType type) {
    currentPattern = pattern;
    currentType = type;
    playing = true;
    patternStartTime = millis();
    lastToggleTime = millis();
    beepCount = 0;
    buzzerState = false;
    
    // Set specific parameters based on pattern
    switch (pattern) {
        case BUZZER_SINGLE_BEEP:
            targetBeepCount = 1;
            beepDuration = BEEP_SHORT_DURATION;
            beepInterval = BEEP_INTERVAL;
            break;
            
        case BUZZER_DOUBLE_BEEP:
            targetBeepCount = 2;
            beepDuration = BEEP_SHORT_DURATION;
            beepInterval = BEEP_INTERVAL;
            break;
            
        case BUZZER_TRIPLE_BEEP:
            targetBeepCount = 3;
            beepDuration = BEEP_SHORT_DURATION;
            beepInterval = BEEP_INTERVAL;
            break;
            
        default:
            break;
    }
}

void BuzzerController::stop() {
    playing = false;
    currentPattern = BUZZER_OFF;
    setLowBuzzer(false);
    setHighBuzzer(false);
}

void BuzzerController::stopAll() {
    stop();
}

// ============================================================================
// Direct Control
// ============================================================================

void BuzzerController::setLowBuzzer(bool state) {
    if (!muted) {
        digitalWrite(LOW_BUZZ_PIN, state ? HIGH : LOW);
    }
}

void BuzzerController::setHighBuzzer(bool state) {
    if (!muted) {
        digitalWrite(HIGH_BUZZ_PIN, state ? HIGH : LOW);
    }
}

void BuzzerController::setBothBuzzers(bool state) {
    setLowBuzzer(state);
    setHighBuzzer(state);
}

// ============================================================================
// Tone Control (not implemented for simple on/off buzzers)
// ============================================================================

void BuzzerController::tone(uint16_t frequency, uint16_t duration, BuzzerType type) {
    // For simple on/off buzzers, just turn on for duration
    activateBuzzer(type, true);
    toneStartTime = millis();
    toneDuration = duration;
    toneActive = true;
}

void BuzzerController::noTone(BuzzerType type) {
    activateBuzzer(type, false);
    toneActive = false;
}

// ============================================================================
// Custom Beep Functions
// ============================================================================

void BuzzerController::beep(uint16_t duration, BuzzerType type) {
    activateBuzzer(type, true);
    delay(duration);
    activateBuzzer(type, false);
}

void BuzzerController::beepTimes(uint8_t count, uint16_t duration, uint16_t interval, BuzzerType type) {
    for (uint8_t i = 0; i < count; i++) {
        beep(duration, type);
        if (i < count - 1) {
            delay(interval);
        }
    }
}

// ============================================================================
// Status Sounds
// ============================================================================

void BuzzerController::playStartupSound() {
    playPattern(BUZZER_STARTUP, LOW_BUZZER);
}

void BuzzerController::playShutdownSound() {
    playPattern(BUZZER_SHUTDOWN, LOW_BUZZER);
}

void BuzzerController::playWarningSound() {
    playPattern(BUZZER_WARNING, HIGH_BUZZER);
}

void BuzzerController::playErrorSound() {
    playPattern(BUZZER_ERROR, HIGH_BUZZER);
}

void BuzzerController::playSuccessSound() {
    playPattern(BUZZER_SUCCESS, LOW_BUZZER);
}

void BuzzerController::playButtonPressSound() {
    playPattern(BUZZER_SINGLE_BEEP, LOW_BUZZER);
}

void BuzzerController::playEStopSound() {
    playPattern(BUZZER_CONTINUOUS_HIGH, HIGH_BUZZER);
}

void BuzzerController::playBrakeSound() {
    playPattern(BUZZER_SINGLE_BEEP, LOW_BUZZER);
}

// ============================================================================
// Volume/Mute Control
// ============================================================================

void BuzzerController::setIntensity(uint8_t newIntensity) {
    intensity = newIntensity;
}

void BuzzerController::mute() {
    muted = true;
    setLowBuzzer(false);
    setHighBuzzer(false);
}

void BuzzerController::unmute() {
    muted = false;
}

// ============================================================================
// Pattern Implementations
// ============================================================================

void BuzzerController::patternSingleBeep() {
    unsigned long now = millis();
    unsigned long elapsed = now - patternStartTime;
    
    if (beepCount < targetBeepCount) {
        if (!buzzerState && (now - lastToggleTime >= beepInterval)) {
            // Start beep
            activateBuzzer(currentType, true);
            buzzerState = true;
            lastToggleTime = now;
        } else if (buzzerState && (now - lastToggleTime >= beepDuration)) {
            // End beep
            activateBuzzer(currentType, false);
            buzzerState = false;
            lastToggleTime = now;
            beepCount++;
        }
    } else {
        // Pattern complete
        stop();
    }
}

void BuzzerController::patternDoubleBeep() {
    patternSingleBeep();  // Uses same logic with different targetBeepCount
}

void BuzzerController::patternTripleBeep() {
    patternSingleBeep();  // Uses same logic with different targetBeepCount
}

void BuzzerController::patternContinuous() {
    activateBuzzer(currentType, true);
}

void BuzzerController::patternIntermittent() {
    unsigned long now = millis();
    
    // Toggle every 500ms
    if (now - lastToggleTime >= 500) {
        buzzerState = !buzzerState;
        activateBuzzer(currentType, buzzerState);
        lastToggleTime = now;
    }
}

void BuzzerController::patternFastBeep() {
    unsigned long now = millis();
    
    // Toggle every 100ms
    if (now - lastToggleTime >= 100) {
        buzzerState = !buzzerState;
        activateBuzzer(currentType, buzzerState);
        lastToggleTime = now;
    }
}

void BuzzerController::patternSlowBeep() {
    unsigned long now = millis();
    
    // Toggle every 1000ms
    if (now - lastToggleTime >= 1000) {
        buzzerState = !buzzerState;
        activateBuzzer(currentType, buzzerState);
        lastToggleTime = now;
    }
}

void BuzzerController::patternWarning() {
    // Alternating beeps
    unsigned long now = millis();
    unsigned long elapsed = now - patternStartTime;
    
    if ((elapsed / 200) % 2 == 0) {
        activateBuzzer(currentType, true);
    } else {
        activateBuzzer(currentType, false);
    }
}

void BuzzerController::patternError() {
    // Fast alternating HIGH buzzer
    unsigned long now = millis();
    
    if (now - lastToggleTime >= 100) {
        buzzerState = !buzzerState;
        setHighBuzzer(buzzerState);
        lastToggleTime = now;
    }
}

void BuzzerController::patternSuccess() {
    // Two quick beeps then stop
    unsigned long elapsed = millis() - patternStartTime;
    
    if (elapsed < 100) {
        activateBuzzer(currentType, true);
    } else if (elapsed < 200) {
        activateBuzzer(currentType, false);
    } else if (elapsed < 300) {
        activateBuzzer(currentType, true);
    } else {
        activateBuzzer(currentType, false);
        stop();
    }
}

void BuzzerController::patternStartup() {
    // Ascending tone simulation (single long beep)
    unsigned long elapsed = millis() - patternStartTime;
    
    if (elapsed < 500) {
        activateBuzzer(LOW_BUZZER, true);
    } else {
        activateBuzzer(LOW_BUZZER, false);
        stop();
    }
}

void BuzzerController::patternShutdown() {
    // Descending tone simulation (two beeps)
    unsigned long elapsed = millis() - patternStartTime;
    
    if (elapsed < 200) {
        activateBuzzer(LOW_BUZZER, true);
    } else if (elapsed < 300) {
        activateBuzzer(LOW_BUZZER, false);
    } else if (elapsed < 500) {
        activateBuzzer(LOW_BUZZER, true);
    } else {
        activateBuzzer(LOW_BUZZER, false);
        stop();
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

void BuzzerController::activateBuzzer(BuzzerType type, bool state) {
    if (muted) {
        return;
    }
    
    switch (type) {
        case LOW_BUZZER:
            setLowBuzzer(state);
            break;
            
        case HIGH_BUZZER:
            setHighBuzzer(state);
            break;
            
        case BOTH_BUZZERS:
            setLowBuzzer(state);
            setHighBuzzer(state);
            break;
    }
}

void BuzzerController::toggleBuzzer() {
    buzzerState = !buzzerState;
    activateBuzzer(currentType, buzzerState);
}

bool BuzzerController::isPatternComplete() {
    // Check if pattern has finished
    if (currentPattern == BUZZER_SINGLE_BEEP || 
        currentPattern == BUZZER_DOUBLE_BEEP || 
        currentPattern == BUZZER_TRIPLE_BEEP) {
        return (beepCount >= targetBeepCount);
    }
    return false;
}

void BuzzerController::resetPattern() {
    beepCount = 0;
    buzzerState = false;
    patternStartTime = millis();
    lastToggleTime = millis();
}

