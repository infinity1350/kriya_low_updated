/**
 * @file buzzer_controller.h
 * @brief Buzzer and siren control with various alert patterns
 */

#ifndef BUZZER_CONTROLLER_H
#define BUZZER_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

// Buzzer Pattern Types
enum BuzzerPattern {
    BUZZER_OFF,
    BUZZER_SINGLE_BEEP,
    BUZZER_DOUBLE_BEEP,
    BUZZER_TRIPLE_BEEP,
    BUZZER_CONTINUOUS,
    BUZZER_INTERMITTENT,
    BUZZER_FAST_BEEP,
    BUZZER_SLOW_BEEP,
    BUZZER_WARNING,
    BUZZER_ERROR,
    BUZZER_SUCCESS,
    BUZZER_STARTUP,
    BUZZER_SHUTDOWN,
    BUZZER_CONTINUOUS_HIGH  // For voltage mismatch emergency
};

// Buzzer Type
enum BuzzerType {
    LOW_BUZZER,    // Low frequency buzzer
    HIGH_BUZZER,   // High frequency siren
    BOTH_BUZZERS   // Both buzzer and siren
};

// Buzzer Note (for tone generation if needed)
struct BuzzerNote {
    uint16_t frequency;  // Hz
    uint16_t duration;   // ms
};

class BuzzerController {
public:
    BuzzerController();
    
    void begin();
    void update();
    
    // Pattern control
    void playPattern(BuzzerPattern pattern, BuzzerType type = LOW_BUZZER);
    void stop();
    void stopAll();
    
    // Direct control
    void setLowBuzzer(bool state);
    void setHighBuzzer(bool state);
    void setBothBuzzers(bool state);
    
    // Tone control (for PWM-capable pins)
    void tone(uint16_t frequency, uint16_t duration, BuzzerType type = LOW_BUZZER);
    void noTone(BuzzerType type = LOW_BUZZER);
    
    // Custom patterns
    void beep(uint16_t duration = BEEP_SHORT_DURATION, BuzzerType type = LOW_BUZZER);
    void beepTimes(uint8_t count, uint16_t duration = BEEP_SHORT_DURATION, 
                   uint16_t interval = BEEP_INTERVAL, BuzzerType type = LOW_BUZZER);
    
    // Status sounds
    void playStartupSound();
    void playShutdownSound();
    void playWarningSound();
    void playErrorSound();
    void playSuccessSound();
    void playButtonPressSound();
    void playEStopSound();
    void playBrakeSound();
    
    // State query
    bool isPlaying() const { return playing; }
    BuzzerPattern getCurrentPattern() const { return currentPattern; }
    
    // Volume/intensity control (for PWM)
    void setIntensity(uint8_t intensity);  // 0-255
    uint8_t getIntensity() const { return intensity; }
    
    // Mute control
    void mute();
    void unmute();
    bool isMuted() const { return muted; }
    
private:
    BuzzerPattern currentPattern;
    BuzzerType currentType;
    bool playing;
    bool muted;
    uint8_t intensity;
    
    // Pattern state
    unsigned long patternStartTime;
    unsigned long lastToggleTime;
    uint8_t beepCount;
    uint8_t targetBeepCount;
    uint16_t beepDuration;
    uint16_t beepInterval;
    bool buzzerState;
    
    // Tone generation state
    unsigned long toneStartTime;
    uint16_t toneDuration;
    bool toneActive;
    
    // Pattern implementations
    void patternSingleBeep();
    void patternDoubleBeep();
    void patternTripleBeep();
    void patternContinuous();
    void patternIntermittent();
    void patternFastBeep();
    void patternSlowBeep();
    void patternWarning();
    void patternError();
    void patternSuccess();
    void patternStartup();
    void patternShutdown();
    
    // Helper functions
    void activateBuzzer(BuzzerType type, bool state);
    void toggleBuzzer();
    bool isPatternComplete();
    void resetPattern();
};

// Global buzzer controller instance (defined in main.cpp)
extern BuzzerController buzzerController;

#endif // BUZZER_CONTROLLER_H

