/**
 * @file safety_monitor.h
 * @brief Safety monitoring with interrupt-based button handling
 */

#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <Arduino.h>
#include "config.h"

// Debounced switch state (volatile for ISR access)
struct DebouncedSwitch {
    volatile bool currentState;
    volatile bool lastState;
    volatile unsigned long lastDebounceTime;
    volatile bool changed;
};

// Safety alert levels
enum SafetyAlert {
    ALERT_NONE,
    ALERT_LOW,
    ALERT_MEDIUM,
    ALERT_HIGH,
    ALERT_CRITICAL
};

class SafetyMonitor {
public:
    SafetyMonitor();
    
    void begin();
    void update();
    
    // Switch state access
    bool isEStopPressed() const { return emergencyStopActive; }
    bool isButton1Pressed() const { return button1State; }
    bool isButton2Pressed() const { return button2State; }
    bool isButton3Pressed() const { return button3State; }
    
    // Change detection
    bool eStopChanged() const { return eStopSwitch.changed; }
    bool button1Changed() const { return button1Switch.changed; }
    bool button2Changed() const { return button2Switch.changed; }
    bool button3Changed() const { return button3Switch.changed; }
    
    // Rising/falling edge detection
    bool eStopRisingEdge() const;
    bool eStopFallingEdge() const;
    bool button1RisingEdge() const;
    bool button1FallingEdge() const;
    bool button2RisingEdge() const;
    bool button2FallingEdge() const;
    bool button3RisingEdge() const;
    bool button3FallingEdge() const;
    
    // LED control
    void setButton1LED(bool state);
    void setButton2LED(bool state);
    void setButton3LED(bool state);
    void setHeartbeatLED(bool state);
    
    // Siren/Buzzer control
    void setSiren(bool state);
    void setBuzzer(bool state);
    void updateSiren();  // Call in loop for timed siren pattern
    
    // Safety checks
    bool isSafeToOperate() const;
    SafetyAlert getCurrentAlertLevel() const;
    const char* getAlertMessage() const;
    
    // Heartbeat
    void updateHeartbeat();
    
    // Reset functions
    void clearAlerts();
    void acknowledgeEStop();
    void clearChangedFlags();
    
    // ISR handlers (must be public for attachment)
    static void emergencyStopISR();
    static void button1ISR();
    static void button2ISR();
    static void button3ISR();
    
private:
    // Volatile state variables for ISR access
    static volatile bool emergencyStopActive;
    static volatile bool button1State;
    static volatile bool button2State;
    static volatile bool button3State;
    
    // Debounced switch objects
    static DebouncedSwitch eStopSwitch;
    static DebouncedSwitch button1Switch;
    static DebouncedSwitch button2Switch;
    static DebouncedSwitch button3Switch;
    
    // Alert state
    SafetyAlert currentAlert;
    char alertMessage[64];
    bool eStopAcknowledged;
    
    // Heartbeat state
    unsigned long lastHeartbeatToggle;
    bool heartbeatState;
    
    // Siren state for cycling
    unsigned long sirenCycleStart;
    bool sirenActive;
    
    // Timing
    unsigned long lastUpdate;
    
    // Alert evaluation
    void evaluateSafetyConditions();
    
    // LED states
    bool button1LEDState;
    bool button2LEDState;
    bool button3LEDState;
};

// Global safety monitor instance
extern SafetyMonitor safetyMonitor;

#endif // SAFETY_MONITOR_H
