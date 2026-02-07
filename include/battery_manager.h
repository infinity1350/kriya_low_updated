/**
 * @file battery_manager.h
 * @brief Battery state management and hot-swap logic
 */

#ifndef BATTERY_MANAGER_H
#define BATTERY_MANAGER_H

#include <Arduino.h>
#include "config.h"
#include "bms_manager.h"

// Battery State Definitions
enum BatteryState {
    BATTERY_ACTIVE,      // Discharge MOSFET ON, supplying load
    BATTERY_STANDBY,     // Discharge MOSFET OFF, ready for swap
    BATTERY_REMOVED,     // Voltage < 10V, physically disconnected
    BATTERY_FAULT        // Communication error or protection active
};

// System State Definitions
enum SystemState {
    SYSTEM_IDLE,
    SYSTEM_SWITCHING_BATTERY,
    SYSTEM_BATTERY_REMOVED,
    SYSTEM_VOLTAGE_MISMATCH,
    SYSTEM_EMERGENCY_STOP,
    SYSTEM_ERROR
};

// Hot-swap Event Types
enum HotSwapEvent {
    HOTSWAP_NONE,
    HOTSWAP_REMOVED,
    HOTSWAP_RECONNECTED_OK,
    HOTSWAP_RECONNECTED_MISMATCH
};

class BatteryManager {
public:
    BatteryManager();
    
    void begin();
    
    // Main update functions
    void updateBatteryStates();
    void checkSwitchConditions();
    void detectHotSwap();
    
    // Initialization
    bool initializeBatteries();
    bool verifyInitialState();
    
    // Battery switching
    bool isReadyToSwitch();
    bool executeBatterySwitch();
    
    // Hot-swap detection
    HotSwapEvent checkHotSwapStatus(uint8_t batteryNum);
    float checkVoltageMatch(float v1, float v2);
    
    // State queries
    uint8_t getActiveBattery() const { return activeBattery; }
    uint8_t getStandbyBattery() const { return (activeBattery == 1) ? 2 : 1; }
    BatteryState getBattery1State() const { return battery1State; }
    BatteryState getBattery2State() const { return battery2State; }
    SystemState getSystemState() const { return systemState; }
    
    // State setters (for external control)
    void setSystemState(SystemState state) { systemState = state; }
    void emergencyShutdown();
    
    // Status strings for display/logging
    const char* getStateString(BatteryState state);
    const char* getSystemStateString();
    
private:
    uint8_t activeBattery;          // 1 or 2
    BatteryState battery1State;
    BatteryState battery2State;
    SystemState systemState;
    
    // Hot-swap tracking
    bool battery1WasConnected;
    bool battery2WasConnected;
    float lastBattery1Voltage;
    float lastBattery2Voltage;
    
    // Switching state
    bool switchInProgress;
    unsigned long switchStartTime;
    uint8_t switchStep;
    
    // Post-switch monitoring
    bool postSwitchMonitoring;
    unsigned long postSwitchStartTime;
    
    // Helper functions
    bool isBatteryConnected(uint8_t batteryNum);
    bool isBatteryHealthy(uint8_t batteryNum);
    bool verifyLoadTransfer(uint8_t batteryNum, float minCurrent);
    bool verifyIsolation(uint8_t batteryNum, float maxCurrent);
    void updateBatteryState(uint8_t batteryNum);
};

// Global battery manager instance (defined in main.cpp)
extern BatteryManager batteryManager;

#endif // BATTERY_MANAGER_H

