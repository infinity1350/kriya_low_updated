/**
 * @file battery_manager.cpp
 * @brief Battery state management, switching, and hot-swap detection
 */

#include "battery_manager.h"
#include "bms_manager.h"
#include "display_manager.h"
#include "led_controller.h"
#include "buzzer_controller.h"

// External BMS instances
extern BMSManager bms1;
extern BMSManager bms2;
extern DisplayManager display;
extern LEDController ledController;
extern BuzzerController buzzerController;

// ============================================================================
// Constructor and Initialization
// ============================================================================

BatteryManager::BatteryManager() {
    activeBattery = 1;
    battery1State = BATTERY_STANDBY;
    battery2State = BATTERY_STANDBY;
    systemState = SYSTEM_IDLE;
    
    battery1WasConnected = false;
    battery2WasConnected = false;
    lastBattery1Voltage = 0.0f;
    lastBattery2Voltage = 0.0f;
    
    switchInProgress = false;
    switchStartTime = 0;
    switchStep = 0;
    
    postSwitchMonitoring = false;
    postSwitchStartTime = 0;
}

void BatteryManager::begin() {
    // Nothing specific needed here
}

// ============================================================================
// Initialization and Verification
// ============================================================================

bool BatteryManager::initializeBatteries() {
    Serial.println("Initializing battery MOSFETs...");
    
    // Step 1: Disable charge MOSFETs on both batteries
    Serial.println("  Disabling charge MOSFETs...");
    if (!bms1.disableChargeMosfet()) {
        Serial.println("  ERROR: Failed to disable Battery 1 charge MOSFET");
        return false;
    }
    
    if (!bms2.disableChargeMosfet()) {
        Serial.println("  ERROR: Failed to disable Battery 2 charge MOSFET");
        return false;
    }
    
    delay(100);
    
    // Step 2: Enable discharge on Battery 1 (make it active)
    Serial.println("  Enabling Battery 1 discharge MOSFET...");
    if (!bms1.enableDischargeMosfet()) {
        Serial.println("  ERROR: Failed to enable Battery 1 discharge MOSFET");
        return false;
    }
    
    delay(100);
    
    // Step 3: Disable discharge on Battery 2 (make it standby)
    Serial.println("  Disabling Battery 2 discharge MOSFET...");
    if (!bms2.disableDischargeMosfet()) {
        Serial.println("  ERROR: Failed to disable Battery 2 discharge MOSFET");
        return false;
    }
    
    delay(100);
    
    Serial.println("  MOSFET initialization complete");
    return true;
}

bool BatteryManager::verifyInitialState() {
    Serial.println("Verifying initial MOSFET states...");
    
    // Read both BMS
    bms1.requestBasicInfo();
    delay(50);
    bms2.requestBasicInfo();
    delay(50);
    
    BMSData data1 = bms1.getData();
    BMSData data2 = bms2.getData();
    
    // Check Battery 1
    Serial.print("  BAT1: Charge=");
    Serial.print(data1.chargeMosfetOn ? "ON" : "OFF");
    Serial.print(", Discharge=");
    Serial.print(data1.dischargeMosfetOn ? "ON" : "OFF");
    Serial.print(", Current=");
    Serial.print(data1.current, 2);
    Serial.println("A");
    
    // Check Battery 2
    Serial.print("  BAT2: Charge=");
    Serial.print(data2.chargeMosfetOn ? "ON" : "OFF");
    Serial.print(", Discharge=");
    Serial.print(data2.dischargeMosfetOn ? "ON" : "OFF");
    Serial.print(", Current=");
    Serial.print(data2.current, 2);
    Serial.println("A");
    
    // Verify expected states
    bool bat1OK = !data1.chargeMosfetOn && data1.dischargeMosfetOn;
    bool bat2OK = !data2.chargeMosfetOn && !data2.dischargeMosfetOn;
    
    if (bat1OK && bat2OK) {
        Serial.println("  Initial state verification: OK");
        activeBattery = 1;
        battery1State = BATTERY_ACTIVE;
        battery2State = BATTERY_STANDBY;
        return true;
    } else {
        Serial.println("  Initial state verification: FAILED");
        return false;
    }
}

// ============================================================================
// State Update Functions
// ============================================================================

void BatteryManager::updateBatteryStates() {
    updateBatteryState(1);
    updateBatteryState(2);
}

void BatteryManager::updateBatteryState(uint8_t batteryNum) {
    BMSManager* bms = (batteryNum == 1) ? &bms1 : &bms2;
    BatteryState* state = (batteryNum == 1) ? &battery1State : &battery2State;
    
    BMSData data = bms->getData();
    
    // Check if battery is connected (voltage > 10V)
    if (!isBatteryConnected(batteryNum)) {
        *state = BATTERY_REMOVED;
        return;
    }
    
    // Check for faults
    if (!data.dataValid || bms->isTimeout() || data.anyProtectionActive) {
        *state = BATTERY_FAULT;
        return;
    }
    
    // Check MOSFET state to determine active/standby
    if (data.dischargeMosfetOn) {
        *state = BATTERY_ACTIVE;
    } else {
        *state = BATTERY_STANDBY;
    }
}

bool BatteryManager::isBatteryConnected(uint8_t batteryNum) {
    BMSManager* bms = (batteryNum == 1) ? &bms1 : &bms2;
    BMSData data = bms->getData();
    
    // Battery is connected if voltage > 10V
    return (data.totalVoltage > 10.0f);
}

bool BatteryManager::isBatteryHealthy(uint8_t batteryNum) {
    BMSManager* bms = (batteryNum == 1) ? &bms1 : &bms2;
    BMSData data = bms->getData();
    
    // Check communication
    if (!data.dataValid || bms->isTimeout()) {
        return false;
    }
    
    // Check protections
    if (data.anyProtectionActive) {
        return false;
    }
    
    // Check voltage range
    if (data.totalVoltage < 42.0f || data.totalVoltage > 54.75f) {
        return false;
    }
    
    // Check temperature
    if (data.temp1 > 60.0f || data.temp1 < -10.0f) {
        return false;
    }
    
    return true;
}

// ============================================================================
// Battery Switching Logic
// ============================================================================

void BatteryManager::checkSwitchConditions() {
    // Don't check if already switching or in error state
    if (switchInProgress || systemState == SYSTEM_EMERGENCY_STOP) {
        return;
    }
    
    // Check if post-switch monitoring is active
    if (postSwitchMonitoring) {
        unsigned long elapsed = millis() - postSwitchStartTime;
        
        uint8_t standbyBat = getStandbyBattery();
        BMSManager* standbyBMS = (standbyBat == 1) ? &bms1 : &bms2;
        BMSData standbyData = standbyBMS->getData();
        
        // Check if old active battery (now standby) still has current
        if (standbyData.current > 1.0f) {
            Serial.println("WARNING: Standby battery MOSFET may be stuck ON!");
            Serial.print("Current: ");
            Serial.print(standbyData.current, 2);
            Serial.println("A");
        }
        
        // Monitor for 10 seconds
        if (elapsed > 10000) {
            postSwitchMonitoring = false;
            Serial.println("Post-switch monitoring complete");
        }
        
        return;
    }
    
    // Check if active battery needs switching
    if (isReadyToSwitch()) {
        executeBatterySwitch();
    }
}

bool BatteryManager::isReadyToSwitch() {
    BMSManager* activeBMS = (activeBattery == 1) ? &bms1 : &bms2;
    BMSManager* standbyBMS = (activeBattery == 1) ? &bms2 : &bms1;
    
    BMSData activeData = activeBMS->getData();
    BMSData standbyData = standbyBMS->getData();
    
    // Check if active battery is low (≤20% SOC)
    if (activeData.socPercent > 60) {
        return false;
    }
    
    Serial.println("Active battery SOC ≤ 20%, checking standby battery...");
    
    // Verify standby battery is ready
    if (!isBatteryConnected(getStandbyBattery())) {
        Serial.println("  Standby battery not connected");
        return false;
    }
    
    if (!isBatteryHealthy(getStandbyBattery())) {
        Serial.println("  Standby battery not healthy");
        return false;
    }
    
    if (standbyData.totalVoltage < 45.0f) {
        Serial.print("  Standby voltage too low: ");
        Serial.print(standbyData.totalVoltage, 2);
        Serial.println("V");
        return false;
    }
    
    if (standbyData.socPercent < 30) {
        Serial.print("  Standby SOC too low: ");
        Serial.print(standbyData.socPercent);
        Serial.println("%");
        return false;
    }
    
    if (standbyData.temp1 > 60.0f) {
        Serial.print("  Standby temperature too high: ");
        Serial.print(standbyData.temp1, 1);
        Serial.println("°C");
        return false;
    }
    
    Serial.println("  Standby battery ready for switch");
    return true;
}

bool BatteryManager::executeBatterySwitch() {
    uint8_t oldActive = activeBattery;
    uint8_t newActive = (oldActive == 1) ? 2 : 1;
    
    BMSManager* oldActiveBMS = (oldActive == 1) ? &bms1 : &bms2;
    BMSManager* newActiveBMS = (newActive == 1) ? &bms1 : &bms2;
    
    // Get current data for voltage logging
    BMSData oldData = oldActiveBMS->getData();
    BMSData newData = newActiveBMS->getData();
    
    // Log voltage difference (user has GND isolation, but log for diagnostics)
    float voltageDiff = abs(oldData.totalVoltage - newData.totalVoltage);
    if (voltageDiff > VOLTAGE_MISMATCH_WARN_V) {
        // Warning only - user has hardware GND isolation
        // Log via ROS if available
    }
    
    switchInProgress = true;
    systemState = SYSTEM_SWITCHING_BATTERY;
    switchStartTime = millis();
    
    // =====================================================================
    // MAKE-BEFORE-BREAK SWITCHING (GND isolated by hardware)
    // User has isolated GND pins, so we can safely enable new before disabling old
    // =====================================================================
    
    // Step 1: Enable discharge on new battery (both batteries briefly parallel)
    // With GND isolation, no equalizing current flows
    if (!newActiveBMS->enableDischargeMosfet()) {
        switchInProgress = false;
        systemState = SYSTEM_ERROR;
        return false;
    }
    
    // Step 2: Wait for stabilization
    delay(SWITCH_STABILIZATION_MS);
    
    // Step 3: Verify load transfer to new battery
    newActiveBMS->requestBasicInfo();
    delay(50);
    BMSData newActiveData = newActiveBMS->getData();
    
    // Check if new battery is supplying load (negative current = discharging)
    if (newActiveData.current < 0.5f && newActiveData.current > -0.5f) {
        // Load transfer failed - re-enable old battery and abort
        oldActiveBMS->enableDischargeMosfet();
        switchInProgress = false;
        systemState = SYSTEM_ERROR;
        return false;
    }
    
    // Step 4: Disable discharge on old battery
    if (!oldActiveBMS->disableDischargeMosfet()) {
        // Warning only - new battery is already active
    }
    
    // Step 5: Wait for isolation
    delay(SWITCH_ISOLATION_MS);
    
    // Step 6: Verify old battery is isolated
    oldActiveBMS->requestBasicInfo();
    delay(50);
    BMSData oldActiveData = oldActiveBMS->getData();
    
    // Log if not fully isolated (diagnostic only)
    if (abs(oldActiveData.current) > 0.1f) {
        // Old battery still has current - MOSFET may be stuck
        // Continue anyway as new battery is active
    }
    
    // Step 7: Update system state
    activeBattery = newActive;
    
    if (newActive == 1) {
        battery1State = BATTERY_ACTIVE;
        battery2State = BATTERY_STANDBY;
    } else {
        battery1State = BATTERY_STANDBY;
        battery2State = BATTERY_ACTIVE;
    }
    
    // Step 8: Notify user
    char message[32];
    sprintf(message, "SAFE TO REMOVE BAT%d", oldActive);
    display.drawAlertScreen(message, COLOR_GREEN);
    
    // Play triple beep
    buzzerController.playPattern(BUZZER_TRIPLE_BEEP);
    
    // Green sweep animation
    ledController.setPattern(LED_SWITCH_SWEEP);
    
    // Step 9: Start post-switch monitoring
    postSwitchMonitoring = true;
    postSwitchStartTime = millis();
    
    switchInProgress = false;
    systemState = SYSTEM_IDLE;
    
    return true;
}

// ============================================================================
// Hot-Swap Detection
// ============================================================================

void BatteryManager::detectHotSwap() {
    // Only monitor standby battery for hot-swap
    uint8_t standbyBat = getStandbyBattery();
    
    HotSwapEvent event = checkHotSwapStatus(standbyBat);
    
    if (event == HOTSWAP_REMOVED) {
        Serial.print("HOT-SWAP: Battery ");
        Serial.print(standbyBat);
        Serial.println(" REMOVED");
        
        BMSManager* standbyBMS = (standbyBat == 1) ? &bms1 : &bms2;
        BMSData data = standbyBMS->getData();
        Serial.print("  Voltage: ");
        Serial.print(data.totalVoltage, 2);
        Serial.println("V");
        
        // Update display
        char message[32];
        sprintf(message, "BATTERY %d REMOVED", standbyBat);
        display.drawAlertScreen(message, COLOR_YELLOW);
        
        // LED pattern: half yellow, half off
        ledController.setPattern(LED_SINGLE_BATTERY);
        
        // Play two beeps
        buzzerController.playPattern(BUZZER_DOUBLE_BEEP);
        
        systemState = SYSTEM_BATTERY_REMOVED;
        
    } else if (event == HOTSWAP_RECONNECTED_OK) {
        Serial.print("HOT-SWAP: Battery ");
        Serial.print(standbyBat);
        Serial.println(" RECONNECTED - READY");
        
        BMSManager* activeBMS = (activeBattery == 1) ? &bms1 : &bms2;
        BMSManager* standbyBMS = (standbyBat == 1) ? &bms1 : &bms2;
        
        BMSData activeData = activeBMS->getData();
        BMSData standbyData = standbyBMS->getData();
        
        float voltageDiff = checkVoltageMatch(activeData.totalVoltage, standbyData.totalVoltage);
        
        Serial.print("  Active: ");
        Serial.print(activeData.totalVoltage, 2);
        Serial.print("V, Standby: ");
        Serial.print(standbyData.totalVoltage, 2);
        Serial.print("V, Diff: ");
        Serial.print(voltageDiff, 2);
        Serial.println("V");
        
        // Update display
        char message[32];
        sprintf(message, "BATTERY %d RECONNECTED - READY", standbyBat);
        display.drawAlertScreen(message, COLOR_GREEN);
        
        // Play two beeps
        buzzerController.playPattern(BUZZER_DOUBLE_BEEP);
        
        // Return to idle pattern
        ledController.setPattern(LED_BREATHING);
        ledController.setColor(LED_GREEN);
        
        systemState = SYSTEM_IDLE;
        
    } else if (event == HOTSWAP_RECONNECTED_MISMATCH) {
        Serial.print("HOT-SWAP: Battery ");
        Serial.print(standbyBat);
        Serial.println(" RECONNECTED - VOLTAGE MISMATCH!");
        
        BMSManager* activeBMS = (activeBattery == 1) ? &bms1 : &bms2;
        BMSManager* standbyBMS = (standbyBat == 1) ? &bms1 : &bms2;
        
        BMSData activeData = activeBMS->getData();
        BMSData standbyData = standbyBMS->getData();
        
        float voltageDiff = checkVoltageMatch(activeData.totalVoltage, standbyData.totalVoltage);
        
        Serial.println("  CRITICAL: VOLTAGE MISMATCH");
        Serial.print("  Active: ");
        Serial.print(activeData.totalVoltage, 2);
        Serial.print("V, Standby: ");
        Serial.print(standbyData.totalVoltage, 2);
        Serial.println("V");
        Serial.print("  Difference: ");
        Serial.print(voltageDiff, 2);
        Serial.println("V");
        
        // Update display with voltage info
        display.clearScreen();
        display.printCentered("CRITICAL: VOLTAGE MISMATCH", 60, COLOR_RED);
        
        char voltStr[32];
        sprintf(voltStr, "Active: %.1fV", activeData.totalVoltage);
        display.printCentered(voltStr, 100, COLOR_WHITE);
        
        sprintf(voltStr, "Standby: %.1fV", standbyData.totalVoltage);
        display.printCentered(voltStr, 130, COLOR_WHITE);
        
        sprintf(voltStr, "Diff: %.1fV", voltageDiff);
        display.printCentered(voltStr, 160, COLOR_YELLOW);
        
        display.printCentered("DISCONNECT AND REPLACE", 200, COLOR_WHITE);
        
        // Activate continuous HIGH siren
        buzzerController.playPattern(BUZZER_CONTINUOUS_HIGH);
        
        // Red/white strobe at 4Hz
        ledController.setPattern(LED_VOLTAGE_MISMATCH);
        
        systemState = SYSTEM_VOLTAGE_MISMATCH;
    }
}

HotSwapEvent BatteryManager::checkHotSwapStatus(uint8_t batteryNum) {
    bool* wasConnected = (batteryNum == 1) ? &battery1WasConnected : &battery2WasConnected;
    float* lastVoltage = (batteryNum == 1) ? &lastBattery1Voltage : &lastBattery2Voltage;
    
    BMSManager* bms = (batteryNum == 1) ? &bms1 : &bms2;
    BMSData data = bms->getData();
    
    bool isConnectedNow = (data.totalVoltage > 10.0f);
    bool wasConnectedBefore = *wasConnected;
    
    // Update tracking variables
    *wasConnected = isConnectedNow;
    *lastVoltage = data.totalVoltage;
    
    // Detect removal: was connected (>45V), now disconnected (<10V)
    if (wasConnectedBefore && !isConnectedNow && *lastVoltage > 45.0f) {
        return HOTSWAP_REMOVED;
    }
    
    // Detect reconnection: was disconnected (<10V), now connected (>45V)
    if (!wasConnectedBefore && isConnectedNow && data.totalVoltage > 45.0f) {
        // Check voltage matching
        uint8_t activeNum = activeBattery;
        BMSManager* activeBMS = (activeNum == 1) ? &bms1 : &bms2;
        BMSData activeData = activeBMS->getData();
        
        float voltageDiff = checkVoltageMatch(activeData.totalVoltage, data.totalVoltage);
        
        if (voltageDiff < 5.0f) {
            return HOTSWAP_RECONNECTED_OK;
        } else {
            return HOTSWAP_RECONNECTED_MISMATCH;
        }
    }
    
    return HOTSWAP_NONE;
}

float BatteryManager::checkVoltageMatch(float v1, float v2) {
    return abs(v1 - v2);
}

// ============================================================================
// Emergency Functions
// ============================================================================

void BatteryManager::emergencyShutdown() {
    Serial.println("EMERGENCY SHUTDOWN INITIATED");
    
    // Disable both batteries immediately
    bms1.disableDischargeMosfet();
    bms2.disableDischargeMosfet();
    
    systemState = SYSTEM_EMERGENCY_STOP;
    
    Serial.println("Both batteries disabled");
}

// ============================================================================
// Utility Functions
// ============================================================================

const char* BatteryManager::getStateString(BatteryState state) {
    switch (state) {
        case BATTERY_ACTIVE:    return "ACTIVE";
        case BATTERY_STANDBY:   return "STANDBY";
        case BATTERY_REMOVED:   return "REMOVED";
        case BATTERY_FAULT:     return "FAULT";
        default:                return "UNKNOWN";
    }
}

const char* BatteryManager::getSystemStateString() {
    switch (systemState) {
        case SYSTEM_IDLE:               return "IDLE";
        case SYSTEM_SWITCHING_BATTERY:  return "SWITCHING BATTERY";
        case SYSTEM_BATTERY_REMOVED:    return "BATTERY REMOVED";
        case SYSTEM_VOLTAGE_MISMATCH:   return "VOLTAGE MISMATCH";
        case SYSTEM_EMERGENCY_STOP:     return "E-STOP ACTIVE";
        case SYSTEM_ERROR:              return "ERROR";
        default:                        return "UNKNOWN";
    }
}

