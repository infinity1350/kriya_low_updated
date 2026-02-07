/**
 * @file main.cpp
 * @brief Main application for STM32F407VET6 Battery Management System - Phase 2
 * 
 * Features:
 * - Dual JBD BMS communication
 * - Pre-charge circuit with configurable timing
 * - Contactor control (mutually exclusive with pre-charge)
 * - Emergency stop with interrupt handling
 * - Hot-swap battery switching (make-before-break)
 * - TFT_eSPI display with smooth animations
 * - WS2812B LED strip (84 LEDs) using Adafruit NeoPixel
 * - Siren/Buzzer control
 * - ROS serial communication via USB CDC
 * - Three buttons: Brake, Action, Trolley Hitch
 */

#include <Arduino.h>
#include <SPI.h>
#include "config.h"
#include "bms_manager.h"
#include "battery_manager.h"
#include "display_manager.h"
#include "led_controller.h"
#include "buzzer_controller.h"
#include "safety_monitor.h"
#include "power_manager.h"
#include "ros_interface.h"

// ============================================================================
// UART Serial Instances
// ============================================================================
// Use custom names to avoid conflict with framework's Serial1/Serial2
HardwareSerial BMS_Serial1(UART1_RX_PIN, UART1_TX_PIN);  // Battery #1
HardwareSerial BMS_Serial2(UART2_RX_PIN, UART2_TX_PIN);  // Battery #2

// ============================================================================
// Global Object Instances
// ============================================================================
BMSManager bms1(&BMS_Serial1, 1);
BMSManager bms2(&BMS_Serial2, 2);
BatteryManager batteryManager;
DisplayManager display;
LEDController ledController;
BuzzerController buzzerController;
SafetyMonitor safetyMonitor;
PowerManager powerManager;
ROSInterface rosInterface;

// ============================================================================
// Task Timing Variables
// ============================================================================
unsigned long lastROS = 0;
unsigned long lastSafety = 0;
unsigned long lastBMS = 0;
unsigned long lastBattery = 0;
unsigned long lastDisplay = 0;
unsigned long lastLED = 0;
unsigned long lastHeartbeat = 0;

// ============================================================================
// System State
// ============================================================================
bool systemInitialized = false;
bool previousEmergencyState = false;

// Serial print state tracking
bool lastBrakePrintState = false;
bool lastActionPrintState = false;
bool lastTrolleyPrintState = false;
unsigned long lastBatteryPrint = 0;
unsigned long lastSafetyPrint = 0;

// ============================================================================
// Function Prototypes
// ============================================================================
void setupPins();
void setupPeripherals();
void handleEmergencyTransitions();
void handleButtons();
void readBMSDirect(HardwareSerial& serial, BMSManager& bms);
void disableChargingMosfets();

// ============================================================================
// Setup Function
// ============================================================================
void setup() {
    // IMPORTANT: Always initialize Serial for USB CDC
    // - In DEBUG_MODE: Used for debug output
    // - In ROS MODE: Used by rosserial for communication
    Serial.begin(115200);

    // Critical delay for USB CDC enumeration on STM32
    // Without this, rosserial sync can fail
    delay(1000);

    #if DEBUG_MODE
        Serial.println("\n\n========================================");
        Serial.println("[BOOT] KRIYA BMS Starting (DEBUG MODE)");
        Serial.println("========================================");
    #endif
    DEBUG_PRINTLN("[SETUP] Initial delay complete");
    
    // Setup hardware pins first
    DEBUG_PRINTLN("[SETUP] Setting up pins...");
    setupPins();
    DEBUG_PRINTLN("[SETUP] Pins configured OK");
    
    // Initialize peripherals
    DEBUG_PRINTLN("[SETUP] Initializing peripherals...");
    setupPeripherals();
    DEBUG_PRINTLN("[SETUP] Peripherals initialized OK");
    
    // Check if emergency stop is active at startup
    #if DEBUG_MODE
        Serial.print("[SETUP] E-Stop state: ");
        Serial.println(safetyMonitor.isEStopPressed() ? "ACTIVE" : "NOT ACTIVE");
    #endif
    
    if (safetyMonitor.isEStopPressed()) {
        // Emergency active at startup - stay disabled
        DEBUG_PRINTLN("[SETUP] Emergency active - staying disabled");
        powerManager.emergencyStop();
        ledController.setPattern(LED_ERROR_PULSE);
        display.drawAlertScreen("E-STOP ACTIVE", COLOR_RED);
    } else {
        // Normal startup - begin pre-charge sequence
        DEBUG_PRINTLN("[SETUP] Starting pre-charge sequence...");
        powerManager.startPrecharge();
        DEBUG_PRINTLN("[SETUP] Pre-charge started OK");
        
        delay(100);  // Small delay after relay switch
        
        ledController.setColor(LED_ORANGE);
        ledController.setPattern(LED_BREATHING);
        DEBUG_PRINTLN("[SETUP] LED configured OK");
    }
    
    // Track initial emergency state
    previousEmergencyState = safetyMonitor.isEStopPressed();
    
    systemInitialized = true;
    DEBUG_PRINTLN("[SETUP] ========== SETUP COMPLETE ==========");
    DEBUG_PRINTLN("[LOOP] Entering main loop...\n");
}

// ============================================================================
// Main Loop - Task Scheduler
// ============================================================================
void loop() {
    unsigned long now = millis();
    
    #if DEBUG_MODE
    static unsigned long lastDebugPrint = 0;
    if (now - lastDebugPrint >= 2000) {
        lastDebugPrint = now;
        Serial.print("[LOOP] Power: ");
        Serial.print(powerManager.getStateString());
        Serial.print(" | BMS1 SOC: ");
        Serial.print(bms1.getData().socPercent);
        Serial.print("% | BMS2 SOC: ");
        Serial.print(bms2.getData().socPercent);
        Serial.println("%");
    }
    #endif
    
    // =========================================================================
    // PRIORITY 1: Emergency and Power Management
    // =========================================================================
    handleEmergencyTransitions();
    powerManager.update();
    
    // =========================================================================
    // PRIORITY 2: Safety Monitoring (100Hz)
    // =========================================================================
    if (now - lastSafety >= SAFETY_CHECK_INTERVAL_MS) {
        lastSafety = now;
        safetyMonitor.update();
        handleButtons();
        safetyMonitor.clearChangedFlags();
    }
    
    // =========================================================================
    // PRIORITY 3: Serial Communication (100Hz)
    // =========================================================================
    if (now - lastROS >= ROS_SPIN_INTERVAL_MS) {
        lastROS = now;
        rosInterface.update();
    }

    // =========================================================================
    // Serial Print Data (10Hz for battery/safety, on-change for buttons)
    // =========================================================================
    // Print button states on change
    bool currentBrake = safetyMonitor.isButton1Pressed();
    if (currentBrake != lastBrakePrintState) {
        Serial.print("BRAKE:");
        Serial.println(currentBrake ? 1 : 0);
        lastBrakePrintState = currentBrake;
    }

    bool currentAction = safetyMonitor.isButton2Pressed();
    if (currentAction != lastActionPrintState) {
        Serial.print("ACTION:");
        Serial.println(currentAction ? 1 : 0);
        lastActionPrintState = currentAction;
    }

    bool currentTrolley = safetyMonitor.isButton3Pressed();
    if (currentTrolley != lastTrolleyPrintState) {
        Serial.print("TROLLEY:");
        Serial.println(currentTrolley ? 1 : 0);
        lastTrolleyPrintState = currentTrolley;
    }

    // Print battery status at 10Hz (100ms)
    if (now - lastBatteryPrint >= 100) {
        lastBatteryPrint = now;

        BMSData data1 = bms1.getData();
        BMSData data2 = bms2.getData();

        Serial.print("BATTERY:");
        // Battery 1
        Serial.print(data1.totalVoltage, 2); Serial.print(",");
        Serial.print(data1.current, 2); Serial.print(",");
        Serial.print(data1.socPercent); Serial.print(",");
        Serial.print(data1.temp1, 1); Serial.print(",");
        Serial.print(data1.temp2, 1); Serial.print(",");
        Serial.print(batteryManager.getBattery1State()); Serial.print(",");
        // Battery 2
        Serial.print(data2.totalVoltage, 2); Serial.print(",");
        Serial.print(data2.current, 2); Serial.print(",");
        Serial.print(data2.socPercent); Serial.print(",");
        Serial.print(data2.temp1, 1); Serial.print(",");
        Serial.print(data2.temp2, 1); Serial.print(",");
        Serial.print(batteryManager.getBattery2State()); Serial.print(",");
        // System state
        Serial.print(batteryManager.getActiveBattery()); Serial.print(",");
        Serial.println(batteryManager.getSystemState());
    }

    // Print safety status at 10Hz (100ms)
    if (now - lastSafetyPrint >= 100) {
        lastSafetyPrint = now;

        Serial.print("SAFETY:");
        Serial.print(safetyMonitor.isEStopPressed() ? 1 : 0); Serial.print(",");
        Serial.print(safetyMonitor.isButton1Pressed() ? 1 : 0); Serial.print(",");
        Serial.print(safetyMonitor.isButton2Pressed() ? 1 : 0); Serial.print(",");
        Serial.print(safetyMonitor.isButton3Pressed() ? 1 : 0); Serial.print(",");
        Serial.println((int)safetyMonitor.getCurrentAlertLevel());
    }

    // =========================================================================
    // BMS Communication - every 2 seconds
    // =========================================================================
    if (now - lastBMS >= BMS_UPDATE_INTERVAL_MS) {
        lastBMS = now;
        digitalWrite(ONBOARD_LED_PIN, HIGH);
        
        DEBUG_PRINTLN("[BMS] Reading BMS1...");
        readBMSDirect(BMS_Serial1, bms1);
        DEBUG_PRINTLN("[BMS] BMS1 done");
        
        DEBUG_PRINTLN("[BMS] Reading BMS2...");
        readBMSDirect(BMS_Serial2, bms2);
        DEBUG_PRINTLN("[BMS] BMS2 done");
        
        digitalWrite(ONBOARD_LED_PIN, LOW);
    }


    
    // =========================================================================
    // Battery Management Logic (20Hz) - Only when system is ready
    // =========================================================================
    if (powerManager.isReady() && (now - lastBattery >= 50)) {
        lastBattery = now;
        batteryManager.updateBatteryStates();
        batteryManager.checkSwitchConditions();
        batteryManager.detectHotSwap();
    }
    
    // =========================================================================
    // Display Update (30Hz)
    // =========================================================================
    if (now - lastDisplay >= LED_UPDATE_INTERVAL_MS) {
        lastDisplay = now;
        display.update();
    }
    
    // =========================================================================
    // LED Update (30Hz)
    // =========================================================================
    if (now - lastLED >= LED_UPDATE_INTERVAL_MS) {
        lastLED = now;
        if (safetyMonitor.isEStopPressed()) {
            ledController.fill(LED_RED);
            ledController.show();
        } else if (powerManager.isPrecharging()) {
            ledController.update();
        } else if (powerManager.isReady()) {
            ledController.update();
        }
    }
    
    // =========================================================================
    // Heartbeat (1Hz)
    // =========================================================================
    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeat = now;
        safetyMonitor.updateHeartbeat();
    }
    
    // =========================================================================
    // Buzzer (runs every loop)
    // =========================================================================
    buzzerController.update();
}

// ============================================================================
// Setup Functions
// ============================================================================
void setupPins() {
    // Pre-charge and contactor pins (handled by PowerManager, but set safe defaults)
    pinMode(PRECHARGE_PIN, OUTPUT);
    pinMode(CONTACTOR_PIN, OUTPUT);
    digitalWrite(PRECHARGE_PIN, LOW);
    digitalWrite(CONTACTOR_PIN, LOW);
    
    // Onboard LED
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    digitalWrite(ONBOARD_LED_PIN, LOW);
}

void setupPeripherals() {
    DEBUG_PRINTLN("  [PERIPH] SPI.begin()...");
    SPI.begin();
    DEBUG_PRINTLN("  [PERIPH] SPI OK");
    
    DEBUG_PRINTLN("  [PERIPH] display.begin()...");
    display.begin();
    DEBUG_PRINTLN("  [PERIPH] Display OK");
    
    DEBUG_PRINTLN("  [PERIPH] BMS UART init...");
    bms1.begin();
    bms2.begin();
    DEBUG_PRINTLN("  [PERIPH] BMS UART OK");
    
    // IMPORTANT: Disable charging MOSFETs on both batteries at startup
    disableChargingMosfets();
    
    DEBUG_PRINTLN("  [PERIPH] batteryManager.begin()...");
    batteryManager.begin();
    DEBUG_PRINTLN("  [PERIPH] Battery manager OK");
    
    DEBUG_PRINTLN("  [PERIPH] ledController.begin()...");
    ledController.begin();
    ledController.setBrightness(LED_BRIGHTNESS);
    DEBUG_PRINTLN("  [PERIPH] LED controller OK");
    
    DEBUG_PRINTLN("  [PERIPH] buzzerController.begin()...");
    buzzerController.begin();
    DEBUG_PRINTLN("  [PERIPH] Buzzer OK");
    
    DEBUG_PRINTLN("  [PERIPH] safetyMonitor.begin()...");
    safetyMonitor.begin();
    DEBUG_PRINTLN("  [PERIPH] Safety monitor OK");
    
    DEBUG_PRINTLN("  [PERIPH] powerManager.begin()...");
    powerManager.begin();
    DEBUG_PRINTLN("  [PERIPH] Power manager OK");

    // Serial interface - always initialize
    DEBUG_PRINTLN("  [PERIPH] Serial interface starting...");
    rosInterface.begin();
    DEBUG_PRINTLN("  [PERIPH] Serial interface OK");
}

// ============================================================================
// Emergency Transition Handler
// ============================================================================
void handleEmergencyTransitions() {
    bool currentEmergency = safetyMonitor.isEStopPressed();
    
    // Detect emergency stop activation
    if (currentEmergency && !previousEmergencyState) {
        // Emergency just activated
        powerManager.emergencyStop();
        ledController.setPattern(LED_ERROR_PULSE);
        buzzerController.playEStopSound();
    }
    
    // Detect emergency stop release
    if (!currentEmergency && previousEmergencyState) {
        // Emergency just released - start pre-charge sequence
        powerManager.reset();
        powerManager.startPrecharge();
        
        ledController.setColor(LED_ORANGE);
        ledController.setPattern(LED_BREATHING);
        buzzerController.stop();
    }
    
    // Update state for next iteration
    previousEmergencyState = currentEmergency;
}

// ============================================================================
// Button Handler
// ============================================================================
void handleButtons() {
    // Button 1: Brake - LED follows button state, published to ROS
    if (safetyMonitor.button1Changed()) {
        safetyMonitor.setButton1LED(safetyMonitor.isButton1Pressed());
        if (safetyMonitor.button1RisingEdge()) {
            buzzerController.playButtonPressSound();
        }
    }
    
    // Button 2: Action (Next Waypoint) - Toggle LED, published to ROS
    static bool button2Toggle = false;
    if (safetyMonitor.button2RisingEdge()) {
        button2Toggle = !button2Toggle;
        safetyMonitor.setButton2LED(button2Toggle);
        buzzerController.playButtonPressSound();
    }
    
    // Button 3: Trolley Hitch - Toggle LED, published to ROS
    static bool button3Toggle = false;
    if (safetyMonitor.button3RisingEdge()) {
        button3Toggle = !button3Toggle;
        safetyMonitor.setButton3LED(button3Toggle);
        buzzerController.playButtonPressSound();
    }
}

// ============================================================================
// Direct BMS Read (Delay-based, works reliably)
// ============================================================================
void readBMSDirect(HardwareSerial& serial, BMSManager& bms) {
    // Flush any pending data with timeout
    unsigned long flushStart = millis();
    for (int i = 0; i < 256 && serial.available() && (millis() - flushStart < 50); i++) {
        serial.read();
    }


    // Build and send JBD BMS frame for basic info (cmd 0x03)
    uint8_t frame[7] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
    serial.write(frame, 7);

    // Wait for response (delay-based is more reliable than millis loop)
    delay(100);

    // Read response with timeout protection
    uint8_t buffer[64];
    uint8_t idx = 0;
    unsigned long startTime = millis();
    const unsigned long TIMEOUT_MS = 100;  // 100ms timeout

    // Read available bytes with timeout and iteration limit
    while (idx < 64 && (millis() - startTime < TIMEOUT_MS)) {
        if (serial.available()) {
            buffer[idx++] = serial.read();
        }
    }

    // Update BMS data if we got a valid response
    DEBUG_PRINT("[BMS] Received ");
    DEBUG_PRINT(idx);
    DEBUG_PRINTLN(" bytes");

    if (idx >= 30) {
        DEBUG_PRINTLN("[BMS] Updating buffer...");
        bms.updateFromBuffer(buffer, idx);
        DEBUG_PRINTLN("[BMS] Buffer updated");
    } else {
        DEBUG_PRINTLN("[BMS] Not enough data");
    }
}

// ============================================================================
// Disable Charging MOSFETs at Startup
// ============================================================================
void disableChargingMosfets() {
    DEBUG_PRINTLN("  [SETUP] Checking and disabling charging MOSFETs...");
    
    // Read BMS status first to check current MOSFET state
    readBMSDirect(BMS_Serial1, bms1);
    readBMSDirect(BMS_Serial2, bms2);
    
    BMSData data1 = bms1.getData();
    BMSData data2 = bms2.getData();
    
    // Disable charging MOSFET on BMS1 if it's enabled
    if (data1.dataValid && data1.chargeMosfetOn) {
        DEBUG_PRINTLN("  [SETUP] BMS1 charging MOSFET is ON - disabling...");
        bms1.disableChargeMosfet();
        delay(100);
    }
    
    // Disable charging MOSFET on BMS2 if it's enabled
    if (data2.dataValid && data2.chargeMosfetOn) {
        DEBUG_PRINTLN("  [SETUP] BMS2 charging MOSFET is ON - disabling...");
        bms2.disableChargeMosfet();
        delay(100);
    }
    
    // Always ensure charging is disabled on both batteries for safety
    bms1.disableChargeMosfet();
    delay(50);
    bms2.disableChargeMosfet();
    delay(50);
    
    DEBUG_PRINTLN("  [SETUP] Charging MOSFETs disabled on both batteries");
}
