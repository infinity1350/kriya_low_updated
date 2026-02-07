/**
 * @file ros_interface.cpp
 * @brief Serial interface implementation (no ROS dependencies)
 * Uses simple Serial.print() for data transmission
 */

#include "ros_interface.h"

// External instances
extern BMSManager bms1;
extern BMSManager bms2;
extern BatteryManager batteryManager;
extern SafetyMonitor safetyMonitor;
extern DisplayManager display;

// ============================================================================
// Constructor
// ============================================================================

ROSInterface::ROSInterface() {
    lastBatteryPublish = 0;
    lastSafetyPublish = 0;

    lastBrakeState = false;
    lastActionState = false;
    lastTrolleyState = false;

    inputIndex = 0;
    memset(inputBuffer, 0, sizeof(inputBuffer));
}

// ============================================================================
// Initialization
// ============================================================================

void ROSInterface::begin() {
    // Serial already initialized in main.cpp
    // Just print a startup message
    Serial.println("\n========================================");
    Serial.println("STM32 BMS Serial Interface Started");
    Serial.println("========================================");
    Serial.println("Data format:");
    Serial.println("  BRAKE:<0|1>");
    Serial.println("  ACTION:<0|1>");
    Serial.println("  TROLLEY:<0|1>");
    Serial.println("  BATTERY:V1,I1,SOC1,T1a,T1b,S1,V2,I2,SOC2,T2a,T2b,S2,ACTIVE,STATE");
    Serial.println("  SAFETY:ESTOP,B1,B2,B3,ALERT");
    Serial.println("========================================\n");
}

// ============================================================================
// Main Update Function
// ============================================================================

void ROSInterface::update() {
    unsigned long now = millis();

    // Process incoming serial commands
    spinOnce();

    // Check for button state changes
    checkButtonChanges();

    // Publish battery status at configured rate
    if (now - lastBatteryPublish >= SERIAL_BATTERY_PUBLISH_MS) {
        lastBatteryPublish = now;
        publishBatteryStatus();
    }

    // Publish safety status at configured rate
    if (now - lastSafetyPublish >= SERIAL_SAFETY_PUBLISH_MS) {
        lastSafetyPublish = now;
        publishSafetyStatus();
    }
}

void ROSInterface::spinOnce() {
    // Read incoming serial data
    processSerialInput();
}

// ============================================================================
// Button Change Detection
// ============================================================================

void ROSInterface::checkButtonChanges() {
    // Button 1 - Brake
    bool currentBrake = safetyMonitor.isButton1Pressed();
    if (currentBrake != lastBrakeState) {
        publishBrakeState(currentBrake);
        lastBrakeState = currentBrake;
    }

    // Button 2 - Action
    bool currentAction = safetyMonitor.isButton2Pressed();
    if (currentAction != lastActionState) {
        publishActionButton(currentAction);
        lastActionState = currentAction;
    }

    // Button 3 - Trolley Hitch
    bool currentTrolley = safetyMonitor.isButton3Pressed();
    if (currentTrolley != lastTrolleyState) {
        publishTrolleyHitch(currentTrolley);
        lastTrolleyState = currentTrolley;
    }
}

// ============================================================================
// Publishers (Serial.print based)
// ============================================================================

void ROSInterface::publishBrakeState(bool state) {
    Serial.print("BRAKE:");
    Serial.println(state ? 1 : 0);
}

void ROSInterface::publishActionButton(bool state) {
    Serial.print("ACTION:");
    Serial.println(state ? 1 : 0);
}

void ROSInterface::publishTrolleyHitch(bool state) {
    Serial.print("TROLLEY:");
    Serial.println(state ? 1 : 0);
}

void ROSInterface::publishBatteryStatus() {
    BMSData data1 = bms1.getData();
    BMSData data2 = bms2.getData();

    // Format: BATTERY:V1,I1,SOC1,T1a,T1b,S1,V2,I2,SOC2,T2a,T2b,S2,ACTIVE,STATE
    Serial.print("BATTERY:");

    // Battery 1 data
    Serial.print(data1.totalVoltage, 2);        Serial.print(",");
    Serial.print(data1.current, 2);             Serial.print(",");
    Serial.print(data1.socPercent);             Serial.print(",");
    Serial.print(data1.temp1, 1);               Serial.print(",");
    Serial.print(data1.temp2, 1);               Serial.print(",");
    Serial.print(batteryManager.getBattery1State()); Serial.print(",");

    // Battery 2 data
    Serial.print(data2.totalVoltage, 2);        Serial.print(",");
    Serial.print(data2.current, 2);             Serial.print(",");
    Serial.print(data2.socPercent);             Serial.print(",");
    Serial.print(data2.temp1, 1);               Serial.print(",");
    Serial.print(data2.temp2, 1);               Serial.print(",");
    Serial.print(batteryManager.getBattery2State()); Serial.print(",");

    // System state
    Serial.print(batteryManager.getActiveBattery()); Serial.print(",");
    Serial.println(batteryManager.getSystemState());
}

void ROSInterface::publishSafetyStatus() {
    // Format: SAFETY:ESTOP,B1,B2,B3,ALERT
    Serial.print("SAFETY:");
    Serial.print(safetyMonitor.isEStopPressed() ? 1 : 0);      Serial.print(",");
    Serial.print(safetyMonitor.isButton1Pressed() ? 1 : 0);    Serial.print(",");
    Serial.print(safetyMonitor.isButton2Pressed() ? 1 : 0);    Serial.print(",");
    Serial.print(safetyMonitor.isButton3Pressed() ? 1 : 0);    Serial.print(",");
    Serial.println((int)safetyMonitor.getCurrentAlertLevel());
}

// ============================================================================
// Serial Input Processing
// ============================================================================

void ROSInterface::processSerialInput() {
    while (Serial.available() > 0) {
        char c = Serial.read();

        // End of line - process command
        if (c == '\n' || c == '\r') {
            if (inputIndex > 0) {
                inputBuffer[inputIndex] = '\0';

                // Parse command
                // Expected formats:
                //   ROBOT:MODE,WAYPOINT,ERROR
                //   LED:bitmask (0-7, bits control button LEDs)

                if (strncmp(inputBuffer, "ROBOT:", 6) == 0) {
                    // Parse robot status: ROBOT:NAVIGATING,Waypoint_1,
                    char* data = inputBuffer + 6;

                    char mode[32] = "UNKNOWN";
                    char waypoint[32] = "--";
                    char error[64] = "";

                    // Parse comma-separated values
                    char* token = strtok(data, ",");
                    if (token != nullptr) {
                        strncpy(mode, token, sizeof(mode) - 1);
                    }

                    token = strtok(nullptr, ",");
                    if (token != nullptr) {
                        strncpy(waypoint, token, sizeof(waypoint) - 1);
                    }

                    token = strtok(nullptr, ",");
                    if (token != nullptr) {
                        strncpy(error, token, sizeof(error) - 1);
                    }

                    // Update display
                    display.updateRobotStatus(mode, waypoint, error);

                } else if (strncmp(inputBuffer, "LED:", 4) == 0) {
                    // Control button LEDs: LED:5 (binary 101 = button 1 and 3 on)
                    uint8_t ledMask = atoi(inputBuffer + 4);

                    safetyMonitor.setButton1LED(ledMask & 0x01);
                    safetyMonitor.setButton2LED(ledMask & 0x02);
                    safetyMonitor.setButton3LED(ledMask & 0x04);
                }

                // Reset buffer
                inputIndex = 0;
                memset(inputBuffer, 0, sizeof(inputBuffer));
            }
        }
        // Add character to buffer
        else if (inputIndex < sizeof(inputBuffer) - 1) {
            inputBuffer[inputIndex++] = c;
        }
    }
}
