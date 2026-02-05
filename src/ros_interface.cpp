/**
 * @file ros_interface.cpp
 * @brief ROS serial interface implementation
 */

#include "ros_interface.h"

// External instances
extern BMSManager bms1;
extern BMSManager bms2;
extern BatteryManager batteryManager;
extern SafetyMonitor safetyMonitor;
extern DisplayManager display;

// Static instance pointer for callbacks
static ROSInterface* rosInstance = nullptr;

// ============================================================================
// Constructor
// ============================================================================

ROSInterface::ROSInterface() :
    brakeStatePub("/brake_state", &brakeMsg),
    actionButtonPub("/action_button", &actionMsg),
    trolleyHitchPub("/trolley_hitch", &trolleyMsg),
    batteryStatusPub("/battery_status", &batteryMsg),
    safetyStatusPub("/safety_status", &safetyMsg),
    robotStatusSub("/robot_status", &ROSInterface::robotStatusCallback),
    buttonLedSub("/button_leds", &ROSInterface::buttonLedCallback)
{
    lastBatteryPublish = 0;
    lastSafetyPublish = 0;
    lastSpinTime = 0;
    
    lastBrakeState = false;
    lastActionState = false;
    lastTrolleyState = false;
    
    rosConnected = false;
    lastConnectionCheck = 0;
    
    memset(batteryData, 0, sizeof(batteryData));
    memset(safetyBuffer, 0, sizeof(safetyBuffer));
    
    rosInstance = this;
}

// ============================================================================
// Initialization
// ============================================================================

void ROSInterface::begin() {
    // Small delay to ensure USB CDC is fully enumerated
    delay(500);

    // Initialize ROS node handle
    nh.initNode();

    // Small delay after node initialization
    delay(100);

    // Advertise publishers
    nh.advertise(brakeStatePub);
    nh.advertise(actionButtonPub);
    nh.advertise(trolleyHitchPub);
    nh.advertise(batteryStatusPub);
    nh.advertise(safetyStatusPub);

    // Subscribe to topics
    nh.subscribe(robotStatusSub);
    nh.subscribe(buttonLedSub);

    // Setup battery message array
    batteryMsg.data_length = BAT_STATUS_SIZE;
    batteryMsg.data = batteryData;

    // Wait for connection with longer timeout
    unsigned long startTime = millis();
    while (!nh.connected() && (millis() - startTime < 5000)) {
        nh.spinOnce();
        delay(50);
    }

    rosConnected = nh.connected();

    if (rosConnected) {
        nh.loginfo("STM32 BMS System connected to ROS");
    } else {
        // Connection failed - will retry during update() calls
        // This is normal if rosserial host is not running yet
    }
}

// ============================================================================
// Main Update Function
// ============================================================================

void ROSInterface::update() {
    unsigned long now = millis();
    
    // Spin ROS
    spinOnce();
    
    // Check for button state changes
    checkButtonChanges();
    
    // Publish battery status at configured rate
    if (now - lastBatteryPublish >= ROS_BATTERY_PUBLISH_MS) {
        lastBatteryPublish = now;
        publishBatteryStatus();
    }
    
    // Publish safety status at configured rate
    if (now - lastSafetyPublish >= ROS_SAFETY_PUBLISH_MS) {
        lastSafetyPublish = now;
        publishSafetyStatus();
    }
    
    // Update connection status periodically
    if (now - lastConnectionCheck >= 1000) {
        lastConnectionCheck = now;
        rosConnected = nh.connected();
        display.setRobotConnected(rosConnected);
    }
}

void ROSInterface::spinOnce() {
    nh.spinOnce();
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
// Publishers
// ============================================================================

void ROSInterface::publishBrakeState(bool state) {
    brakeMsg.data = state;
    brakeStatePub.publish(&brakeMsg);
}

void ROSInterface::publishActionButton(bool state) {
    actionMsg.data = state;
    actionButtonPub.publish(&actionMsg);
}

void ROSInterface::publishTrolleyHitch(bool state) {
    trolleyMsg.data = state;
    trolleyHitchPub.publish(&trolleyMsg);
}

void ROSInterface::publishBatteryStatus() {
    BMSData data1 = bms1.getData();
    BMSData data2 = bms2.getData();
    
    // Battery 1 data
    batteryData[0] = data1.totalVoltage;
    batteryData[1] = data1.current;
    batteryData[2] = (float)data1.socPercent;
    batteryData[3] = data1.temp1;
    batteryData[4] = data1.temp2;
    batteryData[5] = (float)batteryManager.getBattery1State();
    
    // Battery 2 data
    batteryData[6] = data2.totalVoltage;
    batteryData[7] = data2.current;
    batteryData[8] = (float)data2.socPercent;
    batteryData[9] = data2.temp1;
    batteryData[10] = data2.temp2;
    batteryData[11] = (float)batteryManager.getBattery2State();
    
    // System state
    batteryData[12] = (float)batteryManager.getActiveBattery();
    batteryData[13] = (float)batteryManager.getSystemState();
    
    batteryStatusPub.publish(&batteryMsg);
}

void ROSInterface::publishSafetyStatus() {
    // Build safety status string
    // Format: "ESTOP:0,B1:0,B2:0,B3:0,ALERT:0"
    // B1=Brake, B2=Action, B3=Trolley
    sprintf(safetyBuffer, "ESTOP:%d,B1:%d,B2:%d,B3:%d,ALERT:%d",
            safetyMonitor.isEStopPressed() ? 1 : 0,
            safetyMonitor.isButton1Pressed() ? 1 : 0,
            safetyMonitor.isButton2Pressed() ? 1 : 0,
            safetyMonitor.isButton3Pressed() ? 1 : 0,
            (int)safetyMonitor.getCurrentAlertLevel());
    
    safetyMsg.data = safetyBuffer;
    safetyStatusPub.publish(&safetyMsg);
}

// ============================================================================
// Subscriber Callbacks
// ============================================================================

void ROSInterface::robotStatusCallback(const std_msgs::String& msg) {
    // Parse robot status message
    // Expected format: "MODE:NAVIGATING,WP:Waypoint_1,ERR:"
    
    if (rosInstance == nullptr) return;
    
    const char* data = msg.data;
    
    char mode[32] = "UNKNOWN";
    char waypoint[32] = "--";
    char error[64] = "";
    
    // Simple parsing
    const char* modePtr = strstr(data, "MODE:");
    if (modePtr != nullptr) {
        modePtr += 5;
        const char* endPtr = strchr(modePtr, ',');
        if (endPtr != nullptr) {
            size_t len = endPtr - modePtr;
            if (len < sizeof(mode)) {
                strncpy(mode, modePtr, len);
                mode[len] = '\0';
            }
        }
    }
    
    const char* wpPtr = strstr(data, "WP:");
    if (wpPtr != nullptr) {
        wpPtr += 3;
        const char* endPtr = strchr(wpPtr, ',');
        if (endPtr != nullptr) {
            size_t len = endPtr - wpPtr;
            if (len < sizeof(waypoint)) {
                strncpy(waypoint, wpPtr, len);
                waypoint[len] = '\0';
            }
        } else {
            // WP is last field
            strncpy(waypoint, wpPtr, sizeof(waypoint) - 1);
        }
    }
    
    const char* errPtr = strstr(data, "ERR:");
    if (errPtr != nullptr) {
        errPtr += 4;
        strncpy(error, errPtr, sizeof(error) - 1);
    }
    
    // Update display
    display.updateRobotStatus(mode, waypoint, error);
}

void ROSInterface::buttonLedCallback(const std_msgs::UInt8& msg) {
    // Control button LEDs via bitmask
    // Bit 0: Button 1 LED
    // Bit 1: Button 2 LED
    // Bit 2: Button 3 LED
    
    uint8_t ledMask = msg.data;
    
    safetyMonitor.setButton1LED(ledMask & 0x01);
    safetyMonitor.setButton2LED(ledMask & 0x02);
    safetyMonitor.setButton3LED(ledMask & 0x04);
}

