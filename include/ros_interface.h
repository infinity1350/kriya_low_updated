/**
 * @file ros_interface.h
 * @brief Serial interface for communication with upper-level controller
 * Simple Serial.print() based - no ROS dependencies
 */

#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <Arduino.h>
#include "config.h"
#include "bms_manager.h"
#include "battery_manager.h"
#include "safety_monitor.h"
#include "display_manager.h"

// Publish rates
#define SERIAL_BATTERY_PUBLISH_MS    100   // 10Hz
#define SERIAL_SAFETY_PUBLISH_MS     100   // 10Hz
#define SERIAL_BUTTON_PUBLISH_MS     10    // On change, but max 100Hz

class ROSInterface {
public:
    ROSInterface();

    void begin();
    void update();

    // Serial communication - call frequently in main loop
    void spinOnce();

    // Publishers (now using Serial.print)
    void publishBatteryStatus();
    void publishSafetyStatus();
    void publishBrakeState(bool state);
    void publishActionButton(bool state);
    void publishTrolleyHitch(bool state);

    // State tracking for edge-triggered publishing
    void checkButtonChanges();

    // Process incoming serial commands
    void processSerialInput();

    // Connection status (always connected in serial mode)
    bool isConnected() const { return true; }

private:
    // Timing
    unsigned long lastBatteryPublish;
    unsigned long lastSafetyPublish;

    // Previous button states for edge detection
    bool lastBrakeState;
    bool lastActionState;
    bool lastTrolleyState;

    // Serial input buffer
    char inputBuffer[128];
    uint8_t inputIndex;
};

// Global serial interface instance
extern ROSInterface rosInterface;

#endif // ROS_INTERFACE_H

