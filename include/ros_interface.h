/**
 * @file ros_interface.h
 * @brief ROS serial interface for communication with upper-level controller
 */

#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <Arduino.h>

// Configure rosserial buffer sizes for STM32 BEFORE including ros.h
// Default is 512 bytes which can cause sync issues on STM32 USB CDC
// Increase to 1024 bytes for better reliability
#ifndef INPUT_SIZE
  #define INPUT_SIZE 1024
#endif

#ifndef OUTPUT_SIZE
  #define OUTPUT_SIZE 1024
#endif

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32MultiArray.h>

#include "config.h"
#include "bms_manager.h"
#include "battery_manager.h"
#include "safety_monitor.h"
#include "display_manager.h"

// ROS publish rates
#define ROS_BATTERY_PUBLISH_MS    50   // 20Hz
#define ROS_SAFETY_PUBLISH_MS     50   // 20Hz
#define ROS_BUTTON_PUBLISH_MS     10   // On change, but max 100Hz

// Battery status array indices
// [0-5] = BAT1: voltage, current, soc, temp1, temp2, state
// [6-11] = BAT2: voltage, current, soc, temp1, temp2, state
// [12] = active_battery
// [13] = system_state
#define BAT_STATUS_SIZE 14

class ROSInterface {
public:
    ROSInterface();
    
    void begin();
    void update();
    
    // ROS spin - call frequently in main loop
    void spinOnce();
    
    // Publishers
    void publishBatteryStatus();
    void publishSafetyStatus();
    void publishBrakeState(bool state);
    void publishActionButton(bool state);
    void publishTrolleyHitch(bool state);
    
    // State tracking for edge-triggered publishing
    void checkButtonChanges();
    
    // Connection status
    bool isConnected() const { return rosConnected; }
    
private:
    ros::NodeHandle nh;
    
    // Publishers
    ros::Publisher brakeStatePub;
    ros::Publisher actionButtonPub;
    ros::Publisher trolleyHitchPub;
    ros::Publisher batteryStatusPub;
    ros::Publisher safetyStatusPub;
    
    // Messages
    std_msgs::Bool brakeMsg;
    std_msgs::Bool actionMsg;
    std_msgs::Bool trolleyMsg;
    std_msgs::Float32MultiArray batteryMsg;
    std_msgs::String safetyMsg;
    
    // Subscribers
    ros::Subscriber<std_msgs::String> robotStatusSub;
    ros::Subscriber<std_msgs::UInt8> buttonLedSub;
    
    // Callback functions (must be static for rosserial)
    static void robotStatusCallback(const std_msgs::String& msg);
    static void buttonLedCallback(const std_msgs::UInt8& msg);
    
    // Timing
    unsigned long lastBatteryPublish;
    unsigned long lastSafetyPublish;
    unsigned long lastSpinTime;
    
    // Previous button states for edge detection
    bool lastBrakeState;
    bool lastActionState;
    bool lastTrolleyState;
    
    // Connection tracking
    bool rosConnected;
    unsigned long lastConnectionCheck;
    
    // Buffer for battery data
    float batteryData[BAT_STATUS_SIZE];
    
    // Safety message buffer
    char safetyBuffer[128];
};

// Global ROS interface instance
extern ROSInterface rosInterface;

#endif // ROS_INTERFACE_H

