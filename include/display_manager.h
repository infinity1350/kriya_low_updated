/**
 * @file display_manager.h
 * @brief TFT_eSPI display management with smooth animations
 */

#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include "config.h"
#include "bms_manager.h"
#include "battery_manager.h"
#include "power_manager.h"

// Display modes
enum DisplayMode {
    DISPLAY_MAIN_SCREEN,
    DISPLAY_BATTERY1_DETAILS,
    DISPLAY_BATTERY2_DETAILS,
    DISPLAY_SYSTEM_INFO,
    DISPLAY_ERROR_SCREEN,
    DISPLAY_PRECHARGE_SCREEN
};

// Robot status from ROS (updated via ROS subscription)
struct RobotStatus {
    char mode[32];           // e.g., "IDLE", "NAVIGATING", "CHARGING"
    char waypoint[32];       // Current waypoint name
    char errorMsg[64];       // Error message if any
    bool hasError;
    uint8_t speedPercent;    // 0-100 speed
    bool isConnected;        // ROS connection status
    unsigned long lastUpdate;
};

// Animation state for smooth transitions
struct BatteryAnimState {
    float displayedSOC;      // Currently displayed SOC (animated)
    float targetSOC;         // Target SOC from BMS
    float displayedVoltage;  // Currently displayed voltage
    float targetVoltage;     // Target voltage from BMS
    bool needsUpdate;        // Flag for dirty region
};

class DisplayManager {
public:
    DisplayManager();
    
    void begin();
    void update();
    
    // Display modes
    void setMode(DisplayMode mode);
    DisplayMode getMode() const { return currentMode; }
    
    // Screen rendering
    void drawMainScreen();
    void drawPrechargeScreen(uint8_t progress);
    void drawEmergencyScreen();
    void drawBatteryDetails(uint8_t batteryNum);
    void drawSystemInfo();
    
    // UI Elements
    void drawBatteryGauge(uint16_t x, uint16_t y, uint8_t batteryNum, 
                         float soc, bool isActive, bool isCharging);
    
    // Robot status from ROS
    void updateRobotStatus(const char* mode, const char* waypoint, const char* error);
    void setRobotConnected(bool connected);
    
    // Text helpers
    void printCentered(const char* text, uint16_t y, uint16_t color = TFT_WHITE);
    
    // Alert screens
    void drawAlertScreen(const char* message, uint16_t bgColor);
    
    // Utility
    void clearScreen();
    
    // Color helpers
    uint16_t getColorForSOC(float soc);
    uint16_t getColorForTemperature(float temp);
    
private:
    TFT_eSPI tft;
    DisplayMode currentMode;
    DisplayMode lastMode;
    unsigned long lastUpdate;
    bool needsFullRedraw;
    
    // Robot status (from ROS)
    RobotStatus robotStatus;
    
    // Animation states for smooth transitions
    BatteryAnimState bat1Anim;
    BatteryAnimState bat2Anim;
    
    // Previous values for dirty-region detection
    float lastBat1SOC;
    float lastBat2SOC;
    float lastBat1Voltage;
    float lastBat2Voltage;
    float lastBat1Current;
    float lastBat2Current;
    char lastRobotMode[32];
    char lastPowerState[16];
    
    // Layout constants
    static const uint16_t MARGIN = 5;
    static const uint16_t HEADER_HEIGHT = 35;
    static const uint16_t FOOTER_HEIGHT = 25;
    static const uint16_t BATTERY_BOX_WIDTH = 145;
    static const uint16_t BATTERY_BOX_HEIGHT = 165;
    static const uint16_t GAUGE_WIDTH = 80;
    static const uint16_t GAUGE_HEIGHT = 100;
    
    // Helper functions
    void drawHeader(const char* statusText);
    void drawFooter();
    void drawMinimalBattery(uint16_t x, uint16_t y, uint8_t batNum, 
                            float soc, BMSData& data, bool isActive);
    void drawBatteryCard(uint16_t x, uint16_t y, uint8_t batNum, 
                         float soc, BMSData& data, bool isActive);
    
    // Animation helpers
    void updateAnimations();
    float lerp(float current, float target, float speed);
    void animateBatteryLevel(BatteryAnimState& anim, float targetSOC, float targetVoltage);
    
    // Animation state
    uint8_t animationFrame;
    unsigned long lastAnimationUpdate;
    uint8_t chargingAnimFrame;
    unsigned long lastChargingAnimUpdate;
};

// Global display instance (defined in main.cpp)
extern DisplayManager display;

#endif // DISPLAY_MANAGER_H
