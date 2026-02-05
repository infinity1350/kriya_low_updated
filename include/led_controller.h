/**
 * @file led_controller.h
 * @brief WS2812B LED strip control using Adafruit NeoPixel library
 */

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"

// LED Pattern Types
enum LEDPattern {
    LED_OFF,
    LED_SOLID,
    LED_BREATHING,
    LED_RAINBOW,
    LED_CHASE,
    LED_THEATER_CHASE,
    LED_FADE,
    LED_SPARKLE,
    LED_RUNNING_LIGHTS,
    LED_COLOR_WIPE,
    LED_SCANNER,
    LED_BATTERY_STATUS,
    LED_WARNING_FLASH,
    LED_ERROR_PULSE,
    LED_SWITCH_SWEEP,        // Green sweep for successful switch
    LED_SINGLE_BATTERY,      // Half yellow, half off
    LED_VOLTAGE_MISMATCH     // Red/white strobe at 4Hz
};

// LED Color structure
struct LEDColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

// Predefined colors
const LEDColor LED_RED = {255, 0, 0};
const LEDColor LED_GREEN = {0, 255, 0};
const LEDColor LED_BLUE = {0, 0, 255};
const LEDColor LED_YELLOW = {255, 255, 0};
const LEDColor LED_CYAN = {0, 255, 255};
const LEDColor LED_MAGENTA = {255, 0, 255};
const LEDColor LED_WHITE = {255, 255, 255};
const LEDColor LED_ORANGE = {255, 165, 0};
const LEDColor LED_PURPLE = {128, 0, 128};
const LEDColor LED_PINK = {255, 192, 203};

class LEDController {
public:
    LEDController();
    ~LEDController();
    
    void begin();
    void update();
    
    // Pattern control
    void setPattern(LEDPattern pattern);
    void setColor(LEDColor color);
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void setBrightness(uint8_t brightness);  // 0-255
    void setSpeed(uint8_t speed);            // 0-255 (higher = faster)
    
    // Direct LED control
    void setLED(uint8_t index, LEDColor color);
    void setLED(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
    void setRange(uint8_t start, uint8_t end, LEDColor color);
    void clear();
    void fill(LEDColor color);
    void show();
    
    // Status indication
    void showBatteryStatus(float battery1SOC, float battery2SOC);
    void showWarning(bool active);
    void showError(bool active);
    void showIdlePattern();
    void showActivePattern();
    
    // Animation control
    void pause();
    void resume();
    bool isPaused() const { return paused; }
    
    // Getters
    LEDPattern getCurrentPattern() const { return currentPattern; }
    LEDColor getCurrentColor() const { return currentColor; }
    uint8_t getBrightness() const { return brightness; }
    
private:
    Adafruit_NeoPixel* strip;
    LEDPattern currentPattern;
    LEDColor currentColor;
    uint8_t brightness;
    uint8_t speed;
    bool paused;
    
    // Animation state
    unsigned long lastUpdate;
    uint16_t patternStep;
    uint16_t hueOffset;
    
    // Pattern implementations
    void patternOff();
    void patternSolid();
    void patternBreathing();
    void patternRainbow();
    void patternChase();
    void patternScanner();
    void patternBatteryStatus();
    void patternWarningFlash();
    void patternErrorPulse();
    void patternSwitchSweep();
    void patternSingleBattery();
    void patternVoltageMismatch();
    
    // Helper functions
    uint32_t colorToPackedRGB(LEDColor color);
    uint32_t colorToPackedRGB(uint8_t r, uint8_t g, uint8_t b);
    uint8_t calculateDelay();
    uint32_t wheel(uint8_t wheelPos);
    uint8_t sine8(uint8_t x);
    uint32_t colorHSV(uint16_t hue, uint8_t sat, uint8_t val);
};

// Global LED controller instance (defined in main.cpp)
extern LEDController ledController;

#endif // LED_CONTROLLER_H
