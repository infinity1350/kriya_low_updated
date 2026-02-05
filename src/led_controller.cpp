/**
 * @file led_controller.cpp
 * @brief WS2812B LED strip control using Adafruit NeoPixel library
 */

#include "led_controller.h"

// ============================================================================
// Constructor and Initialization
// ============================================================================

LEDController::LEDController() {
    strip = nullptr;
    currentPattern = LED_OFF;
    currentColor = LED_GREEN;
    brightness = 128;
    speed = 128;
    paused = false;
    lastUpdate = 0;
    patternStep = 0;
    hueOffset = 0;
}

LEDController::~LEDController() {
    if (strip != nullptr) {
        delete strip;
        strip = nullptr;
    }
}

void LEDController::begin() {
    DEBUG_PRINTLN("    [LED] Creating NeoPixel object...");
    // Using NEO_KHZ400 for this specific LED strip
    strip = new Adafruit_NeoPixel(NUM_LEDS, LED_STRIP_PIN, NEO_GRB + NEO_KHZ400);
    
    DEBUG_PRINTLN("    [LED] strip->begin()...");
    strip->begin();
    strip->setBrightness(LED_BRIGHTNESS);
    clear();
    show();
    DEBUG_PRINTLN("    [LED] LED begin() complete");
}

// ============================================================================
// Main Update Function
// ============================================================================

void LEDController::update() {
    if (paused || strip == nullptr) {
        return;
    }
    
    unsigned long now = millis();
    uint8_t delayMs = calculateDelay();
    
    if (now - lastUpdate < delayMs) {
        return;
    }
    
    lastUpdate = now;
    
    // Execute current pattern
    switch (currentPattern) {
        case LED_OFF:
            patternOff();
            break;
        case LED_SOLID:
            patternSolid();
            break;
        case LED_BREATHING:
            patternBreathing();
            break;
        case LED_RAINBOW:
            patternRainbow();
            break;
        case LED_CHASE:
            patternChase();
            break;
        case LED_SCANNER:
            patternScanner();
            break;
        case LED_BATTERY_STATUS:
            patternBatteryStatus();
            break;
        case LED_WARNING_FLASH:
            patternWarningFlash();
            break;
        case LED_ERROR_PULSE:
            patternErrorPulse();
            break;
        case LED_SWITCH_SWEEP:
            patternSwitchSweep();
            break;
        case LED_SINGLE_BATTERY:
            patternSingleBattery();
            break;
        case LED_VOLTAGE_MISMATCH:
            patternVoltageMismatch();
            break;
        default:
            patternOff();
            break;
    }
    
    show();
}

// ============================================================================
// Pattern Control
// ============================================================================

void LEDController::setPattern(LEDPattern pattern) {
    if (currentPattern != pattern) {
        currentPattern = pattern;
        patternStep = 0;
        hueOffset = 0;
    }
}

void LEDController::setColor(LEDColor color) {
    currentColor = color;
}

void LEDController::setColor(uint8_t r, uint8_t g, uint8_t b) {
    currentColor.r = r;
    currentColor.g = g;
    currentColor.b = b;
}

void LEDController::setBrightness(uint8_t b) {
    brightness = b;
    if (strip != nullptr) {
        strip->setBrightness(b);
    }
}

void LEDController::setSpeed(uint8_t s) {
    speed = s;
}

// ============================================================================
// Direct LED Control
// ============================================================================

void LEDController::clear() {
    if (strip != nullptr) {
        strip->clear();
    }
}

void LEDController::fill(LEDColor color) {
    if (strip != nullptr) {
        uint32_t packedColor = colorToPackedRGB(color);
        strip->fill(packedColor, 0, NUM_LEDS);
    }
}

void LEDController::setLED(uint8_t index, LEDColor color) {
    if (strip != nullptr && index < NUM_LEDS) {
        strip->setPixelColor(index, colorToPackedRGB(color));
    }
}

void LEDController::setLED(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (strip != nullptr && index < NUM_LEDS) {
        strip->setPixelColor(index, strip->Color(r, g, b));
    }
}

void LEDController::setRange(uint8_t start, uint8_t end, LEDColor color) {
    if (strip != nullptr) {
        uint32_t packedColor = colorToPackedRGB(color);
        for (uint8_t i = start; i < end && i < NUM_LEDS; i++) {
            strip->setPixelColor(i, packedColor);
        }
    }
}

void LEDController::show() {
    if (strip != nullptr) {
        strip->show();
    }
}

// ============================================================================
// Pattern Implementations
// ============================================================================

void LEDController::patternOff() {
    clear();
}

void LEDController::patternSolid() {
    fill(currentColor);
}

void LEDController::patternBreathing() {
    // Breathing effect using sine wave
    uint8_t breath = sine8(patternStep);
    uint8_t val = map(breath, 0, 255, 30, 255);
    
    uint8_t r = (currentColor.r * val) / 255;
    uint8_t g = (currentColor.g * val) / 255;
    uint8_t b = (currentColor.b * val) / 255;
    
    if (strip != nullptr) {
        strip->fill(strip->Color(r, g, b), 0, NUM_LEDS);
    }
    
    patternStep += 2;
}

void LEDController::patternRainbow() {
    if (strip != nullptr) {
        for (uint16_t i = 0; i < NUM_LEDS; i++) {
            uint16_t hue = hueOffset + (i * 65536 / NUM_LEDS);
            strip->setPixelColor(i, colorHSV(hue, 255, 255));
        }
    }
    hueOffset += 256;
}

void LEDController::patternChase() {
    clear();
    
    if (strip != nullptr) {
        // Moving dot
        uint8_t pos = patternStep % NUM_LEDS;
        strip->setPixelColor(pos, colorToPackedRGB(currentColor));
        
        // Trail
        for (uint8_t i = 1; i < 5; i++) {
            uint8_t trailPos = (pos - i + NUM_LEDS) % NUM_LEDS;
            uint8_t fade = 255 - (i * 50);
            uint8_t r = (currentColor.r * fade) / 255;
            uint8_t g = (currentColor.g * fade) / 255;
            uint8_t b = (currentColor.b * fade) / 255;
            strip->setPixelColor(trailPos, strip->Color(r, g, b));
        }
    }
    
    patternStep++;
}

void LEDController::patternScanner() {
    // Knight Rider style scanner
    clear();
    
    if (strip != nullptr) {
        uint8_t pos = patternStep % (NUM_LEDS * 2);
        if (pos >= NUM_LEDS) {
            pos = (NUM_LEDS * 2 - 1) - pos;
        }
        
        strip->setPixelColor(pos, colorToPackedRGB(currentColor));
        
        // Trail
        for (uint8_t i = 1; i < 8; i++) {
            uint8_t fade = 255 - (i * 30);
            uint8_t r = (currentColor.r * fade) / 255;
            uint8_t g = (currentColor.g * fade) / 255;
            uint8_t b = (currentColor.b * fade) / 255;
            
            if (pos >= i) {
                strip->setPixelColor(pos - i, strip->Color(r, g, b));
            }
            if (pos + i < NUM_LEDS) {
                strip->setPixelColor(pos + i, strip->Color(r, g, b));
            }
        }
    }
    
    patternStep++;
}

void LEDController::patternBatteryStatus() {
    // This pattern displays current state - updated via showBatteryStatus()
    // Keep current LED state, don't clear
}

void LEDController::patternWarningFlash() {
    // Fast yellow/orange flash
    if ((patternStep % 2) == 0) {
        fill(LED_YELLOW);
    } else {
        clear();
    }
    patternStep++;
}

void LEDController::patternErrorPulse() {
    // Red pulsing
    uint8_t val = sine8(patternStep * 2);
    val = map(val, 0, 255, 50, 255);
    
    if (strip != nullptr) {
        strip->fill(strip->Color(val, 0, 0), 0, NUM_LEDS);
    }
    
    patternStep++;
}

void LEDController::patternSwitchSweep() {
    // Green sweep left to right for successful battery switch
    clear();
    
    if (strip != nullptr) {
        uint8_t pos = patternStep % (NUM_LEDS + 10);
        
        for (uint8_t i = 0; i < 10; i++) {
            int16_t ledPos = pos - i;
            if (ledPos >= 0 && ledPos < NUM_LEDS) {
                uint8_t brightness = 255 - (i * 25);
                strip->setPixelColor(ledPos, strip->Color(0, brightness, 0));
            }
        }
    }
    
    patternStep++;
    
    // Reset after one sweep
    if (patternStep >= NUM_LEDS + 10) {
        patternStep = 0;
        // After sweep completes, return to idle pattern
        setPattern(LED_BREATHING);
        setColor(LED_GREEN);
    }
}

void LEDController::patternSingleBattery() {
    // Half strip yellow (active battery), half off (removed battery)
    // Supports dynamic NUM_LEDS (84 LEDs = 42 per side)
    if (strip != nullptr) {
        const uint8_t HALF = NUM_LEDS / 2;
        for (uint8_t i = 0; i < HALF; i++) {
            strip->setPixelColor(i, colorToPackedRGB(LED_YELLOW));
        }
        for (uint8_t i = HALF; i < NUM_LEDS; i++) {
            strip->setPixelColor(i, 0);
        }
    }
}

void LEDController::patternVoltageMismatch() {
    // Red/white strobe at 4Hz (250ms period = 125ms on, 125ms off)
    // At ~33ms update rate, that's about 4 steps per state
    if ((patternStep / 4) % 2 == 0) {
        fill(LED_RED);
    } else {
        fill(LED_WHITE);
    }
    patternStep++;
}

// ============================================================================
// Status Indication Functions
// ============================================================================

void LEDController::showBatteryStatus(float battery1SOC, float battery2SOC) {
    if (strip == nullptr) return;
    
    // Split strip in half: 42 LEDs per battery (84 total)
    const uint8_t LEDS_PER_BATTERY = NUM_LEDS / 2;
    
    // Left half for Battery 1
    uint8_t led1Count = (battery1SOC * LEDS_PER_BATTERY) / 100;
    LEDColor color1 = (battery1SOC > 50) ? LED_GREEN : (battery1SOC > 20) ? LED_YELLOW : LED_RED;
    
    for (uint8_t i = 0; i < LEDS_PER_BATTERY; i++) {
        if (i < led1Count) {
            strip->setPixelColor(i, colorToPackedRGB(color1));
        } else {
            strip->setPixelColor(i, 0);
        }
    }
    
    // Right half for Battery 2
    uint8_t led2Count = (battery2SOC * LEDS_PER_BATTERY) / 100;
    LEDColor color2 = (battery2SOC > 50) ? LED_GREEN : (battery2SOC > 20) ? LED_YELLOW : LED_RED;
    
    for (uint8_t i = LEDS_PER_BATTERY; i < NUM_LEDS; i++) {
        if (i < LEDS_PER_BATTERY + led2Count) {
            strip->setPixelColor(i, colorToPackedRGB(color2));
        } else {
            strip->setPixelColor(i, 0);
        }
    }
}

void LEDController::showWarning(bool active) {
    if (active) {
        setPattern(LED_WARNING_FLASH);
    } else {
        setPattern(LED_BREATHING);
        setColor(LED_GREEN);
    }
}

void LEDController::showError(bool active) {
    if (active) {
        setPattern(LED_ERROR_PULSE);
    } else {
        setPattern(LED_BREATHING);
        setColor(LED_GREEN);
    }
}

void LEDController::showIdlePattern() {
    setPattern(LED_BREATHING);
    setColor(LED_GREEN);
}

void LEDController::showActivePattern() {
    setPattern(LED_RAINBOW);
}

// ============================================================================
// Animation Control
// ============================================================================

void LEDController::pause() {
    paused = true;
}

void LEDController::resume() {
    paused = false;
}

// ============================================================================
// Helper Functions
// ============================================================================

uint32_t LEDController::colorToPackedRGB(LEDColor color) {
    if (strip != nullptr) {
        return strip->Color(color.r, color.g, color.b);
    }
    return 0;
}

uint32_t LEDController::colorToPackedRGB(uint8_t r, uint8_t g, uint8_t b) {
    if (strip != nullptr) {
        return strip->Color(r, g, b);
    }
    return 0;
}

uint8_t LEDController::calculateDelay() {
    // Map speed (0-255) to delay (100ms-10ms)
    return map(speed, 0, 255, 100, 10);
}

uint32_t LEDController::wheel(uint8_t wheelPos) {
    // Color wheel for rainbow effects
    // Input 0-255, output packed RGB
    wheelPos = 255 - wheelPos;
    if (wheelPos < 85) {
        return colorToPackedRGB(255 - wheelPos * 3, 0, wheelPos * 3);
    }
    if (wheelPos < 170) {
        wheelPos -= 85;
        return colorToPackedRGB(0, wheelPos * 3, 255 - wheelPos * 3);
    }
    wheelPos -= 170;
    return colorToPackedRGB(wheelPos * 3, 255 - wheelPos * 3, 0);
}

uint8_t LEDController::sine8(uint8_t x) {
    // Fast 8-bit sine approximation
    // Returns 0-255 for input 0-255
    static const uint8_t sineTable[256] = {
        128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,
        176,179,182,185,188,190,193,196,198,201,203,206,208,211,213,215,
        218,220,222,224,226,228,230,232,234,235,237,238,240,241,243,244,
        245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,255,
        255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246,
        245,244,243,241,240,238,237,235,234,232,230,228,226,224,222,220,
        218,215,213,211,208,206,203,201,198,196,193,190,188,185,182,179,
        176,173,170,167,165,162,158,155,152,149,146,143,140,137,134,131,
        128,124,121,118,115,112,109,106,103,100,97,93,90,88,85,82,
        79,76,73,70,67,65,62,59,57,54,52,49,47,44,42,40,
        37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,
        10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,
        0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,
        10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,
        37,40,42,44,47,49,52,54,57,59,62,65,67,70,73,76,
        79,82,85,88,90,93,97,100,103,106,109,112,115,118,121,124
    };
    return sineTable[x];
}

uint32_t LEDController::colorHSV(uint16_t hue, uint8_t sat, uint8_t val) {
    // Convert HSV to RGB using NeoPixel's gamma correction
    if (strip != nullptr) {
        return strip->ColorHSV(hue, sat, val);
    }
    return 0;
}
