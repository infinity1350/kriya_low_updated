/**
 * @file display_manager.cpp
 * @brief TFT_eSPI display with elegant minimalistic UI
 */

#include "display_manager.h"
#include "bms_manager.h"
#include "battery_manager.h"
#include "power_manager.h"

// External instances
extern BMSManager bms1;
extern BMSManager bms2;
extern BatteryManager batteryManager;
extern PowerManager powerManager;

// ============================================================================
// Color Palette - Elegant Minimalistic Theme
// ============================================================================
#define BG_COLOR        0x0000      // Pure black background
#define TEXT_PRIMARY    0xFFFF      // White - primary text
#define TEXT_SECONDARY  0x7BEF      // Light gray - secondary text
#define TEXT_MUTED      0x4A69      // Dark gray - muted text
#define ACCENT_CYAN     0x07FF      // Cyan accent
#define ACCENT_GREEN    0x07E0      // Green - good status
#define ACCENT_ORANGE   0xFD20      // Orange - warning
#define ACCENT_RED      0xF800      // Red - error/critical
#define BORDER_LIGHT    0x3186      // Subtle border color
#define BORDER_ACTIVE   0x07FF      // Active border (cyan)

// ============================================================================
// Constructor and Initialization
// ============================================================================

DisplayManager::DisplayManager() : tft() {
    currentMode = DISPLAY_MAIN_SCREEN;
    lastMode = DISPLAY_MAIN_SCREEN;
    lastUpdate = 0;
    needsFullRedraw = true;
    animationFrame = 0;
    lastAnimationUpdate = 0;
    chargingAnimFrame = 0;
    lastChargingAnimUpdate = 0;
    
    // Initialize robot status
    memset(&robotStatus, 0, sizeof(RobotStatus));
    strcpy(robotStatus.mode, "IDLE");
    strcpy(robotStatus.waypoint, "--");
    robotStatus.isConnected = false;
    
    // Initialize animation states
    bat1Anim = {0, 0, 0, 0, true};
    bat2Anim = {0, 0, 0, 0, true};
    
    // Initialize previous values
    lastBat1SOC = -1;
    lastBat2SOC = -1;
    lastBat1Voltage = -1;
    lastBat2Voltage = -1;
    lastBat1Current = -1;
    lastBat2Current = -1;
    memset(lastRobotMode, 0, sizeof(lastRobotMode));
    memset(lastPowerState, 0, sizeof(lastPowerState));
}

void DisplayManager::begin() {
    tft.init();
    tft.setRotation(DISPLAY_ROTATION);
    tft.fillScreen(BG_COLOR);
    
    // Elegant startup splash
    tft.setTextColor(ACCENT_CYAN);
    tft.setTextSize(3);
    printCentered("KRIYA", 80);
    
    tft.setTextColor(TEXT_MUTED);
    tft.setTextSize(1);
    printCentered("Battery Management System", 130);
    
    // Subtle loading indicator
    tft.drawRect(80, 170, 160, 4, BORDER_LIGHT);
    for (int i = 0; i < 156; i += 4) {
        tft.fillRect(82, 172, i, 2, ACCENT_CYAN);
        delay(10);
    }
    
    delay(500);
    clearScreen();
    needsFullRedraw = true;
}

void DisplayManager::clearScreen() {
    tft.fillScreen(BG_COLOR);
    needsFullRedraw = true;
}

// ============================================================================
// Main Update Function
// ============================================================================

void DisplayManager::update() {
    unsigned long now = millis();
    
    // Track power state changes
    static PowerState lastPowerState = POWER_IDLE;
    static bool wasPrecharging = false;
    static bool wasMainScreenDrawn = false;
    PowerState currentPowerState = powerManager.getState();
    
    // Detect power state change
    if (currentPowerState != lastPowerState) {
        needsFullRedraw = true;
        wasMainScreenDrawn = false;
        lastPowerState = currentPowerState;
    }
    
    // Detect pre-charge completion
    bool isPrecharging = powerManager.isPrecharging();
    uint8_t prechargeProgress = powerManager.getPrechargeProgress();
    if (wasPrecharging && isPrecharging && prechargeProgress >= 100) {
        needsFullRedraw = true;
        wasPrecharging = false;
    } else if (isPrecharging && prechargeProgress < 100) {
        wasPrecharging = true;
    } else if (!isPrecharging) {
        wasPrecharging = false;
    }
    
    // Mode change detection
    if (currentMode != lastMode) {
        clearScreen();
        lastMode = currentMode;
        needsFullRedraw = true;
    }
    
    // Update animations
    updateAnimations();
    
    // Animation timing
    if (now - lastAnimationUpdate >= 50) {
        lastAnimationUpdate = now;
        animationFrame++;
        if (animationFrame > 100) animationFrame = 0;
    }
    
    // Draw current screen
    if (powerManager.isEmergency()) {
        drawEmergencyScreen();
    } else if (powerManager.isPrecharging()) {
        uint8_t progress = powerManager.getPrechargeProgress();
        if (progress >= 100) {
            if (!wasMainScreenDrawn) {
                needsFullRedraw = true;
                wasMainScreenDrawn = true;
            }
            drawMainScreen();
        } else {
            wasMainScreenDrawn = false;
            drawPrechargeScreen(progress);
        }
    } else {
        wasMainScreenDrawn = true;
        if (currentMode == DISPLAY_MAIN_SCREEN) {
            drawMainScreen();
        }
    }
    
    lastUpdate = now;
}

// ============================================================================
// Animation System
// ============================================================================

void DisplayManager::updateAnimations() {
    BMSData data1 = bms1.getData();
    BMSData data2 = bms2.getData();
    
    bat1Anim.targetSOC = data1.socPercent;
    bat1Anim.targetVoltage = data1.totalVoltage;
    bat2Anim.targetSOC = data2.socPercent;
    bat2Anim.targetVoltage = data2.totalVoltage;
    
    animateBatteryLevel(bat1Anim, bat1Anim.targetSOC, bat1Anim.targetVoltage);
    animateBatteryLevel(bat2Anim, bat2Anim.targetSOC, bat2Anim.targetVoltage);
}

float DisplayManager::lerp(float current, float target, float speed) {
    float diff = target - current;
    if (abs(diff) < 0.5f) return target;
    return current + (diff * speed);
}

void DisplayManager::animateBatteryLevel(BatteryAnimState& anim, float targetSOC, float targetVoltage) {
    float oldSOC = anim.displayedSOC;
    float oldVoltage = anim.displayedVoltage;
    
    anim.displayedSOC = lerp(anim.displayedSOC, targetSOC, 0.15f);
    anim.displayedVoltage = lerp(anim.displayedVoltage, targetVoltage, 0.15f);
    
    if (abs(anim.displayedSOC - oldSOC) > 0.3f || abs(anim.displayedVoltage - oldVoltage) > 0.05f) {
        anim.needsUpdate = true;
    }
}

// ============================================================================
// Pre-charge Screen - Minimalistic
// ============================================================================

void DisplayManager::drawPrechargeScreen(uint8_t progress) {
    static uint8_t lastProgress = 255;
    static uint16_t lastFillWidth = 0;
    
    const uint16_t barX = 50;
    const uint16_t barY = 120;
    const uint16_t barWidth = 220;
    const uint16_t barHeight = 8;
    const uint16_t maxFillWidth = barWidth - 4;
    
    if (needsFullRedraw) {
        tft.fillScreen(BG_COLOR);
        
        // Simple text - no backgrounds
        tft.setTextColor(TEXT_PRIMARY);
        tft.setTextSize(2);
        printCentered("INITIALIZING", 60);
        
        // Thin progress bar outline
        tft.drawRect(barX, barY, barWidth, barHeight, BORDER_LIGHT);
        
        // Status text
        tft.setTextColor(TEXT_MUTED);
        tft.setTextSize(1);
        printCentered("Pre-charging capacitors", 180);
        
        lastProgress = 255;
        lastFillWidth = 0;
        needsFullRedraw = false;
    }
    
    if (progress != lastProgress) {
        uint16_t newFillWidth = (maxFillWidth * progress) / 100;
        
        if (newFillWidth > lastFillWidth) {
            tft.fillRect(barX + 2 + lastFillWidth, barY + 2, 
                        newFillWidth - lastFillWidth, barHeight - 4, ACCENT_CYAN);
        }
        
        // Clear and redraw percentage
        tft.fillRect(130, 145, 60, 25, BG_COLOR);
        tft.setTextColor(TEXT_PRIMARY);
        tft.setTextSize(2);
        char buf[8];
        sprintf(buf, "%d%%", progress);
        printCentered(buf, 150);
        
        lastProgress = progress;
        lastFillWidth = newFillWidth;
    }
}

// ============================================================================
// Emergency Screen - Clean Blinking
// ============================================================================

void DisplayManager::drawEmergencyScreen() {
    static unsigned long lastBlink = 0;
    static bool showRed = true;
    
    unsigned long now = millis();
    if (now - lastBlink > 500) {
        lastBlink = now;
        showRed = !showRed;
        
        if (showRed) {
            tft.fillScreen(ACCENT_RED);
            tft.setTextColor(TEXT_PRIMARY);
        } else {
            tft.fillScreen(BG_COLOR);
            tft.setTextColor(ACCENT_RED);
        }
        
        tft.setTextSize(3);
        printCentered("EMERGENCY", 90);
        printCentered("STOP", 130);
        
        tft.setTextSize(1);
        tft.setTextColor(showRed ? TEXT_PRIMARY : TEXT_MUTED);
        printCentered("System Disabled", 190);
    }
}

// ============================================================================
// Main Screen - Elegant Minimalistic Design
// ============================================================================

void DisplayManager::drawMainScreen() {
    BMSData data1 = bms1.getData();
    BMSData data2 = bms2.getData();
    uint8_t activeBat = batteryManager.getActiveBattery();
    
    if (needsFullRedraw) {
        tft.fillScreen(BG_COLOR);
        
        // Top status bar - simple line separator
        tft.drawFastHLine(0, 24, DISPLAY_WIDTH, BORDER_LIGHT);
        
        // System title
        tft.setTextColor(TEXT_MUTED);
        tft.setTextSize(1);
        tft.setCursor(8, 8);
        tft.print("KRIYA BMS");
        
        // Power state (right aligned)
        const char* pwrState = powerManager.getStateString();
        tft.setTextColor(powerManager.isReady() ? ACCENT_GREEN : ACCENT_ORANGE);
        tft.setCursor(DISPLAY_WIDTH - 50, 8);
        tft.print(pwrState);
    }
    
    // Battery section
    bool bat1Changed = needsFullRedraw || bat1Anim.needsUpdate;
    bool bat2Changed = needsFullRedraw || bat2Anim.needsUpdate;
    
    if (bat1Changed) {
        drawBatteryCard(10, 35, 1, bat1Anim.displayedSOC, data1, activeBat == 1);
        bat1Anim.needsUpdate = false;
    }
    
    if (bat2Changed) {
        drawBatteryCard(165, 35, 2, bat2Anim.displayedSOC, data2, activeBat == 2);
        bat2Anim.needsUpdate = false;
    }
    
    needsFullRedraw = false;
}

// ============================================================================
// Battery Card - Elegant Design
// ============================================================================

void DisplayManager::drawBatteryCard(uint16_t x, uint16_t y, uint8_t batNum, 
                                     float soc, BMSData& data, bool isActive) {
    const uint16_t w = 145;
    const uint16_t h = 195;
    
    // Clear card area
    tft.fillRect(x, y, w, h, BG_COLOR);
    
    // Card border - subtle or highlighted if active
    uint16_t borderColor = isActive ? ACCENT_CYAN : BORDER_LIGHT;
    tft.drawRoundRect(x, y, w, h, 6, borderColor);
    
    // Battery number header
    tft.setTextColor(isActive ? ACCENT_CYAN : TEXT_MUTED);
    tft.setTextSize(1);
    char header[16];
    sprintf(header, "BATTERY %d", batNum);
    tft.setCursor(x + 35, y + 10);
    tft.print(header);
    
    // Active indicator
    if (isActive) {
        tft.fillCircle(x + w - 15, y + 13, 4, ACCENT_GREEN);
    }
    
    // Battery gauge container
    const uint16_t gaugeX = x + 22;
    const uint16_t gaugeY = y + 30;
    const uint16_t gaugeW = 100;
    const uint16_t gaugeH = 80;
    
    // Gauge outline
    tft.drawRoundRect(gaugeX, gaugeY, gaugeW, gaugeH, 4, borderColor);
    
    // Battery terminal (top nub)
    tft.fillRoundRect(gaugeX + 35, gaugeY - 5, 30, 6, 2, borderColor);
    
    // Fill based on SOC
    uint16_t fillColor;
    if (soc > 50) fillColor = ACCENT_GREEN;
    else if (soc > 20) fillColor = ACCENT_ORANGE;
    else fillColor = ACCENT_RED;
    
    // Calculate fill height
    uint16_t fillH = ((gaugeH - 8) * (uint16_t)soc) / 100;
    uint16_t fillY = gaugeY + gaugeH - 4 - fillH;
    
    // Clear inside gauge
    tft.fillRect(gaugeX + 4, gaugeY + 4, gaugeW - 8, gaugeH - 8, BG_COLOR);
    
    // Draw fill
    if (fillH > 0) {
        tft.fillRoundRect(gaugeX + 4, fillY, gaugeW - 8, fillH, 2, fillColor);
    }
    
    // SOC percentage - large, centered over gauge
    tft.setTextColor(TEXT_PRIMARY);
    tft.setTextSize(3);
    char socStr[8];
    sprintf(socStr, "%d%%", (int)soc);
    uint16_t textW = strlen(socStr) * 18;
    tft.setCursor(gaugeX + (gaugeW - textW) / 2, gaugeY + gaugeH / 2 - 10);
    tft.print(socStr);
    
    // Divider line
    tft.drawFastHLine(x + 10, y + 120, w - 20, BORDER_LIGHT);
    
    // Voltage and Current - clean layout with transparent backgrounds
    tft.setTextSize(1);
    
    // Voltage label and value
    tft.setTextColor(TEXT_MUTED);
    tft.setCursor(x + 15, y + 132);
    tft.print("VOLTAGE");
    
    tft.setTextColor(TEXT_PRIMARY);
    tft.setTextSize(2);
    char voltStr[12];
    if (data.dataValid) {
        sprintf(voltStr, "%.1fV", data.totalVoltage);
    } else {
        sprintf(voltStr, "--.-V");
    }
    tft.setCursor(x + 15, y + 145);
    tft.print(voltStr);
    
    // Current label and value
    tft.setTextSize(1);
    tft.setTextColor(TEXT_MUTED);
    tft.setCursor(x + 85, y + 132);
    tft.print("CURRENT");
    
    // Current with color coding
    uint16_t currColor = TEXT_PRIMARY;
    if (data.dataValid) {
        if (data.current > 0.5f) currColor = ACCENT_GREEN;      // Charging
        else if (data.current < -0.5f) currColor = ACCENT_ORANGE; // Discharging
    }
    
    tft.setTextColor(currColor);
    tft.setTextSize(2);
    char currStr[12];
    if (data.dataValid) {
        sprintf(currStr, "%.1fA", data.current);
    } else {
        sprintf(currStr, "--.-A");
    }
    tft.setCursor(x + 85, y + 145);
    tft.print(currStr);
    
    // Connection status at bottom
    tft.setTextSize(1);
    if (!data.dataValid) {
        tft.setTextColor(ACCENT_RED);
        tft.setCursor(x + 30, y + 175);
        tft.print("DISCONNECTED");
    } else {
        tft.setTextColor(ACCENT_GREEN);
        tft.setCursor(x + 40, y + 175);
        tft.print("CONNECTED");
    }
}

// ============================================================================
// Legacy functions (kept for compatibility)
// ============================================================================

void DisplayManager::drawMinimalBattery(uint16_t x, uint16_t y, uint8_t batNum, 
                                        float soc, BMSData& data, bool isActive) {
    drawBatteryCard(x, y, batNum, soc, data, isActive);
}

void DisplayManager::drawHeader(const char* statusText) {
    // Minimal header
    tft.fillRect(0, 0, DISPLAY_WIDTH, 24, BG_COLOR);
    tft.drawFastHLine(0, 24, DISPLAY_WIDTH, BORDER_LIGHT);
    
    tft.setTextColor(TEXT_MUTED);
    tft.setTextSize(1);
    tft.setCursor(8, 8);
    tft.print(statusText);
}

void DisplayManager::drawBatteryGauge(uint16_t x, uint16_t y, uint8_t batteryNum,
                                      float soc, bool isActive, bool isCharging) {
    BMSData emptyData = {0};
    drawBatteryCard(x, y, batteryNum, soc, emptyData, isActive);
}

void DisplayManager::drawFooter() {
    // Footer no longer used in minimalistic design
}

// ============================================================================
// Alert Screens
// ============================================================================

void DisplayManager::drawAlertScreen(const char* message, uint16_t bgColor) {
    tft.fillScreen(bgColor);
    
    uint16_t textColor = TEXT_PRIMARY;
    if (bgColor == TFT_WHITE || bgColor == TFT_YELLOW) {
        textColor = BG_COLOR;
    }
    
    tft.setTextColor(textColor);
    tft.setTextSize(2);
    printCentered(message, DISPLAY_HEIGHT / 2 - 10);
    
    lastMode = DISPLAY_ERROR_SCREEN;
}

// ============================================================================
// Robot Status Updates (from ROS)
// ============================================================================

void DisplayManager::updateRobotStatus(const char* mode, const char* waypoint, const char* error) {
    strncpy(robotStatus.mode, mode, sizeof(robotStatus.mode) - 1);
    strncpy(robotStatus.waypoint, waypoint, sizeof(robotStatus.waypoint) - 1);
    
    if (error != nullptr && strlen(error) > 0) {
        strncpy(robotStatus.errorMsg, error, sizeof(robotStatus.errorMsg) - 1);
        robotStatus.hasError = true;
    } else {
        robotStatus.errorMsg[0] = '\0';
        robotStatus.hasError = false;
    }
    
    robotStatus.lastUpdate = millis();
}

void DisplayManager::setRobotConnected(bool connected) {
    robotStatus.isConnected = connected;
}

// ============================================================================
// Mode Control
// ============================================================================

void DisplayManager::setMode(DisplayMode mode) {
    if (currentMode != mode) {
        currentMode = mode;
        needsFullRedraw = true;
    }
}

// ============================================================================
// Utility Functions
// ============================================================================

void DisplayManager::printCentered(const char* text, uint16_t y, uint16_t color) {
    int16_t textWidth = strlen(text) * 6 * tft.textsize;
    uint16_t x = (DISPLAY_WIDTH - textWidth) / 2;
    tft.setCursor(x, y);
    if (color != 0) {
        tft.setTextColor(color);
    }
    tft.print(text);
}

uint16_t DisplayManager::getColorForSOC(float soc) {
    if (soc > 50) return ACCENT_GREEN;
    else if (soc > 20) return ACCENT_ORANGE;
    else return ACCENT_RED;
}

uint16_t DisplayManager::getColorForTemperature(float temp) {
    if (temp > 50) return ACCENT_RED;
    else if (temp > 40) return ACCENT_ORANGE;
    else if (temp < 5) return ACCENT_CYAN;
    else return ACCENT_GREEN;
}
