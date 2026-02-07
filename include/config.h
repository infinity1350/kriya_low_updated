/**
 * @file config.h
 * @brief Hardware pin definitions and constants for STM32F407VET6
 * 
 * Updated for Phase 2 with exact hardware pinouts
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================
// IMPORTANT: Set to 0 for ROS mode (rosserial over USB CDC)
//            Set to 1 for debug mode (Serial monitor output)
//
// When DEBUG_MODE = 0:
//   - ROS serial communication is enabled over USB CDC
//   - Serial is used by rosserial (do NOT open Serial Monitor)
//   - Run: rosserial_python serial_node.py /dev/ttyACM0 (or your port)
//
// When DEBUG_MODE = 1:
//   - ROS is DISABLED, debug prints are enabled
//   - Serial is used for debug output (open Serial Monitor at 115200 baud)
#define DEBUG_MODE          0

#if DEBUG_MODE
    #define DEBUG_PRINT(x)      Serial.print(x)
    #define DEBUG_PRINTLN(x)    Serial.println(x)
    #define DEBUG_FLUSH()       Serial.flush()
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_FLUSH()
#endif

// ============================================================================
// UART CONFIGURATION (BMS Communication)
// ============================================================================
// USART1 - Battery #1 (JBD BMS)
#define UART1_TX_PIN        PA9
#define UART1_RX_PIN        PA10
#define UART1_BAUD          9600

// USART2 - Battery #2 (JBD BMS)
#define UART2_TX_PIN        PA2
#define UART2_RX_PIN        PA3
#define UART2_BAUD          9600

// ============================================================================
// SPI CONFIGURATION (TFT_eSPI handles this via User_Setup.h)
// ============================================================================
// Display uses hardware SPI - configured in TFT_eSPI User_Setup.h

// ============================================================================
// WS2812B LED STRIP
// ============================================================================
#define LED_STRIP_PIN       PA1
#define NUM_LEDS            84
#define LED_BRIGHTNESS      255
// Note: Using NEO_KHZ400 for this strip

// ============================================================================
// EMERGENCY STOP
// ============================================================================
#define E_STOP_PIN          PB12      // Emergency stop input (active LOW)

// ============================================================================
// BUTTON PINS (Active LOW with internal pull-up)
// ============================================================================
// Button 1 - Brake engage/disengage
#define BUTTON1_IN_PIN      PB13      // Input
#define BUTTON1_OUT_PIN     PB6       // LED output

// Button 2 - Action (next waypoint)
#define BUTTON2_IN_PIN      PB15      // Input
#define BUTTON2_OUT_PIN     PB7       // LED output

// Button 3 - Trolley hitch status
#define BUTTON3_IN_PIN      PB14      // Input
#define BUTTON3_OUT_PIN     PB8       // LED output

// ============================================================================
// AUDIO OUTPUTS
// ============================================================================
#define SIREN_PIN           PB1       // Siren output
#define BUZZER_PIN          PB2       // Buzzer output

// ============================================================================
// POWER CONTROL PINS
// ============================================================================
#define PRECHARGE_PIN       PB9       // Pre-charge circuit control
#define CONTACTOR_PIN       PA13      // Main contactor control

// ============================================================================
// STATUS LED
// ============================================================================
#define ONBOARD_LED_PIN     PA6       // Onboard status LED

// ============================================================================
// PRE-CHARGE CIRCUIT TIMING (Configurable)
// ============================================================================
#define PRECHARGE_DURATION_MS       2500    // Pre-charge time before contactor (2.5s)
#define PRECHARGE_TO_CONTACTOR_MS   300     // Delay after pre-charge complete before contactor ON
#define CONTACTOR_SETTLE_MS         500     // Wait after contactor ON before pre-charge OFF
#define EMERGENCY_DEBOUNCE_MS       10      // Fast debounce for emergency input

// ============================================================================
// SIREN TIMING
// ============================================================================
#define SIREN_ON_DURATION_MS        5000    // Siren ON time during emergency (5s)
#define SIREN_CYCLE_DURATION_MS     30000   // Full siren cycle (30s)

// ============================================================================
// BUTTON DEBOUNCE
// ============================================================================
#define DEBOUNCE_DELAY_MS           50      // Button debounce time

// ============================================================================
// TIMING CONSTANTS
// ============================================================================
#define HEARTBEAT_INTERVAL_MS       1000    // Heartbeat LED toggle
#define BMS_UPDATE_INTERVAL_MS      2000    // BMS read interval
#define DISPLAY_UPDATE_INTERVAL_MS  500     // Display refresh
#define SAFETY_CHECK_INTERVAL_MS    10      // Safety monitoring (100Hz)
#define LED_UPDATE_INTERVAL_MS      33      // LED patterns (~30Hz)
#define ROS_SPIN_INTERVAL_MS        10      // ROS communication (100Hz)

// ============================================================================
// BMS CONFIGURATION
// ============================================================================
#define BMS_TIMEOUT_MS              5000    // BMS communication timeout
#define BMS_READ_TIMEOUT_MS         200     // Single read timeout
#define BMS_MAX_CELLS               16
#define BMS_PACKET_SIZE             256

// ============================================================================
// SAFETY THRESHOLDS
// ============================================================================
#define BATTERY_LOW_VOLTAGE         44.0f   // V (for 13S pack)
#define BATTERY_HIGH_VOLTAGE        54.6f   // V
#define BATTERY_LOW_PERCENTAGE      20.0f   // %
#define BATTERY_HIGH_TEMP           60.0f   // °C
#define BATTERY_LOW_TEMP            -10.0f  // °C

// ============================================================================
// BATTERY SWITCHING PARAMETERS
// ============================================================================
#define SWITCH_STABILIZATION_MS     100     // Wait after enabling new battery
#define SWITCH_ISOLATION_MS         100     // Wait after disabling old battery
#define VOLTAGE_MISMATCH_WARN_V     3.0f    // Log warning if diff exceeds this

// ============================================================================
// DISPLAY CONFIGURATION
// ============================================================================
#define DISPLAY_WIDTH               320
#define DISPLAY_HEIGHT              240
#define DISPLAY_ROTATION            1       // 0-3 for different orientations

// Color definitions (RGB565) - for TFT_eSPI compatibility
#define COLOR_BLACK                 0x0000
#define COLOR_WHITE                 0xFFFF
#define COLOR_RED                   0xF800
#define COLOR_GREEN                 0x07E0
#define COLOR_BLUE                  0x001F
#define COLOR_YELLOW                0xFFE0
#define COLOR_ORANGE                0xFD20
#define COLOR_CYAN                  0x07FF
#define COLOR_MAGENTA               0xF81F
#define COLOR_GRAY                  0x8410
#define COLOR_DARKGRAY              0x4208

// ============================================================================
// LEGACY COMPATIBILITY (map old names to new)
// ============================================================================
#define LOW_BUZZ_PIN                BUZZER_PIN
#define HIGH_BUZZ_PIN               SIREN_PIN
#define BUTTON1_SWITCH_PIN          BUTTON1_IN_PIN
#define BUTTON2_SWITCH_PIN          BUTTON2_IN_PIN
#define BUTTON3_SWITCH_PIN          BUTTON3_IN_PIN
#define BUTTON1_LED_PIN             BUTTON1_OUT_PIN
#define BUTTON2_LED_PIN             BUTTON2_OUT_PIN
#define BUTTON3_LED_PIN             BUTTON3_OUT_PIN
#define HEARTBEAT_LED_PIN           ONBOARD_LED_PIN

// Legacy timing names
#define DEBOUNCE_DELAY              DEBOUNCE_DELAY_MS
#define BEEP_SHORT_DURATION         100
#define BEEP_LONG_DURATION          500
#define BEEP_INTERVAL               200

#endif // CONFIG_H
