// ============================================================================
// TFT_eSPI User Setup — KRIYA BMS (STM32F407VET6 / BlackPill F411CE)
//
// *** IMPORTANT: Verify all pin assignments against your PCB schematic ***
//
// This file is included by TFT_eSPI when USER_SETUP_LOADED is defined in
// platformio.ini build_flags. It must live in the project include/ directory
// so it is tracked by git and survives PlatformIO library cache clears.
//
// White-screen root cause: Without TFT_RST defined, TFT_eSPI skips the
// hardware reset pulse. On MCU warm-reset the display IC stays in an unknown
// state and ignores the init sequence → white screen. Defining TFT_RST forces
// a clean RST LOW/HIGH cycle before every tft.init(), making init reliable.
// ============================================================================

#define USER_SETUP_INFO "KRIYA_BMS_User_Setup"

// ============================================================================
// Display driver — change to ST7789_DRIVER or ST7735_DRIVER if needed
// ============================================================================
#define ILI9341_DRIVER

// ============================================================================
// SPI pins (hardware SPI1 on STM32)
// PA5 = SPI1_SCK   PA7 = SPI1_MOSI
// ============================================================================
#define TFT_MOSI  PA7
#define TFT_SCLK  PA5

// ============================================================================
// Control pins — VERIFY THESE THREE AGAINST YOUR SCHEMATIC
// ============================================================================
#define TFT_CS    PA4   // Chip-select
#define TFT_DC    PB0   // Data / Command
#define TFT_RST   PB10  // Reset — CRITICAL for reliable init (white-screen fix)

// MISO not needed: display is write-only
// Defining MISO would reconfigure PA6 (used as ONBOARD_LED) as SPI alternate function
// #define TFT_MISO  PA6

// ============================================================================
// SPI clock — 27 MHz is safe for both STM32F407 (84 MHz APB2) and F411 (100 MHz)
// ============================================================================
#define SPI_FREQUENCY  27000000

// ============================================================================
// Fonts
// ============================================================================
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
#define SMOOTH_FONT
