/**
 * @file bms_manager.h
 * @brief JBD BMS communication and data management
 */

#ifndef BMS_MANAGER_H
#define BMS_MANAGER_H

#include <Arduino.h>
#include "config.h"

// JBD BMS Protocol Constants
#define BMS_START_BYTE          0xDD
#define BMS_READ_CMD            0xA5
#define BMS_WRITE_CMD           0x5A
#define BMS_BASIC_INFO_CMD      0x03
#define BMS_CELL_VOLTAGE_CMD    0x04
#define BMS_HARDWARE_VERSION_CMD 0x05
#define BMS_MOS_CONTROL_CMD     0xE1
#define BMS_END_BYTE            0x77

// MOS Control Values
#define MOS_RELEASE_LOCK        0x00
#define MOS_DISABLE_CHARGE      0x01
#define MOS_DISABLE_DISCHARGE   0x02
#define MOS_DISABLE_BOTH        0x03

// BMS Data Structure (15S configuration for Phase 1)
struct BMSData {
    // Basic Info
    float totalVoltage;         // V
    float current;              // A (positive = charging, negative = discharging)
    float remainingCapacity;    // Ah
    float nominalCapacity;      // Ah
    uint16_t cycles;
    uint8_t socPercent;         // State of Charge %
    
    // Cell Voltages (15 cells for 48V pack)
    uint16_t cellVoltages[15];  // mV
    uint8_t numCells;
    
    // Temperatures
    float temp1;                // °C
    float temp2;                // °C
    
    // MOSFET Status
    bool chargeMosfetOn;
    bool dischargeMosfetOn;
    
    // Protection Status
    bool anyProtectionActive;
    struct {
        bool cellOvervoltage;
        bool cellUndervoltage;
        bool packOvervoltage;
        bool packUndervoltage;
        bool chargeOvertemp;
        bool chargeUndertemp;
        bool dischargeOvertemp;
        bool dischargeUndertemp;
        bool chargeOvercurrent;
        bool dischargeOvercurrent;
        bool shortCircuit;
        bool frontEndDetection;
        bool softwareLock;
    } protectionStatus;
    
    // Status flags
    bool dataValid;
    unsigned long lastUpdate;
};

class BMSManager {
public:
    BMSManager(HardwareSerial* serial, uint8_t batteryNumber);
    
    void begin();
    void update();
    
    // Data access
    BMSData getData() const { return data; }
    BMSData& getDataRef() { return data; }  // For direct updates
    bool isDataValid() const { return data.dataValid; }
    bool isTimeout() const;
    
    // Direct data update (for external parsing)
    void updateFromBuffer(uint8_t* buffer, uint8_t length);
    
    // Request data
    bool requestBasicInfo();
    bool requestCellVoltages();
    
    // MOSFET Control Commands
    bool disableChargeMosfet();
    bool disableDischargeMosfet();
    bool enableDischargeMosfet();  // Actually sends release lock
    bool disableBothMosfets();
    bool verifyMosfetState(bool expectedCharge, bool expectedDischarge);
    
    // Utility functions
    float getMinCellVoltage() const;
    float getMaxCellVoltage() const;
    float getCellVoltageDelta() const;
    float getAverageCellVoltage() const;
    bool hasProtectionActive() const;
    
private:
    HardwareSerial* uart;
    uint8_t batteryNum;
    BMSData data;
    
    // Protocol functions - Frame construction
    void buildReadFrame(uint8_t cmd, uint8_t* frame, uint8_t* frameLen);
    void buildWriteFrame(uint8_t cmd, uint8_t* writeData, uint8_t dataLen, uint8_t* frame, uint8_t* frameLen);
    uint16_t calculateChecksum(uint8_t* data, uint8_t length);
    bool validateFrame(uint8_t* frame, uint8_t length);
    
    // Protocol functions - Parsing
    bool parseBasicInfo(uint8_t* data, uint8_t length);
    bool parseCellVoltages(uint8_t* data, uint8_t length);
    
    // Communication
    bool sendFrame(uint8_t* frame, uint8_t frameLen);
    bool receiveFrame(uint8_t* buffer, uint8_t* length, uint16_t timeout);
    bool sendCommand(uint8_t cmd);
    bool sendControlCommand(uint8_t cmd, uint8_t* data, uint8_t dataLen);
    
    // Simplified read (matches working reference code exactly)
    bool readBMSSimple();
    
    // Buffer management
    static const uint16_t RX_BUFFER_SIZE = 256;
    uint8_t rxBuffer[RX_BUFFER_SIZE];
    
    // Timing
    unsigned long lastRequestTime;
    unsigned long lastSuccessfulRead;
};

// Global BMS instances (defined in main.cpp)
extern BMSManager bms1;
extern BMSManager bms2;

#endif // BMS_MANAGER_H

