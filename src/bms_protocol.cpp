/**
 * @file bms_protocol.cpp
 * @brief JBD BMS Protocol Implementation
 * 
 * Implements the JBD BMS UART protocol with proper frame construction,
 * checksum calculation, and data parsing for 15S battery packs.
 */

#include "bms_manager.h"

// ============================================================================
// Constructor and Initialization
// ============================================================================

BMSManager::BMSManager(HardwareSerial* serial, uint8_t batteryNumber) {
    uart = serial;
    batteryNum = batteryNumber;
    lastRequestTime = 0;
    lastSuccessfulRead = 0;
    
    // Initialize data structure
    memset(&data, 0, sizeof(BMSData));
    data.dataValid = false;
    data.numCells = 15;  // 15S configuration for Phase 1
}

void BMSManager::begin() {
    uart->begin(9600);
    delay(100);  // Allow serial to stabilize
}

// ============================================================================
// Protocol Layer - Frame Construction
// ============================================================================

void BMSManager::buildReadFrame(uint8_t cmd, uint8_t* frame, uint8_t* frameLen) {
    // Frame structure: [START] [READ] [CMD] [LEN] [CHECKSUM_H] [CHECKSUM_L] [STOP]
    // For read commands, LEN = 0x00
    
    uint8_t checksumData[2] = {cmd, 0x00};
    uint16_t checksum = calculateChecksum(checksumData, 2);
    
    frame[0] = BMS_START_BYTE;     // 0xDD
    frame[1] = BMS_READ_CMD;       // 0xA5
    frame[2] = cmd;                // Command (0x03, 0x04, etc.)
    frame[3] = 0x00;               // Length = 0 for read
    frame[4] = (checksum >> 8) & 0xFF;  // Checksum high byte
    frame[5] = checksum & 0xFF;         // Checksum low byte
    frame[6] = BMS_END_BYTE;       // 0x77
    
    *frameLen = 7;
}

void BMSManager::buildWriteFrame(uint8_t cmd, uint8_t* writeData, uint8_t dataLen, 
                                 uint8_t* frame, uint8_t* frameLen) {
    // Frame structure: [START] [WRITE] [CMD] [LEN] [DATA...] [CHECKSUM_H] [CHECKSUM_L] [STOP]
    
    // Build checksum data: CMD + LEN + DATA
    uint8_t checksumBuffer[32];  // Reasonable max
    checksumBuffer[0] = cmd;
    checksumBuffer[1] = dataLen;
    for (uint8_t i = 0; i < dataLen; i++) {
        checksumBuffer[2 + i] = writeData[i];
    }
    
    uint16_t checksum = calculateChecksum(checksumBuffer, 2 + dataLen);
    
    // Build frame
    frame[0] = BMS_START_BYTE;     // 0xDD
    frame[1] = BMS_WRITE_CMD;      // 0x5A
    frame[2] = cmd;                // Command
    frame[3] = dataLen;            // Data length
    
    // Copy data
    for (uint8_t i = 0; i < dataLen; i++) {
        frame[4 + i] = writeData[i];
    }
    
    frame[4 + dataLen] = (checksum >> 8) & 0xFF;  // Checksum high
    frame[5 + dataLen] = checksum & 0xFF;         // Checksum low
    frame[6 + dataLen] = BMS_END_BYTE;            // 0x77
    
    *frameLen = 7 + dataLen;
}

uint16_t BMSManager::calculateChecksum(uint8_t* data, uint8_t length) {
    // JBD BMS checksum: (~sum + 1) & 0xFFFF
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (~sum + 1) & 0xFFFF;
}

bool BMSManager::validateFrame(uint8_t* frame, uint8_t length) {
    // Minimum frame size check
    if (length < 7) {
        return false;
    }
    
    // Check start and stop bytes
    if (frame[0] != BMS_START_BYTE || frame[length - 1] != BMS_END_BYTE) {
        return false;
    }
    
    // Extract checksum from frame
    uint8_t receivedChecksumH = frame[length - 3];
    uint8_t receivedChecksumL = frame[length - 2];
    uint16_t receivedChecksum = (receivedChecksumH << 8) | receivedChecksumL;
    
    // Calculate expected checksum (CMD + LEN + DATA)
    uint8_t dataLength = frame[3];
    uint8_t checksumDataLen = 2 + dataLength;  // CMD + LEN + DATA bytes
    uint16_t expectedChecksum = calculateChecksum(&frame[2], checksumDataLen);
    
    return (receivedChecksum == expectedChecksum);
}

// ============================================================================
// Communication Layer
// ============================================================================

bool BMSManager::sendFrame(uint8_t* frame, uint8_t frameLen) {
    // Don't flush - just write (flush can cause issues on some STM32)
    uart->write(frame, frameLen);
    return true;
}

bool BMSManager::receiveFrame(uint8_t* buffer, uint8_t* length, uint16_t timeout) {
    // Simple approach matching working reference code
    unsigned long startTime = millis();
    uint8_t idx = 0;
    
    // Continuously read bytes with smart timeout
    while (millis() - startTime < timeout && idx < 64) {
        if (uart->available()) {
            buffer[idx++] = uart->read();
            // Extend timeout slightly when data is flowing
            if (millis() - startTime > (timeout - 50)) {
                startTime = millis() - (timeout - 50);
            }
        }
    }
    
    *length = idx;
    
    // Need minimum 7 bytes for valid frame (START + CMD + STATUS + LEN + CHK_H + CHK_L + STOP)
    // For basic info response, need at least 30 bytes (7 + 23 data)
    if (idx < 7) {
        return false;
    }
    
    // Find start byte in received data (in case of garbage before frame)
    uint8_t frameStart = 0;
    for (uint8_t i = 0; i < idx; i++) {
        if (buffer[i] == BMS_START_BYTE) {
            frameStart = i;
            break;
        }
    }
    
    // Shift buffer if start byte wasn't at position 0
    if (frameStart > 0 && frameStart < idx) {
        uint8_t newLen = idx - frameStart;
        memmove(buffer, buffer + frameStart, newLen);
        idx = newLen;
        *length = idx;
    }
    
    // Validate frame structure
    if (idx >= 7 && buffer[0] == BMS_START_BYTE && buffer[idx - 1] == BMS_END_BYTE) {
        return true;  // Basic validation passed
    }
    
    return false;
}

bool BMSManager::sendCommand(uint8_t cmd) {
    uint8_t frame[16];
    uint8_t frameLen;
    
    buildReadFrame(cmd, frame, &frameLen);
    
    // Clear any pending data
    while (uart->available()) {
        uart->read();
    }
    
    sendFrame(frame, frameLen);
    
    // Receive response
    uint8_t responseLen;
    if (!receiveFrame(rxBuffer, &responseLen, 500)) {
        return false;
    }
    
    // Extract data portion (skip START, STATE, CMD, LEN and CHKSUM, STOP)
    uint8_t dataLen = rxBuffer[3];
    uint8_t* dataStart = &rxBuffer[4];
    
    // Parse based on command
    if (cmd == BMS_BASIC_INFO_CMD) {
        return parseBasicInfo(dataStart, dataLen);
    } else if (cmd == BMS_CELL_VOLTAGE_CMD) {
        return parseCellVoltages(dataStart, dataLen);
    }
    
    return true;
}

bool BMSManager::sendControlCommand(uint8_t cmd, uint8_t* controlData, uint8_t dataLen) {
    uint8_t frame[16];
    uint8_t frameLen;
    
    buildWriteFrame(cmd, controlData, dataLen, frame, &frameLen);
    
    // Clear any pending data
    while (uart->available()) {
        uart->read();
    }
    
    sendFrame(frame, frameLen);
    
    // Some BMS units send response, some don't. Wait briefly.
    delay(50);
    
    // Clear any response
    while (uart->available()) {
        uart->read();
    }
    
    return true;
}

// ============================================================================
// Data Parsing
// ============================================================================

bool BMSManager::parseBasicInfo(uint8_t* rawData, uint8_t length) {
    if (length < 23) {
        return false;
    }
    
    // Parse total voltage (10mV units)
    data.totalVoltage = ((rawData[0] << 8) | rawData[1]) * 0.01f;
    
    // Parse current (10mA units, signed)
    int16_t currentRaw = (int16_t)((rawData[2] << 8) | rawData[3]);
    data.current = currentRaw * 0.01f;
    
    // Parse remaining capacity (10mAh units)
    data.remainingCapacity = ((rawData[4] << 8) | rawData[5]) * 0.01f;
    
    // Parse nominal capacity (10mAh units)
    data.nominalCapacity = ((rawData[6] << 8) | rawData[7]) * 0.01f;
    
    // Parse cycle count
    data.cycles = (rawData[8] << 8) | rawData[9];
    
    // Skip production date (rawData[10-11])
    
    // Skip balance status (rawData[12-15], 32-bit)
    
    // Parse protection status (16-bit)
    uint16_t protStatus = (rawData[16] << 8) | rawData[17];
    data.protectionStatus.cellOvervoltage = protStatus & (1 << 0);
    data.protectionStatus.cellUndervoltage = protStatus & (1 << 1);
    data.protectionStatus.packOvervoltage = protStatus & (1 << 2);
    data.protectionStatus.packUndervoltage = protStatus & (1 << 3);
    data.protectionStatus.chargeOvertemp = protStatus & (1 << 4);
    data.protectionStatus.chargeUndertemp = protStatus & (1 << 5);
    data.protectionStatus.dischargeOvertemp = protStatus & (1 << 6);
    data.protectionStatus.dischargeUndertemp = protStatus & (1 << 7);
    data.protectionStatus.chargeOvercurrent = protStatus & (1 << 8);
    data.protectionStatus.dischargeOvercurrent = protStatus & (1 << 9);
    data.protectionStatus.shortCircuit = protStatus & (1 << 10);
    data.protectionStatus.frontEndDetection = protStatus & (1 << 11);
    data.protectionStatus.softwareLock = protStatus & (1 << 12);
    
    data.anyProtectionActive = (protStatus != 0);
    
    // Skip software version (rawData[18])
    
    // Parse SOC (State of Charge %)
    data.socPercent = rawData[19];
    
    // Parse FET status
    uint8_t fetStatus = rawData[20];
    data.chargeMosfetOn = fetStatus & 0x01;      // Bit 0
    data.dischargeMosfetOn = fetStatus & 0x02;   // Bit 1
    
    // Parse number of cells and temps
    data.numCells = rawData[21];
    uint8_t numTemps = rawData[22];
    
    // Parse temperatures (if present)
    // Temperature in 0.1K units, offset by 2731 (273.1K = 0°C)
    if (length >= 25 && numTemps >= 1) {
        uint16_t temp1Raw = (rawData[23] << 8) | rawData[24];
        data.temp1 = (temp1Raw - 2731) / 10.0f;
    }
    
    if (length >= 27 && numTemps >= 2) {
        uint16_t temp2Raw = (rawData[25] << 8) | rawData[26];
        data.temp2 = (temp2Raw - 2731) / 10.0f;
    }
    
    // Mark data as valid
    data.dataValid = true;
    data.lastUpdate = millis();
    lastSuccessfulRead = millis();
    
    return true;
}

bool BMSManager::parseCellVoltages(uint8_t* rawData, uint8_t length) {
    // Each cell voltage is 2 bytes (mV)
    uint8_t cellsInResponse = length / 2;
    
    if (cellsInResponse > 15) {
        cellsInResponse = 15;  // Limit to our array size
    }
    
    for (uint8_t i = 0; i < cellsInResponse; i++) {
        data.cellVoltages[i] = (rawData[i * 2] << 8) | rawData[i * 2 + 1];
    }
    
    return true;
}

// ============================================================================
// Public Interface
// ============================================================================

void BMSManager::update() {
    // Use simplified read that matches working reference code
    readBMSSimple();
}

bool BMSManager::requestBasicInfo() {
    return sendCommand(BMS_BASIC_INFO_CMD);
}

bool BMSManager::requestCellVoltages() {
    return sendCommand(BMS_CELL_VOLTAGE_CMD);
}

// ============================================================================
// Simplified BMS Read - Matches Working Reference Code Exactly
// ============================================================================

bool BMSManager::readBMSSimple() {
    // Skip if uart not set
    if (uart == nullptr) {
        data.dataValid = false;
        return false;
    }
    
    // Build frame exactly like working code
    uint8_t frame[7];
    uint8_t chkH, chkL;
    uint8_t checksumData[2] = {0x03, 0x00};  // cmd=0x03, len=0x00
    
    // Calculate checksum: 0xFFFF - sum + 1
    uint16_t sum = checksumData[0] + checksumData[1];
    uint16_t checksum = 0xFFFF - sum + 1;
    chkH = (checksum >> 8) & 0xFF;
    chkL = checksum & 0xFF;
    
    frame[0] = 0xDD;  // Start
    frame[1] = 0xA5;  // Read command
    frame[2] = 0x03;  // Basic info command
    frame[3] = 0x00;  // Length = 0
    frame[4] = chkH;
    frame[5] = chkL;
    frame[6] = 0x77;  // End
    
    // Flush input buffer - use simple loop with counter to prevent infinite loop
    int flushCount = 0;
    while (uart->available() && flushCount < 256) {
        uart->read();
        flushCount++;
    }
    
    // Send frame
    uart->write(frame, 7);
    
    // Read response with timeout
    uint8_t idx = 0;
    uint32_t startTime = millis();
    const uint32_t TIMEOUT = 200;
    
    while (millis() - startTime < TIMEOUT && idx < 64) {
        if (uart->available()) {
            rxBuffer[idx++] = uart->read();
            // Extend timeout slightly if data is flowing
            if (millis() - startTime > (TIMEOUT - 20)) {
                startTime = millis() - (TIMEOUT - 20);
            }
        }
    }
    
    // Check minimum response length
    if (idx < 23) {
        data.dataValid = false;
        return false;
    }
    
    // Parse directly from buffer (exactly like working code)
    // rxBuffer[0] = 0xDD (start)
    // rxBuffer[1] = 0x03 (cmd echo)
    // rxBuffer[2] = 0x00 (status)
    // rxBuffer[3] = LEN
    // rxBuffer[4+] = data
    
    data.totalVoltage = ((rxBuffer[4] << 8) | rxBuffer[5]) * 0.01f;
    data.current = ((int16_t)((rxBuffer[6] << 8) | rxBuffer[7])) * 0.01f;
    data.remainingCapacity = ((rxBuffer[8] << 8) | rxBuffer[9]) * 0.01f;
    data.nominalCapacity = ((rxBuffer[10] << 8) | rxBuffer[11]) * 0.01f;
    data.cycles = (rxBuffer[12] << 8) | rxBuffer[13];
    
    // Protection status at rxBuffer[20:21] (offset 16-17 in data)
    if (idx >= 22) {
        uint16_t protStatus = (rxBuffer[20] << 8) | rxBuffer[21];
        data.anyProtectionActive = (protStatus != 0);
    }
    
    // SOC at rxBuffer[23] (offset 19 in data)
    if (idx >= 24) {
        data.socPercent = rxBuffer[23];
    }
    
    // FET status at rxBuffer[24] (offset 20 in data)
    if (idx >= 25) {
        uint8_t fetStatus = rxBuffer[24];
        data.chargeMosfetOn = fetStatus & 0x01;
        data.dischargeMosfetOn = fetStatus & 0x02;
    }
    
    // Number of cells and temps
    if (idx >= 27) {
        data.numCells = rxBuffer[25];
        uint8_t numTemps = rxBuffer[26];
        
        // Temperature 1
        if (idx >= 29 && numTemps >= 1) {
            uint16_t temp1Raw = (rxBuffer[27] << 8) | rxBuffer[28];
            data.temp1 = (temp1Raw - 2731) / 10.0f;
        }
        
        // Temperature 2
        if (idx >= 31 && numTemps >= 2) {
            uint16_t temp2Raw = (rxBuffer[29] << 8) | rxBuffer[30];
            data.temp2 = (temp2Raw - 2731) / 10.0f;
        }
    }
    
    data.dataValid = true;
    data.lastUpdate = millis();
    lastSuccessfulRead = millis();
    
    return true;
}

bool BMSManager::isTimeout() const {
    return (millis() - lastSuccessfulRead) > BMS_TIMEOUT_MS;
}

// ============================================================================
// MOSFET Control Commands
// ============================================================================

bool BMSManager::disableChargeMosfet() {
    uint8_t controlData[2] = {0x00, MOS_DISABLE_CHARGE};
    bool result = sendControlCommand(BMS_MOS_CONTROL_CMD, controlData, 2);
    delay(100);  // Wait for MOSFET to switch
    return result;
}

bool BMSManager::disableDischargeMosfet() {
    uint8_t controlData[2] = {0x00, MOS_DISABLE_DISCHARGE};
    bool result = sendControlCommand(BMS_MOS_CONTROL_CMD, controlData, 2);
    delay(100);  // Wait for MOSFET to switch
    return result;
}

bool BMSManager::enableDischargeMosfet() {
    // Enable discharge by releasing lock
    uint8_t controlData[2] = {0x00, MOS_RELEASE_LOCK};
    bool result = sendControlCommand(BMS_MOS_CONTROL_CMD, controlData, 2);
    delay(100);  // Wait for MOSFET to switch
    return result;
}

bool BMSManager::disableBothMosfets() {
    uint8_t controlData[2] = {0x00, MOS_DISABLE_BOTH};
    bool result = sendControlCommand(BMS_MOS_CONTROL_CMD, controlData, 2);
    delay(100);  // Wait for MOSFETs to switch
    return result;
}

bool BMSManager::verifyMosfetState(bool expectedCharge, bool expectedDischarge) {
    // Read current state
    if (!requestBasicInfo()) {
        return false;
    }
    
    // Check if states match expected
    return (data.chargeMosfetOn == expectedCharge && 
            data.dischargeMosfetOn == expectedDischarge);
}

// ============================================================================
// Utility Functions
// ============================================================================

float BMSManager::getMinCellVoltage() const {
    if (!data.dataValid || data.numCells == 0) {
        return 0.0f;
    }
    
    uint16_t minVoltage = data.cellVoltages[0];
    for (uint8_t i = 1; i < data.numCells; i++) {
        if (data.cellVoltages[i] < minVoltage) {
            minVoltage = data.cellVoltages[i];
        }
    }
    return minVoltage / 1000.0f;  // Convert mV to V
}

float BMSManager::getMaxCellVoltage() const {
    if (!data.dataValid || data.numCells == 0) {
        return 0.0f;
    }
    
    uint16_t maxVoltage = data.cellVoltages[0];
    for (uint8_t i = 1; i < data.numCells; i++) {
        if (data.cellVoltages[i] > maxVoltage) {
            maxVoltage = data.cellVoltages[i];
        }
    }
    return maxVoltage / 1000.0f;  // Convert mV to V
}

float BMSManager::getCellVoltageDelta() const {
    return getMaxCellVoltage() - getMinCellVoltage();
}

float BMSManager::getAverageCellVoltage() const {
    if (!data.dataValid || data.numCells == 0) {
        return 0.0f;
    }
    
    uint32_t sum = 0;
    for (uint8_t i = 0; i < data.numCells; i++) {
        sum += data.cellVoltages[i];
    }
    return (sum / data.numCells) / 1000.0f;  // Convert mV to V
}

bool BMSManager::hasProtectionActive() const {
    return data.anyProtectionActive;
}

// ============================================================================
// Direct Data Update from External Buffer
// ============================================================================

void BMSManager::updateFromBuffer(uint8_t* buffer, uint8_t length) {
    if (length < 30) {
        data.dataValid = false;
        return;
    }
    
    // Parse JBD BMS response (buffer starts with 0xDD header)
    // buffer[0] = 0xDD (start)
    // buffer[1] = 0x03 (cmd echo)
    // buffer[2] = 0x00 (status)
    // buffer[3] = length
    // buffer[4+] = data
    
    data.totalVoltage = ((buffer[4] << 8) | buffer[5]) * 0.01f;
    data.current = ((int16_t)((buffer[6] << 8) | buffer[7])) * 0.01f;
    data.remainingCapacity = ((buffer[8] << 8) | buffer[9]) * 0.01f;
    data.nominalCapacity = ((buffer[10] << 8) | buffer[11]) * 0.01f;
    data.cycles = (buffer[12] << 8) | buffer[13];
    
    // SOC at buffer[23]
    if (length >= 24) {
        data.socPercent = buffer[23];
    }
    
    // FET status at buffer[24]
    if (length >= 25) {
        uint8_t fetStatus = buffer[24];
        data.chargeMosfetOn = fetStatus & 0x01;
        data.dischargeMosfetOn = fetStatus & 0x02;
    }
    
    // Number of cells
    if (length >= 26) {
        data.numCells = buffer[25];
    }
    
    // Temperature
    if (length >= 29) {
        uint16_t temp1Raw = (buffer[27] << 8) | buffer[28];
        data.temp1 = (temp1Raw - 2731) / 10.0f;
    }
    
    data.dataValid = true;
    data.lastUpdate = millis();
    lastSuccessfulRead = millis();
}

