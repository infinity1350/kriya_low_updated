#!/usr/bin/env python3
"""
JBD BMS Monitoring and Control System
Production-ready Python application for 24S 100A JBD BMS

Features:
- Complete protocol implementation (0x03, 0x04, 0x05, 0xE1)
- Real-time monitoring with health analysis
- MOS control for charging/discharging with safety features
- CSV data logging with timestamps
- Interactive menu system with ANSI colors
- YAML configuration support
- Comprehensive error handling and retry logic

Author: Auto-generated
License: MIT
"""

import serial
import serial.tools.list_ports
import struct
import time
import yaml
import csv
import os
import sys
import argparse
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Any
from datetime import datetime
from collections import deque
from enum import IntEnum
import threading
import signal


# ============================================================================
# CONSTANTS AND ENUMS
# ============================================================================

class BMSCommand(IntEnum):
    """BMS UART Command Codes"""
    READ_BASIC_INFO = 0x03
    READ_CELL_VOLTAGES = 0x04
    READ_HW_VERSION = 0x05
    CONTROL_MOS = 0xE1


class MOSControl(IntEnum):
    """MOS Control Values for 0xE1 Command"""
    RELEASE_LOCK = 0x00
    DISABLE_CHARGING = 0x01
    DISABLE_DISCHARGING = 0x02
    DISABLE_BOTH = 0x03


# ANSI Color Codes
class Colors:
    """ANSI color codes for terminal output"""
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    RESET = '\033[0m'
    BOLD = '\033[1m'
    DIM = '\033[2m'


# Protocol Constants
FRAME_START = 0xDD
FRAME_STOP = 0x77
STATE_READ = 0xA5
STATE_WRITE = 0x5A


# ============================================================================
# DATA STRUCTURES
# ============================================================================

@dataclass
class ProtectionStatus:
    """Protection status flags from BMS"""
    cell_overvoltage: bool = False
    cell_undervoltage: bool = False
    pack_overvoltage: bool = False
    pack_undervoltage: bool = False
    charge_over_temp: bool = False
    charge_low_temp: bool = False
    discharge_over_temp: bool = False
    discharge_low_temp: bool = False
    charge_overcurrent: bool = False
    discharge_overcurrent: bool = False
    short_circuit: bool = False
    frontend_ic_error: bool = False
    software_lock: bool = False
    
    def has_any_protection(self) -> bool:
        """Check if any protection is active"""
        return any([
            self.cell_overvoltage, self.cell_undervoltage,
            self.pack_overvoltage, self.pack_undervoltage,
            self.charge_over_temp, self.charge_low_temp,
            self.discharge_over_temp, self.discharge_low_temp,
            self.charge_overcurrent, self.discharge_overcurrent,
            self.short_circuit, self.frontend_ic_error,
            self.software_lock
        ])
    
    def get_active_protections(self) -> List[str]:
        """Get list of active protection names"""
        active = []
        if self.cell_overvoltage: active.append("Cell Overvoltage")
        if self.cell_undervoltage: active.append("Cell Undervoltage")
        if self.pack_overvoltage: active.append("Pack Overvoltage")
        if self.pack_undervoltage: active.append("Pack Undervoltage")
        if self.charge_over_temp: active.append("Charge Over-Temperature")
        if self.charge_low_temp: active.append("Charge Low-Temperature")
        if self.discharge_over_temp: active.append("Discharge Over-Temperature")
        if self.discharge_low_temp: active.append("Discharge Low-Temperature")
        if self.charge_overcurrent: active.append("Charge Overcurrent")
        if self.discharge_overcurrent: active.append("Discharge Overcurrent")
        if self.short_circuit: active.append("Short Circuit")
        if self.frontend_ic_error: active.append("Front-end IC Error")
        if self.software_lock: active.append("Software Lock")
        return active


@dataclass
class BasicInfo:
    """Basic BMS information from 0x03 command"""
    total_voltage: float  # Volts
    current: float  # Amps (positive = charging, negative = discharging)
    remaining_capacity: float  # Ah
    nominal_capacity: float  # Ah
    cycle_count: int
    production_date: int
    balance_status: int  # 32-bit bitfield
    protection_status: ProtectionStatus
    software_version: int
    soc: int  # State of charge (%)
    fet_status: int  # FET control status
    num_cells: int
    num_temps: int
    temperatures: List[float] = field(default_factory=list)  # Celsius
    charge_mosfet_on: bool = False
    discharge_mosfet_on: bool = False
    timestamp: datetime = field(default_factory=datetime.now)


@dataclass
class CellData:
    """Cell voltage data with statistics"""
    voltages: List[float]  # Individual cell voltages in Volts
    min_voltage: float
    max_voltage: float
    average_voltage: float
    voltage_delta: float  # Max - Min
    min_cell_index: int
    max_cell_index: int
    timestamp: datetime = field(default_factory=datetime.now)
    
    def get_imbalance_percent(self) -> float:
        """Calculate cell imbalance as percentage"""
        if self.average_voltage > 0:
            return (self.voltage_delta / self.average_voltage) * 100
        return 0.0


@dataclass
class BMSSnapshot:
    """Complete BMS state snapshot"""
    basic_info: Optional[BasicInfo] = None
    cell_data: Optional[CellData] = None
    hardware_version: Optional[str] = None
    timestamp: datetime = field(default_factory=datetime.now)
    
    def is_complete(self) -> bool:
        """Check if snapshot has all essential data"""
        return self.basic_info is not None and self.cell_data is not None


# ============================================================================
# CLASS 1: BMSProtocol - Low-level protocol implementation
# ============================================================================

class BMSProtocol:
    """
    JBD BMS UART Protocol Implementation
    
    Handles frame construction, checksum calculation, and data parsing
    according to JBD BMS protocol specification.
    """
    
    @staticmethod
    def calculate_checksum(data: bytes) -> Tuple[int, int]:
        """
        Calculate 16-bit checksum for BMS protocol
        
        Checksum = (~sum + 1) & 0xFFFF
        
        Args:
            data: Bytes to checksum (typically CMD + LEN + DATA)
            
        Returns:
            Tuple of (checksum_high, checksum_low)
        """
        checksum = (~sum(data) + 1) & 0xFFFF
        checksum_high = (checksum >> 8) & 0xFF
        checksum_low = checksum & 0xFF
        return checksum_high, checksum_low
    
    @staticmethod
    def build_read_frame(cmd: int) -> bytes:
        """
        Build a read command frame
        
        Frame: [START] [READ] [CMD] [LEN] [CHECKSUM_H] [CHECKSUM_L] [STOP]
        
        Args:
            cmd: Command code (0x03, 0x04, 0x05)
            
        Returns:
            Complete frame as bytes
        """
        data = bytes([cmd, 0x00])  # CMD + LEN (0 for read commands)
        checksum_h, checksum_l = BMSProtocol.calculate_checksum(data)
        
        frame = bytes([
            FRAME_START,
            STATE_READ,
            cmd,
            0x00,
            checksum_h,
            checksum_l,
            FRAME_STOP
        ])
        
        return frame
    
    @staticmethod
    def build_write_frame(cmd: int, data: bytes) -> bytes:
        """
        Build a write command frame
        
        Frame: [START] [WRITE] [CMD] [LEN] [DATA...] [CHECKSUM_H] [CHECKSUM_L] [STOP]
        
        Args:
            cmd: Command code (0xE1)
            data: Data bytes to send
            
        Returns:
            Complete frame as bytes
        """
        length = len(data)
        checksum_data = bytes([cmd, length]) + data
        checksum_h, checksum_l = BMSProtocol.calculate_checksum(checksum_data)
        
        frame = bytes([FRAME_START, STATE_WRITE, cmd, length]) + data + bytes([checksum_h, checksum_l, FRAME_STOP])
        
        return frame
    
    @staticmethod
    def validate_frame(frame: bytes) -> bool:
        """
        Validate received frame structure and checksum
        
        Args:
            frame: Received frame bytes
            
        Returns:
            True if valid, False otherwise
        """
        if len(frame) < 7:  # Minimum frame size
            return False
        
        if frame[0] != FRAME_START or frame[-1] != FRAME_STOP:
            return False
        
        # Extract checksum from frame
        received_checksum_h = frame[-3]
        received_checksum_l = frame[-2]
        
        # Calculate expected checksum (CMD + LEN + DATA)
        data_to_check = frame[2:-3]  # Skip START, STATE and last 3 bytes (CHKSUM_H, CHKSUM_L, STOP)
        expected_checksum_h, expected_checksum_l = BMSProtocol.calculate_checksum(data_to_check)
        
        return (received_checksum_h == expected_checksum_h and 
                received_checksum_l == expected_checksum_l)
    
    @staticmethod
    def parse_protection_status(status: int) -> ProtectionStatus:
        """
        Parse 16-bit protection status field
        
        Args:
            status: 16-bit protection status value
            
        Returns:
            ProtectionStatus object
        """
        return ProtectionStatus(
            cell_overvoltage=bool(status & (1 << 0)),
            cell_undervoltage=bool(status & (1 << 1)),
            pack_overvoltage=bool(status & (1 << 2)),
            pack_undervoltage=bool(status & (1 << 3)),
            charge_over_temp=bool(status & (1 << 4)),
            charge_low_temp=bool(status & (1 << 5)),
            discharge_over_temp=bool(status & (1 << 6)),
            discharge_low_temp=bool(status & (1 << 7)),
            charge_overcurrent=bool(status & (1 << 8)),
            discharge_overcurrent=bool(status & (1 << 9)),
            short_circuit=bool(status & (1 << 10)),
            frontend_ic_error=bool(status & (1 << 11)),
            software_lock=bool(status & (1 << 12))
        )
    
    @staticmethod
    def parse_basic_info(data: bytes) -> BasicInfo:
        """
        Parse response from 0x03 Read Basic Information command
        
        Args:
            data: Response data bytes (excluding frame headers/footers)
            
        Returns:
            BasicInfo object
        """
        if len(data) < 23:
            raise ValueError(f"Invalid basic info response length: {len(data)}")
        
        # Parse according to protocol specification
        total_voltage = struct.unpack('>H', data[0:2])[0] * 0.01  # 10mV units -> V
        current_raw = struct.unpack('>h', data[2:4])[0]  # Signed 16-bit
        current = current_raw * 0.01  # 10mA units -> A
        remaining_capacity = struct.unpack('>H', data[4:6])[0] * 0.01  # 10mAh -> Ah
        nominal_capacity = struct.unpack('>H', data[6:8])[0] * 0.01  # 10mAh -> Ah
        cycle_count = struct.unpack('>H', data[8:10])[0]
        production_date = struct.unpack('>H', data[10:12])[0]
        balance_status = struct.unpack('>I', data[12:16])[0]  # 32-bit
        protection_status_raw = struct.unpack('>H', data[16:18])[0]  # 16-bit
        software_version = data[18]
        soc = data[19]  # State of charge %
        fet_status = data[20]
        num_cells = data[21]
        num_temps = data[22]
        
        # Parse FET status (bit 0 = charge, bit 1 = discharge)
        charge_mosfet_on = bool(fet_status & 0x01)
        discharge_mosfet_on = bool(fet_status & 0x02)
        
        # Parse temperature sensors (if present)
        temperatures = []
        temp_offset = 23
        for i in range(num_temps):
            if temp_offset + 1 < len(data):
                temp_raw = struct.unpack('>H', data[temp_offset:temp_offset+2])[0]
                # Temperature in 0.1K units, offset by 2731 (273.1K = 0°C)
                temp_celsius = (temp_raw - 2731) / 10.0
                temperatures.append(temp_celsius)
                temp_offset += 2
        
        protection_status = BMSProtocol.parse_protection_status(protection_status_raw)
        
        return BasicInfo(
            total_voltage=total_voltage,
            current=current,
            remaining_capacity=remaining_capacity,
            nominal_capacity=nominal_capacity,
            cycle_count=cycle_count,
            production_date=production_date,
            balance_status=balance_status,
            protection_status=protection_status,
            software_version=software_version,
            soc=soc,
            fet_status=fet_status,
            num_cells=num_cells,
            num_temps=num_temps,
            temperatures=temperatures,
            charge_mosfet_on=charge_mosfet_on,
            discharge_mosfet_on=discharge_mosfet_on
        )
    
    @staticmethod
    def parse_cell_voltages(data: bytes, num_cells: int) -> CellData:
        """
        Parse response from 0x04 Read Cell Voltages command
        
        Args:
            data: Response data bytes
            num_cells: Expected number of cells
            
        Returns:
            CellData object with voltages and statistics
        """
        voltages = []
        
        for i in range(num_cells):
            offset = i * 2
            if offset + 1 < len(data):
                voltage_raw = struct.unpack('>H', data[offset:offset+2])[0]
                voltage = voltage_raw / 1000.0  # mV -> V
                voltages.append(voltage)
        
        if not voltages:
            raise ValueError("No cell voltages parsed")
        
        min_voltage = min(voltages)
        max_voltage = max(voltages)
        average_voltage = sum(voltages) / len(voltages)
        voltage_delta = max_voltage - min_voltage
        min_cell_index = voltages.index(min_voltage)
        max_cell_index = voltages.index(max_voltage)
        
        return CellData(
            voltages=voltages,
            min_voltage=min_voltage,
            max_voltage=max_voltage,
            average_voltage=average_voltage,
            voltage_delta=voltage_delta,
            min_cell_index=min_cell_index,
            max_cell_index=max_cell_index
        )
    
    @staticmethod
    def parse_hardware_version(data: bytes) -> str:
        """
        Parse response from 0x05 Read Hardware Version command
        
        Args:
            data: Response data bytes (ASCII string)
            
        Returns:
            Hardware version string
        """
        try:
            return data.decode('ascii').strip('\x00')
        except UnicodeDecodeError:
            return data.hex()


# ============================================================================
# CLASS 2: BMSCommunication - Serial communication with retry logic
# ============================================================================

class BMSCommunication:
    """
    Serial communication layer for BMS
    
    Handles connection management, command sending, and automatic retry logic.
    """
    
    def __init__(self, port: str = "", baudrate: int = 9600, 
                 timeout: float = 2.0, retry_attempts: int = 3,
                 retry_delay: float = 0.5):
        """
        Initialize BMS communication
        
        Args:
            port: Serial port path (empty for auto-detect)
            baudrate: Baud rate (default 9600)
            timeout: Read timeout in seconds
            retry_attempts: Number of retry attempts
            retry_delay: Delay between retries in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.retry_attempts = retry_attempts
        self.retry_delay = retry_delay
        self.serial_conn: Optional[serial.Serial] = None
        self.protocol = BMSProtocol()
        self._lock = threading.Lock()
    
    def auto_detect_port(self) -> Optional[str]:
        """
        Auto-detect BMS serial port
        
        Returns:
            Port path if found, None otherwise
        """
        ports = list(serial.tools.list_ports.comports())
        
        # Try to find likely BMS ports
        for port in ports:
            # Common USB-to-Serial adapters
            if any(keyword in port.description.lower() for keyword in 
                   ['usb', 'serial', 'uart', 'ch340', 'cp210', 'ftdi']):
                return port.device
        
        # Return first available port if no obvious match
        if ports:
            return ports[0].device
        
        return None
    
    def connect(self, port: str = "") -> bool:
        """
        Connect to BMS
        
        Args:
            port: Serial port (uses stored port if empty)
            
        Returns:
            True if connected successfully
        """
        if port:
            self.port = port
        
        if not self.port:
            detected_port = self.auto_detect_port()
            if detected_port:
                self.port = detected_port
            else:
                return False
        
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            time.sleep(0.1)  # Allow connection to stabilize
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            return False
    
    def disconnect(self) -> None:
        """Disconnect from BMS"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.serial_conn = None
    
    def is_connected(self) -> bool:
        """Check if connected to BMS"""
        return self.serial_conn is not None and self.serial_conn.is_open
    
    def _send_raw_command(self, frame: bytes) -> Optional[bytes]:
        """
        Send raw command frame and receive response
        
        Args:
            frame: Command frame to send
            
        Returns:
            Response data bytes (excluding frame overhead) or None on error
        """
        if not self.is_connected():
            return None
        
        with self._lock:
            try:
                # Clear input buffer
                self.serial_conn.reset_input_buffer()
                
                # Send command
                self.serial_conn.write(frame)
                self.serial_conn.flush()
                
                # Read response header first (4 bytes: START, STATE, CMD, LEN)
                header = self.serial_conn.read(4)
                if len(header) < 4:
                    return None
                
                if header[0] != FRAME_START:
                    return None
                
                data_length = header[3]
                
                # Read rest of frame (data + checksum + stop)
                remaining = self.serial_conn.read(data_length + 3)
                
                full_frame = header + remaining
                
                # Validate frame
                if not self.protocol.validate_frame(full_frame):
                    return None
                
                # Extract data portion (skip header and footer)
                data = full_frame[4:-3]
                
                return data
                
            except serial.SerialException as e:
                print(f"Serial communication error: {e}")
                return None
    
    def send_command(self, cmd: int, data: bytes = b'') -> Optional[bytes]:
        """
        Send command with automatic retry
        
        Args:
            cmd: Command code
            data: Data bytes for write commands
            
        Returns:
            Response data or None on failure
        """
        if data:
            frame = self.protocol.build_write_frame(cmd, data)
        else:
            frame = self.protocol.build_read_frame(cmd)
        
        for attempt in range(self.retry_attempts):
            response = self._send_raw_command(frame)
            if response is not None:
                return response
            
            if attempt < self.retry_attempts - 1:
                time.sleep(self.retry_delay)
        
        return None
    
    def read_basic_info(self) -> Optional[BasicInfo]:
        """
        Read basic BMS information (0x03)
        
        Returns:
            BasicInfo object or None on error
        """
        data = self.send_command(BMSCommand.READ_BASIC_INFO)
        if data:
            try:
                return self.protocol.parse_basic_info(data)
            except Exception as e:
                print(f"Error parsing basic info: {e}")
        return None
    
    def read_cell_voltages(self, num_cells: int = 24) -> Optional[CellData]:
        """
        Read cell voltages (0x04)
        
        Args:
            num_cells: Expected number of cells
            
        Returns:
            CellData object or None on error
        """
        data = self.send_command(BMSCommand.READ_CELL_VOLTAGES)
        if data:
            try:
                return self.protocol.parse_cell_voltages(data, num_cells)
            except Exception as e:
                print(f"Error parsing cell voltages: {e}")
        return None
    
    def read_hardware_version(self) -> Optional[str]:
        """
        Read hardware version (0x05)
        
        Returns:
            Hardware version string or None on error
        """
        data = self.send_command(BMSCommand.READ_HW_VERSION)
        if data:
            try:
                return self.protocol.parse_hardware_version(data)
            except Exception as e:
                print(f"Error parsing hardware version: {e}")
        return None
    
    def control_mos(self, control_value: int) -> bool:
        """
        Send MOS control command (0xE1)
        
        Args:
            control_value: Control value (0x00-0x03)
            
        Returns:
            True if command sent successfully
        """
        data = bytes([0x00, control_value])
        response = self.send_command(BMSCommand.CONTROL_MOS, data)
        return response is not None


# ============================================================================
# CLASS 3: BMSController - MOS control with safety features
# ============================================================================

class BMSController:
    """
    BMS control operations with safety features
    
    Manages MOS switching for charging and discharging control.
    """
    
    def __init__(self, communication: BMSCommunication, 
                 require_confirmation: bool = True,
                 verification_delay: float = 0.5):
        """
        Initialize BMS controller
        
        Args:
            communication: BMSCommunication instance
            require_confirmation: Require user confirmation for control ops
            verification_delay: Delay before verifying control operations
        """
        self.comm = communication
        self.require_confirmation = require_confirmation
        self.verification_delay = verification_delay
    
    def _get_current_state(self) -> Optional[BasicInfo]:
        """Get current BMS state"""
        return self.comm.read_basic_info()
    
    def verify_mos_state(self) -> Optional[Dict[str, bool]]:
        """
        Verify current MOS state
        
        Returns:
            Dictionary with charge_mosfet_on and discharge_mosfet_on states
        """
        info = self._get_current_state()
        if info:
            return {
                'charge_mosfet_on': info.charge_mosfet_on,
                'discharge_mosfet_on': info.discharge_mosfet_on
            }
        return None
    
    def _confirm_action(self, action: str, warning: str) -> bool:
        """
        Get user confirmation for control action
        
        Args:
            action: Action description
            warning: Warning message
            
        Returns:
            True if user confirms
        """
        if not self.require_confirmation:
            return True
        
        print(f"\n{Colors.YELLOW}{Colors.BOLD}WARNING:{Colors.RESET} {warning}")
        print(f"Action: {action}")
        response = input(f"\nType '{Colors.BOLD}CONFIRM{Colors.RESET}' to proceed: ").strip()
        return response == "CONFIRM"
    
    def release_software_lock(self, confirm: bool = True) -> bool:
        """
        Release software lock (0x00)
        
        Args:
            confirm: Require confirmation
            
        Returns:
            True if successful
        """
        state = self._get_current_state()
        if not state:
            print(f"{Colors.RED}Error: Cannot read BMS state{Colors.RESET}")
            return False
        
        print(f"\n{Colors.CYAN}Current State:{Colors.RESET}")
        print(f"  Software Lock: {'ACTIVE' if state.protection_status.software_lock else 'NOT ACTIVE'}")
        
        if not state.protection_status.software_lock:
            print(f"{Colors.YELLOW}Software lock is not active{Colors.RESET}")
            return True
        
        if confirm and self.require_confirmation:
            if not self._confirm_action(
                "Release software lock",
                "This will release the BMS software lock protection"
            ):
                print("Operation cancelled")
                return False
        
        # Send control command
        if not self.comm.control_mos(MOSControl.RELEASE_LOCK):
            print(f"{Colors.RED}Failed to send command{Colors.RESET}")
            return False
        
        # Verify
        time.sleep(self.verification_delay)
        new_state = self._get_current_state()
        
        if new_state and not new_state.protection_status.software_lock:
            print(f"{Colors.GREEN}✓ Software lock released successfully{Colors.RESET}")
            return True
        else:
            print(f"{Colors.YELLOW}Command sent, but verification inconclusive{Colors.RESET}")
            return True
    
    def disable_charging(self, confirm: bool = True) -> bool:
        """
        Disable charging MOS (0x01)
        
        Args:
            confirm: Require confirmation
            
        Returns:
            True if successful
        """
        state = self._get_current_state()
        if not state:
            print(f"{Colors.RED}Error: Cannot read BMS state{Colors.RESET}")
            return False
        
        print(f"\n{Colors.CYAN}Current FET Status:{Colors.RESET}")
        print(f"  Charge MOSFET: {'ON' if state.charge_mosfet_on else 'OFF'}")
        print(f"  Discharge MOSFET: {'ON' if state.discharge_mosfet_on else 'OFF'}")
        
        if not state.charge_mosfet_on:
            print(f"{Colors.YELLOW}Charging is already disabled{Colors.RESET}")
            return True
        
        if confirm and self.require_confirmation:
            if not self._confirm_action(
                "Disable charging MOSFET",
                "This will PREVENT charging current to the battery pack"
            ):
                print("Operation cancelled")
                return False
        
        # Send control command
        if not self.comm.control_mos(MOSControl.DISABLE_CHARGING):
            print(f"{Colors.RED}Failed to send command{Colors.RESET}")
            return False
        
        # Verify
        time.sleep(self.verification_delay)
        new_state = self._get_current_state()
        
        if new_state:
            print(f"\n{Colors.CYAN}New FET Status:{Colors.RESET}")
            print(f"  Charge MOSFET: {'ON' if new_state.charge_mosfet_on else 'OFF'}")
            print(f"  Discharge MOSFET: {'ON' if new_state.discharge_mosfet_on else 'OFF'}")
            
            if not new_state.charge_mosfet_on:
                print(f"{Colors.GREEN}✓ Charging disabled successfully{Colors.RESET}")
                return True
        
        print(f"{Colors.YELLOW}Command sent, verify state manually{Colors.RESET}")
        return True
    
    def disable_discharging(self, confirm: bool = True) -> bool:
        """
        Disable discharging MOS (0x02)
        
        Args:
            confirm: Require confirmation
            
        Returns:
            True if successful
        """
        state = self._get_current_state()
        if not state:
            print(f"{Colors.RED}Error: Cannot read BMS state{Colors.RESET}")
            return False
        
        print(f"\n{Colors.CYAN}Current FET Status:{Colors.RESET}")
        print(f"  Charge MOSFET: {'ON' if state.charge_mosfet_on else 'OFF'}")
        print(f"  Discharge MOSFET: {'ON' if state.discharge_mosfet_on else 'OFF'}")
        
        if not state.discharge_mosfet_on:
            print(f"{Colors.YELLOW}Discharging is already disabled{Colors.RESET}")
            return True
        
        if confirm and self.require_confirmation:
            if not self._confirm_action(
                "Disable discharging MOSFET",
                "This will PREVENT discharge current from the battery pack (NO OUTPUT)"
            ):
                print("Operation cancelled")
                return False
        
        # Send control command
        if not self.comm.control_mos(MOSControl.DISABLE_DISCHARGING):
            print(f"{Colors.RED}Failed to send command{Colors.RESET}")
            return False
        
        # Verify
        time.sleep(self.verification_delay)
        new_state = self._get_current_state()
        
        if new_state:
            print(f"\n{Colors.CYAN}New FET Status:{Colors.RESET}")
            print(f"  Charge MOSFET: {'ON' if new_state.charge_mosfet_on else 'OFF'}")
            print(f"  Discharge MOSFET: {'ON' if new_state.discharge_mosfet_on else 'OFF'}")
            
            if not new_state.discharge_mosfet_on:
                print(f"{Colors.GREEN}✓ Discharging disabled successfully{Colors.RESET}")
                return True
        
        print(f"{Colors.YELLOW}Command sent, verify state manually{Colors.RESET}")
        return True
    
    def disable_both(self, confirm: bool = True) -> bool:
        """
        Disable both charging and discharging MOS (0x03)
        
        Args:
            confirm: Require confirmation
            
        Returns:
            True if successful
        """
        state = self._get_current_state()
        if not state:
            print(f"{Colors.RED}Error: Cannot read BMS state{Colors.RESET}")
            return False
        
        print(f"\n{Colors.CYAN}Current FET Status:{Colors.RESET}")
        print(f"  Charge MOSFET: {'ON' if state.charge_mosfet_on else 'OFF'}")
        print(f"  Discharge MOSFET: {'ON' if state.discharge_mosfet_on else 'OFF'}")
        
        if confirm and self.require_confirmation:
            if not self._confirm_action(
                "Disable BOTH charging and discharging MOSFETs",
                "This will DISABLE ALL CURRENT FLOW (emergency shutdown)"
            ):
                print("Operation cancelled")
                return False
        
        # Send control command
        if not self.comm.control_mos(MOSControl.DISABLE_BOTH):
            print(f"{Colors.RED}Failed to send command{Colors.RESET}")
            return False
        
        # Verify
        time.sleep(self.verification_delay)
        new_state = self._get_current_state()
        
        if new_state:
            print(f"\n{Colors.CYAN}New FET Status:{Colors.RESET}")
            print(f"  Charge MOSFET: {'ON' if new_state.charge_mosfet_on else 'OFF'}")
            print(f"  Discharge MOSFET: {'ON' if new_state.discharge_mosfet_on else 'OFF'}")
            
            if not new_state.charge_mosfet_on and not new_state.discharge_mosfet_on:
                print(f"{Colors.GREEN}✓ Both MOSFETs disabled successfully{Colors.RESET}")
                return True
        
        print(f"{Colors.YELLOW}Command sent, verify state manually{Colors.RESET}")
        return True
    
    def emergency_shutdown(self) -> bool:
        """
        Emergency shutdown - disable all MOSFETs immediately
        
        Returns:
            True if successful
        """
        print(f"\n{Colors.RED}{Colors.BOLD}EMERGENCY SHUTDOWN INITIATED{Colors.RESET}")
        return self.disable_both(confirm=False)


# ============================================================================
# CLASS 4: BMSMonitor - Real-time monitoring and health assessment
# ============================================================================

class BMSMonitor:
    """
    BMS monitoring and health analysis
    
    Collects data, performs health analysis, generates alerts.
    """
    
    def __init__(self, communication: BMSCommunication, config: Dict[str, Any]):
        """
        Initialize BMS monitor
        
        Args:
            communication: BMSCommunication instance
            config: Configuration dictionary
        """
        self.comm = communication
        self.config = config
        self.history: deque = deque(maxlen=config.get('monitoring', {}).get('history_buffer_size', 300))
        self.last_alert_time: Dict[str, float] = {}
        self.alert_cooldown = config.get('monitoring', {}).get('alert_cooldown', 30)
    
    def collect_snapshot(self) -> Optional[BMSSnapshot]:
        """
        Collect complete BMS snapshot
        
        Returns:
            BMSSnapshot object or None on error
        """
        snapshot = BMSSnapshot()
        
        # Read basic info
        snapshot.basic_info = self.comm.read_basic_info()
        if not snapshot.basic_info:
            return None
        
        # Read cell voltages
        num_cells = self.config.get('battery', {}).get('cell_count', 24)
        snapshot.cell_data = self.comm.read_cell_voltages(num_cells)
        
        # Read hardware version (optional, don't fail if unavailable)
        snapshot.hardware_version = self.comm.read_hardware_version()
        
        # Add to history
        if snapshot.is_complete():
            self.history.append(snapshot)
        
        return snapshot
    
    def analyze_cell_health(self, cell_data: CellData) -> Dict[str, Any]:
        """
        Analyze cell health
        
        Args:
            cell_data: CellData object
            
        Returns:
            Health analysis dictionary
        """
        thresholds = self.config.get('thresholds', {})
        battery = self.config.get('battery', {})
        
        delta_warning = thresholds.get('cell_voltage_delta_warning', 0.05)
        delta_critical = thresholds.get('cell_voltage_delta_critical', 0.1)
        min_cell_v = battery.get('min_cell_voltage', 3.0)
        max_cell_v = battery.get('max_cell_voltage', 4.2)
        
        health = {
            'overall_status': 'good',
            'issues': [],
            'warnings': []
        }
        
        # Check voltage spread
        if cell_data.voltage_delta > delta_critical:
            health['overall_status'] = 'critical'
            health['issues'].append(f"Critical cell imbalance: {cell_data.voltage_delta:.3f}V")
        elif cell_data.voltage_delta > delta_warning:
            if health['overall_status'] == 'good':
                health['overall_status'] = 'warning'
            health['warnings'].append(f"Cell imbalance warning: {cell_data.voltage_delta:.3f}V")
        
        # Check individual cells
        for i, voltage in enumerate(cell_data.voltages):
            if voltage < min_cell_v:
                health['overall_status'] = 'critical'
                health['issues'].append(f"Cell {i+1} undervoltage: {voltage:.3f}V")
            elif voltage > max_cell_v:
                health['overall_status'] = 'critical'
                health['issues'].append(f"Cell {i+1} overvoltage: {voltage:.3f}V")
        
        return health
    
    def analyze_pack_health(self, basic_info: BasicInfo) -> Dict[str, Any]:
        """
        Analyze overall pack health
        
        Args:
            basic_info: BasicInfo object
            
        Returns:
            Health analysis dictionary
        """
        thresholds = self.config.get('thresholds', {})
        battery = self.config.get('battery', {})
        
        health = {
            'overall_status': 'good',
            'issues': [],
            'warnings': []
        }
        
        # Check protections
        if basic_info.protection_status.has_any_protection():
            health['overall_status'] = 'critical'
            for protection in basic_info.protection_status.get_active_protections():
                health['issues'].append(f"Protection active: {protection}")
        
        # Check temperatures
        temp_warning = thresholds.get('temperature_warning', 50)
        temp_critical = thresholds.get('temperature_critical', 60)
        temp_low = thresholds.get('temperature_low', 0)
        
        for i, temp in enumerate(basic_info.temperatures):
            if temp > temp_critical:
                health['overall_status'] = 'critical'
                health['issues'].append(f"Temp sensor {i+1} critical: {temp:.1f}°C")
            elif temp > temp_warning:
                if health['overall_status'] == 'good':
                    health['overall_status'] = 'warning'
                health['warnings'].append(f"Temp sensor {i+1} high: {temp:.1f}°C")
            elif temp < temp_low:
                if health['overall_status'] == 'good':
                    health['overall_status'] = 'warning'
                health['warnings'].append(f"Temp sensor {i+1} low: {temp:.1f}°C")
        
        # Check current
        current_abs = abs(basic_info.current)
        current_warning = thresholds.get('current_warning', 90)
        current_critical = thresholds.get('current_critical', 100)
        
        if current_abs > current_critical:
            health['overall_status'] = 'critical'
            health['issues'].append(f"Current critical: {current_abs:.1f}A")
        elif current_abs > current_warning:
            if health['overall_status'] == 'good':
                health['overall_status'] = 'warning'
            health['warnings'].append(f"Current high: {current_abs:.1f}A")
        
        # Check SOC
        soc_low = thresholds.get('soc_low_warning', 20)
        soc_critical = thresholds.get('soc_critical', 10)
        
        if basic_info.soc < soc_critical:
            health['overall_status'] = 'critical'
            health['issues'].append(f"SOC critical: {basic_info.soc}%")
        elif basic_info.soc < soc_low:
            if health['overall_status'] == 'good':
                health['overall_status'] = 'warning'
            health['warnings'].append(f"SOC low: {basic_info.soc}%")
        
        # Check capacity fade
        if basic_info.nominal_capacity > 0:
            capacity_ratio = basic_info.remaining_capacity / basic_info.nominal_capacity
            if capacity_ratio < 0.8:
                if health['overall_status'] == 'good':
                    health['overall_status'] = 'warning'
                health['warnings'].append(f"Capacity degradation detected: {capacity_ratio*100:.1f}% of nominal")
        
        return health
    
    def check_alerts(self, snapshot: BMSSnapshot) -> List[str]:
        """
        Check for alert conditions
        
        Args:
            snapshot: BMSSnapshot to check
            
        Returns:
            List of alert messages
        """
        alerts = []
        current_time = time.time()
        
        if not snapshot.is_complete():
            return alerts
        
        # Analyze health
        cell_health = self.analyze_cell_health(snapshot.cell_data)
        pack_health = self.analyze_pack_health(snapshot.basic_info)
        
        # Generate alerts with cooldown
        def add_alert(alert_key: str, message: str):
            last_time = self.last_alert_time.get(alert_key, 0)
            if current_time - last_time > self.alert_cooldown:
                alerts.append(message)
                self.last_alert_time[alert_key] = current_time
        
        # Cell health alerts
        if cell_health['overall_status'] == 'critical':
            for issue in cell_health['issues']:
                add_alert(f"cell_{issue}", issue)
        
        # Pack health alerts
        if pack_health['overall_status'] == 'critical':
            for issue in pack_health['issues']:
                add_alert(f"pack_{issue}", issue)
        
        return alerts
    
    def get_statistics(self, window: int = 60) -> Dict[str, Any]:
        """
        Get statistical analysis over time window
        
        Args:
            window: Number of samples to analyze
            
        Returns:
            Statistics dictionary
        """
        if not self.history:
            return {}
        
        # Get recent samples
        samples = list(self.history)[-window:]
        
        # Calculate statistics
        voltages = [s.basic_info.total_voltage for s in samples if s.basic_info]
        currents = [s.basic_info.current for s in samples if s.basic_info]
        socs = [s.basic_info.soc for s in samples if s.basic_info]
        
        temps = []
        for s in samples:
            if s.basic_info and s.basic_info.temperatures:
                temps.extend(s.basic_info.temperatures)
        
        stats = {}
        
        if voltages:
            stats['voltage'] = {
                'min': min(voltages),
                'max': max(voltages),
                'avg': sum(voltages) / len(voltages)
            }
        
        if currents:
            stats['current'] = {
                'min': min(currents),
                'max': max(currents),
                'avg': sum(currents) / len(currents)
            }
        
        if socs:
            stats['soc'] = {
                'min': min(socs),
                'max': max(socs),
                'avg': sum(socs) / len(socs)
            }
        
        if temps:
            stats['temperature'] = {
                'min': min(temps),
                'max': max(temps),
                'avg': sum(temps) / len(temps)
            }
        
        return stats
    
    def display_dashboard(self, snapshot: BMSSnapshot, use_colors: bool = True) -> None:
        """
        Display real-time monitoring dashboard
        
        Args:
            snapshot: BMSSnapshot to display
            use_colors: Enable color output
        """
        if not snapshot.is_complete():
            print("Incomplete data snapshot")
            return
        
        def color(text: str, color_code: str) -> str:
            if use_colors:
                return f"{color_code}{text}{Colors.RESET}"
            return text
        
        info = snapshot.basic_info
        cells = snapshot.cell_data
        
        # Clear screen (optional)
        # os.system('clear' if os.name == 'posix' else 'cls')
        
        print("\n" + "=" * 80)
        print(color(f"  JBD BMS MONITORING DASHBOARD - {snapshot.timestamp.strftime('%Y-%m-%d %H:%M:%S')}", Colors.BOLD))
        print("=" * 80)
        
        # Pack Status
        print(f"\n{color('PACK STATUS:', Colors.CYAN + Colors.BOLD)}")
        print(f"  Voltage:    {color(f'{info.total_voltage:.2f}V', Colors.GREEN)}")
        
        current_color = Colors.GREEN if abs(info.current) < 50 else Colors.YELLOW
        current_dir = "Charging" if info.current > 0 else "Discharging" if info.current < 0 else "Idle"
        print(f"  Current:    {color(f'{info.current:+.2f}A', current_color)} ({current_dir})")
        
        soc_color = Colors.GREEN if info.soc > 50 else Colors.YELLOW if info.soc > 20 else Colors.RED
        print(f"  SOC:        {color(f'{info.soc}%', soc_color)}")
        
        print(f"  Capacity:   {info.remaining_capacity:.2f}Ah / {info.nominal_capacity:.2f}Ah")
        print(f"  Cycles:     {info.cycle_count}")
        
        # FET Status
        print(f"\n{color('FET STATUS:', Colors.CYAN + Colors.BOLD)}")
        charge_status = color("ON", Colors.GREEN) if info.charge_mosfet_on else color("OFF", Colors.RED)
        discharge_status = color("ON", Colors.GREEN) if info.discharge_mosfet_on else color("OFF", Colors.RED)
        print(f"  Charge:     {charge_status}")
        print(f"  Discharge:  {discharge_status}")
        
        # Cell Voltages
        print(f"\n{color('CELL VOLTAGES:', Colors.CYAN + Colors.BOLD)}")
        print(f"  Min:        {cells.min_voltage:.3f}V (Cell {cells.min_cell_index + 1})")
        print(f"  Max:        {cells.max_voltage:.3f}V (Cell {cells.max_cell_index + 1})")
        print(f"  Average:    {cells.average_voltage:.3f}V")
        
        delta_color = Colors.GREEN if cells.voltage_delta < 0.05 else Colors.YELLOW if cells.voltage_delta < 0.1 else Colors.RED
        print(f"  Delta:      {color(f'{cells.voltage_delta:.3f}V', delta_color)}")
        print(f"  Imbalance:  {cells.get_imbalance_percent():.2f}%")
        
        # Temperatures
        if info.temperatures:
            print(f"\n{color('TEMPERATURES:', Colors.CYAN + Colors.BOLD)}")
            for i, temp in enumerate(info.temperatures):
                temp_color = Colors.GREEN if temp < 40 else Colors.YELLOW if temp < 50 else Colors.RED
                print(f"  Sensor {i+1}:   {color(f'{temp:.1f}°C', temp_color)}")
        
        # Protection Status
        if info.protection_status.has_any_protection():
            print(f"\n{color('ACTIVE PROTECTIONS:', Colors.RED + Colors.BOLD)}")
            for protection in info.protection_status.get_active_protections():
                print(f"  {color('⚠ ' + protection, Colors.RED)}")
        
        # Health Analysis
        cell_health = self.analyze_cell_health(cells)
        pack_health = self.analyze_pack_health(info)
        
        overall_health = 'critical' if (cell_health['overall_status'] == 'critical' or 
                                       pack_health['overall_status'] == 'critical') else \
                        'warning' if (cell_health['overall_status'] == 'warning' or 
                                     pack_health['overall_status'] == 'warning') else 'good'
        
        health_color = Colors.GREEN if overall_health == 'good' else Colors.YELLOW if overall_health == 'warning' else Colors.RED
        print(f"\n{color('OVERALL HEALTH:', Colors.CYAN + Colors.BOLD)} {color(overall_health.upper(), health_color)}")
        
        if cell_health['warnings'] or pack_health['warnings']:
            print(f"\n{color('Warnings:', Colors.YELLOW)}")
            for warning in cell_health['warnings'] + pack_health['warnings']:
                print(f"  {color('⚠ ' + warning, Colors.YELLOW)}")
        
        if cell_health['issues'] or pack_health['issues']:
            print(f"\n{color('Issues:', Colors.RED)}")
            for issue in cell_health['issues'] + pack_health['issues']:
                print(f"  {color('✗ ' + issue, Colors.RED)}")
        
        print("\n" + "=" * 80)


# ============================================================================
# CLASS 5: DataLogger - CSV data logging
# ============================================================================

class DataLogger:
    """
    CSV data logger for BMS data
    
    Logs snapshots to CSV files with automatic rotation.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize data logger
        
        Args:
            config: Configuration dictionary
        """
        self.config = config
        self.logging_config = config.get('logging', {})
        self.directory = self.logging_config.get('directory', './bms_logs')
        self.enabled = self.logging_config.get('enabled', True)
        self.interval = self.logging_config.get('interval', 5)
        self.max_file_size_mb = self.logging_config.get('max_file_size_mb', 10)
        self.rotation = self.logging_config.get('rotation', 'daily')
        
        self.current_file: Optional[str] = None
        self.csv_writer: Optional[csv.DictWriter] = None
        self.file_handle: Optional[Any] = None
        self.logging_thread: Optional[threading.Thread] = None
        self.stop_logging_event = threading.Event()
        
        # Create log directory
        if self.enabled:
            os.makedirs(self.directory, exist_ok=True)
    
    def _get_log_filename(self) -> str:
        """Generate log filename based on rotation setting"""
        timestamp = datetime.now()
        
        if self.rotation == 'daily':
            return os.path.join(self.directory, f"bms_data_{timestamp.strftime('%Y%m%d')}.csv")
        else:
            return os.path.join(self.directory, f"bms_data_{timestamp.strftime('%Y%m%d_%H%M%S')}.csv")
    
    def _should_rotate(self) -> bool:
        """Check if log file should be rotated"""
        if not self.current_file or not os.path.exists(self.current_file):
            return True
        
        if self.rotation == 'daily':
            # Check if date has changed
            file_date = os.path.basename(self.current_file).split('_')[2].split('.')[0]
            current_date = datetime.now().strftime('%Y%m%d')
            if file_date != current_date:
                return True
        
        if self.rotation == 'size':
            # Check file size
            file_size_mb = os.path.getsize(self.current_file) / (1024 * 1024)
            if file_size_mb > self.max_file_size_mb:
                return True
        
        return False
    
    def _open_log_file(self) -> bool:
        """Open or create log file"""
        try:
            filename = self._get_log_filename()
            file_exists = os.path.exists(filename)
            
            self.file_handle = open(filename, 'a', newline='')
            self.current_file = filename
            
            # Define CSV columns
            num_cells = self.config.get('battery', {}).get('cell_count', 24)
            
            fieldnames = [
                'timestamp',
                'total_voltage',
                'current',
                'remaining_capacity',
                'nominal_capacity',
                'soc',
                'cycle_count',
                'charge_mosfet',
                'discharge_mosfet'
            ]
            
            # Add cell voltages
            for i in range(num_cells):
                fieldnames.append(f'cell_{i+1}_voltage')
            
            # Add temperatures
            fieldnames.extend(['temp_1', 'temp_2', 'temp_3', 'temp_4'])
            
            # Add protection flags
            fieldnames.extend([
                'protection_active',
                'software_lock',
                'cell_overvoltage',
                'cell_undervoltage'
            ])
            
            self.csv_writer = csv.DictWriter(self.file_handle, fieldnames=fieldnames)
            
            # Write header if new file
            if not file_exists:
                self.csv_writer.writeheader()
                self.file_handle.flush()
            
            return True
            
        except Exception as e:
            print(f"Error opening log file: {e}")
            return False
    
    def _close_log_file(self) -> None:
        """Close current log file"""
        if self.file_handle:
            self.file_handle.close()
            self.file_handle = None
            self.csv_writer = None
    
    def log_snapshot(self, snapshot: BMSSnapshot) -> bool:
        """
        Log a snapshot to CSV
        
        Args:
            snapshot: BMSSnapshot to log
            
        Returns:
            True if logged successfully
        """
        if not self.enabled or not snapshot.is_complete():
            return False
        
        # Check for rotation
        if self._should_rotate():
            self._close_log_file()
            if not self._open_log_file():
                return False
        
        # Ensure file is open
        if not self.csv_writer:
            if not self._open_log_file():
                return False
        
        try:
            info = snapshot.basic_info
            cells = snapshot.cell_data
            
            # Build row
            row = {
                'timestamp': snapshot.timestamp.strftime('%Y-%m-%d %H:%M:%S'),
                'total_voltage': f'{info.total_voltage:.3f}',
                'current': f'{info.current:.3f}',
                'remaining_capacity': f'{info.remaining_capacity:.3f}',
                'nominal_capacity': f'{info.nominal_capacity:.3f}',
                'soc': info.soc,
                'cycle_count': info.cycle_count,
                'charge_mosfet': 1 if info.charge_mosfet_on else 0,
                'discharge_mosfet': 1 if info.discharge_mosfet_on else 0
            }
            
            # Add cell voltages
            for i, voltage in enumerate(cells.voltages):
                row[f'cell_{i+1}_voltage'] = f'{voltage:.3f}'
            
            # Add temperatures
            for i in range(4):
                if i < len(info.temperatures):
                    row[f'temp_{i+1}'] = f'{info.temperatures[i]:.1f}'
                else:
                    row[f'temp_{i+1}'] = ''
            
            # Add protection flags
            row['protection_active'] = 1 if info.protection_status.has_any_protection() else 0
            row['software_lock'] = 1 if info.protection_status.software_lock else 0
            row['cell_overvoltage'] = 1 if info.protection_status.cell_overvoltage else 0
            row['cell_undervoltage'] = 1 if info.protection_status.cell_undervoltage else 0
            
            self.csv_writer.writerow(row)
            self.file_handle.flush()
            
            return True
            
        except Exception as e:
            print(f"Error logging snapshot: {e}")
            return False
    
    def start_logging(self, monitor: 'BMSMonitor') -> bool:
        """
        Start automatic logging in background thread
        
        Args:
            monitor: BMSMonitor instance
            
        Returns:
            True if started successfully
        """
        if not self.enabled:
            return False
        
        if self.logging_thread and self.logging_thread.is_alive():
            print("Logging already active")
            return False
        
        self.stop_logging_event.clear()
        
        def logging_loop():
            while not self.stop_logging_event.is_set():
                snapshot = monitor.collect_snapshot()
                if snapshot:
                    self.log_snapshot(snapshot)
                
                # Wait for interval or stop event
                self.stop_logging_event.wait(self.interval)
        
        self.logging_thread = threading.Thread(target=logging_loop, daemon=True)
        self.logging_thread.start()
        
        return True
    
    def stop_logging(self) -> None:
        """Stop automatic logging"""
        if self.logging_thread and self.logging_thread.is_alive():
            self.stop_logging_event.set()
            self.logging_thread.join(timeout=5)
        
        self._close_log_file()


# ============================================================================
# CLASS 6: BMSInterface - Interactive menu system
# ============================================================================

class BMSInterface:
    """
    Interactive user interface for BMS
    
    Provides menu system and user interaction.
    """
    
    def __init__(self, communication: BMSCommunication, 
                 controller: BMSController,
                 monitor: BMSMonitor,
                 logger: DataLogger,
                 config: Dict[str, Any]):
        """
        Initialize BMS interface
        
        Args:
            communication: BMSCommunication instance
            controller: BMSController instance
            monitor: BMSMonitor instance
            logger: DataLogger instance
            config: Configuration dictionary
        """
        self.comm = communication
        self.controller = controller
        self.monitor = monitor
        self.logger = logger
        self.config = config
        self.use_colors = config.get('display', {}).get('use_colors', True)
    
    def print_colored(self, text: str, color: str = Colors.WHITE) -> None:
        """Print colored text"""
        if self.use_colors:
            print(f"{color}{text}{Colors.RESET}")
        else:
            print(text)
    
    def print_header(self, text: str) -> None:
        """Print section header"""
        self.print_colored("\n" + "=" * 80, Colors.CYAN)
        self.print_colored(f"  {text}", Colors.CYAN + Colors.BOLD)
        self.print_colored("=" * 80, Colors.CYAN)
    
    def wait_for_key(self) -> None:
        """Wait for user to press key"""
        input(f"\n{Colors.DIM}Press Enter to continue...{Colors.RESET}")
    
    def display_main_menu(self) -> None:
        """Display main menu"""
        self.print_header("JBD BMS MONITORING & CONTROL SYSTEM")
        
        print("\nMain Menu:")
        print("  1. Real-time Monitoring Dashboard")
        print("  2. MOS Control Operations")
        print("  3. Cell Voltage Analysis")
        print("  4. System Health Report")
        print("  5. Data Logging Control")
        print("  6. Connection Management")
        print("  7. Display Configuration")
        print("  8. Help")
        print("  9. Exit")
        print()
    
    def handle_monitoring_mode(self) -> None:
        """Real-time monitoring dashboard"""
        self.print_header("REAL-TIME MONITORING DASHBOARD")
        print(f"\n{Colors.YELLOW}Press Ctrl+C to return to menu{Colors.RESET}\n")
        
        refresh_rate = self.config.get('monitoring', {}).get('refresh_rate', 1.0)
        
        try:
            while True:
                snapshot = self.monitor.collect_snapshot()
                
                if snapshot and snapshot.is_complete():
                    # Clear screen for clean display
                    os.system('clear' if os.name == 'posix' else 'cls')
                    self.monitor.display_dashboard(snapshot, self.use_colors)
                    
                    # Check for alerts
                    alerts = self.monitor.check_alerts(snapshot)
                    if alerts:
                        self.print_colored("\n🔔 ALERTS:", Colors.RED + Colors.BOLD)
                        for alert in alerts:
                            self.print_colored(f"  • {alert}", Colors.RED)
                else:
                    print(f"{Colors.RED}Failed to collect data{Colors.RESET}")
                
                time.sleep(refresh_rate)
                
        except KeyboardInterrupt:
            print(f"\n{Colors.YELLOW}Returning to menu...{Colors.RESET}")
    
    def handle_control_menu(self) -> None:
        """MOS control operations menu"""
        while True:
            self.print_header("MOS CONTROL OPERATIONS")
            
            # Show current state
            state = self.comm.read_basic_info()
            if state:
                print(f"\n{Colors.CYAN}Current FET Status:{Colors.RESET}")
                charge_status = f"{Colors.GREEN}ON{Colors.RESET}" if state.charge_mosfet_on else f"{Colors.RED}OFF{Colors.RESET}"
                discharge_status = f"{Colors.GREEN}ON{Colors.RESET}" if state.discharge_mosfet_on else f"{Colors.RED}OFF{Colors.RESET}"
                print(f"  Charge MOSFET:    {charge_status}")
                print(f"  Discharge MOSFET: {discharge_status}")
                
                if state.protection_status.software_lock:
                    self.print_colored("\n⚠ Software Lock is ACTIVE", Colors.YELLOW)
            
            print("\nControl Options:")
            print("  1. Release Software Lock")
            print("  2. Disable Charging MOSFET")
            print("  3. Disable Discharging MOSFET")
            print("  4. Disable Both MOSFETs (Emergency Shutdown)")
            print("  5. Refresh Status")
            print("  6. Back to Main Menu")
            
            choice = input("\nSelect option: ").strip()
            
            if choice == '1':
                self.controller.release_software_lock()
                self.wait_for_key()
            elif choice == '2':
                self.controller.disable_charging()
                self.wait_for_key()
            elif choice == '3':
                self.controller.disable_discharging()
                self.wait_for_key()
            elif choice == '4':
                self.controller.disable_both()
                self.wait_for_key()
            elif choice == '5':
                continue
            elif choice == '6':
                break
            else:
                self.print_colored("Invalid option", Colors.RED)
    
    def display_cell_analysis(self) -> None:
        """Display detailed cell voltage analysis"""
        self.print_header("CELL VOLTAGE ANALYSIS")
        
        num_cells = self.config.get('battery', {}).get('cell_count', 24)
        cell_data = self.comm.read_cell_voltages(num_cells)
        
        if not cell_data:
            self.print_colored("Failed to read cell voltages", Colors.RED)
            self.wait_for_key()
            return
        
        print(f"\n{Colors.CYAN}Cell Voltage Statistics:{Colors.RESET}")
        print(f"  Minimum:  {cell_data.min_voltage:.3f}V (Cell {cell_data.min_cell_index + 1})")
        print(f"  Maximum:  {cell_data.max_voltage:.3f}V (Cell {cell_data.max_cell_index + 1})")
        print(f"  Average:  {cell_data.average_voltage:.3f}V")
        print(f"  Delta:    {cell_data.voltage_delta:.3f}V")
        print(f"  Imbalance: {cell_data.get_imbalance_percent():.2f}%")
        
        print(f"\n{Colors.CYAN}Individual Cell Voltages:{Colors.RESET}")
        
        for i, voltage in enumerate(cell_data.voltages):
            cell_num = i + 1
            
            # Color code based on voltage relative to average
            if voltage < cell_data.average_voltage - 0.05:
                color = Colors.YELLOW
            elif voltage > cell_data.average_voltage + 0.05:
                color = Colors.CYAN
            else:
                color = Colors.GREEN
            
            # Mark min/max
            marker = ""
            if i == cell_data.min_cell_index:
                marker = f" {Colors.RED}[MIN]{Colors.RESET}"
            elif i == cell_data.max_cell_index:
                marker = f" {Colors.GREEN}[MAX]{Colors.RESET}"
            
            print(f"  Cell {cell_num:2d}: {color}{voltage:.3f}V{Colors.RESET}{marker}")
        
        # Health analysis
        health = self.monitor.analyze_cell_health(cell_data)
        
        print(f"\n{Colors.CYAN}Cell Health Assessment:{Colors.RESET}")
        health_color = Colors.GREEN if health['overall_status'] == 'good' else Colors.YELLOW if health['overall_status'] == 'warning' else Colors.RED
        self.print_colored(f"  Status: {health['overall_status'].upper()}", health_color)
        
        if health['warnings']:
            print(f"\n{Colors.YELLOW}Warnings:{Colors.RESET}")
            for warning in health['warnings']:
                print(f"  ⚠ {warning}")
        
        if health['issues']:
            print(f"\n{Colors.RED}Issues:{Colors.RESET}")
            for issue in health['issues']:
                print(f"  ✗ {issue}")
        
        self.wait_for_key()
    
    def display_health_report(self) -> None:
        """Display comprehensive system health report"""
        self.print_header("SYSTEM HEALTH REPORT")
        
        snapshot = self.monitor.collect_snapshot()
        
        if not snapshot or not snapshot.is_complete():
            self.print_colored("Failed to collect system data", Colors.RED)
            self.wait_for_key()
            return
        
        info = snapshot.basic_info
        cells = snapshot.cell_data
        
        # Overall system info
        print(f"\n{Colors.CYAN}System Information:{Colors.RESET}")
        print(f"  Hardware Version: {snapshot.hardware_version or 'N/A'}")
        print(f"  Software Version: {info.software_version}")
        print(f"  Number of Cells:  {info.num_cells}")
        print(f"  Cycle Count:      {info.cycle_count}")
        
        # Pack health
        print(f"\n{Colors.CYAN}Pack Health:{Colors.RESET}")
        print(f"  Total Voltage:     {info.total_voltage:.2f}V")
        print(f"  State of Charge:   {info.soc}%")
        print(f"  Remaining Cap:     {info.remaining_capacity:.2f}Ah")
        print(f"  Nominal Cap:       {info.nominal_capacity:.2f}Ah")
        
        if info.nominal_capacity > 0:
            capacity_health = (info.remaining_capacity / info.nominal_capacity) * 100
            cap_color = Colors.GREEN if capacity_health > 80 else Colors.YELLOW if capacity_health > 60 else Colors.RED
            self.print_colored(f"  Capacity Health:   {capacity_health:.1f}%", cap_color)
        
        # Cell health
        print(f"\n{Colors.CYAN}Cell Health:{Colors.RESET}")
        print(f"  Voltage Delta:     {cells.voltage_delta:.3f}V")
        print(f"  Imbalance:         {cells.get_imbalance_percent():.2f}%")
        
        cell_health = self.monitor.analyze_cell_health(cells)
        health_color = Colors.GREEN if cell_health['overall_status'] == 'good' else Colors.YELLOW if cell_health['overall_status'] == 'warning' else Colors.RED
        self.print_colored(f"  Cell Status:       {cell_health['overall_status'].upper()}", health_color)
        
        # Temperature health
        if info.temperatures:
            print(f"\n{Colors.CYAN}Temperature Health:{Colors.RESET}")
            min_temp = min(info.temperatures)
            max_temp = max(info.temperatures)
            avg_temp = sum(info.temperatures) / len(info.temperatures)
            
            print(f"  Minimum:           {min_temp:.1f}°C")
            print(f"  Maximum:           {max_temp:.1f}°C")
            print(f"  Average:           {avg_temp:.1f}°C")
            print(f"  Delta:             {max_temp - min_temp:.1f}°C")
        
        # Protection status
        print(f"\n{Colors.CYAN}Protection Status:{Colors.RESET}")
        if info.protection_status.has_any_protection():
            self.print_colored("  ⚠ ACTIVE PROTECTIONS:", Colors.RED)
            for protection in info.protection_status.get_active_protections():
                self.print_colored(f"    • {protection}", Colors.RED)
        else:
            self.print_colored("  ✓ No active protections", Colors.GREEN)
        
        # Overall health
        pack_health = self.monitor.analyze_pack_health(info)
        
        overall_status = 'critical' if (cell_health['overall_status'] == 'critical' or 
                                       pack_health['overall_status'] == 'critical') else \
                        'warning' if (cell_health['overall_status'] == 'warning' or 
                                     pack_health['overall_status'] == 'warning') else 'good'
        
        print(f"\n{Colors.CYAN}Overall System Health:{Colors.RESET}")
        overall_color = Colors.GREEN if overall_status == 'good' else Colors.YELLOW if overall_status == 'warning' else Colors.RED
        self.print_colored(f"  {overall_status.upper()}", overall_color + Colors.BOLD)
        
        # Statistics
        if len(self.monitor.history) > 10:
            stats = self.monitor.get_statistics(60)
            
            print(f"\n{Colors.CYAN}Recent Statistics (last 60 samples):{Colors.RESET}")
            
            if 'voltage' in stats:
                print(f"  Voltage Range:     {stats['voltage']['min']:.2f}V - {stats['voltage']['max']:.2f}V")
            
            if 'current' in stats:
                print(f"  Current Range:     {stats['current']['min']:.2f}A - {stats['current']['max']:.2f}A")
            
            if 'temperature' in stats:
                print(f"  Temp Range:        {stats['temperature']['min']:.1f}°C - {stats['temperature']['max']:.1f}°C")
        
        self.wait_for_key()
    
    def handle_logging_menu(self) -> None:
        """Data logging control menu"""
        while True:
            self.print_header("DATA LOGGING CONTROL")
            
            logging_active = self.logger.logging_thread and self.logger.logging_thread.is_alive()
            
            print(f"\n{Colors.CYAN}Logging Status:{Colors.RESET}")
            status = f"{Colors.GREEN}ACTIVE{Colors.RESET}" if logging_active else f"{Colors.YELLOW}STOPPED{Colors.RESET}"
            print(f"  Status:       {status}")
            print(f"  Directory:    {self.logger.directory}")
            print(f"  Interval:     {self.logger.interval}s")
            print(f"  Current File: {self.logger.current_file or 'N/A'}")
            
            print("\nOptions:")
            print("  1. Start Logging" if not logging_active else "  1. Stop Logging")
            print("  2. View Log Directory")
            print("  3. Change Logging Interval")
            print("  4. Back to Main Menu")
            
            choice = input("\nSelect option: ").strip()
            
            if choice == '1':
                if logging_active:
                    self.logger.stop_logging()
                    self.print_colored("Logging stopped", Colors.YELLOW)
                else:
                    if self.logger.start_logging(self.monitor):
                        self.print_colored("Logging started", Colors.GREEN)
                    else:
                        self.print_colored("Failed to start logging", Colors.RED)
                self.wait_for_key()
            elif choice == '2':
                print(f"\nLog files in {self.logger.directory}:")
                if os.path.exists(self.logger.directory):
                    files = sorted([f for f in os.listdir(self.logger.directory) if f.endswith('.csv')])
                    if files:
                        for f in files:
                            path = os.path.join(self.logger.directory, f)
                            size = os.path.getsize(path) / 1024
                            print(f"  {f} ({size:.1f} KB)")
                    else:
                        print("  No log files found")
                else:
                    print("  Directory does not exist")
                self.wait_for_key()
            elif choice == '3':
                try:
                    interval = float(input("Enter new interval (seconds): ").strip())
                    if interval > 0:
                        self.logger.interval = interval
                        self.print_colored(f"Interval set to {interval}s", Colors.GREEN)
                    else:
                        self.print_colored("Invalid interval", Colors.RED)
                except ValueError:
                    self.print_colored("Invalid input", Colors.RED)
                self.wait_for_key()
            elif choice == '4':
                break
            else:
                self.print_colored("Invalid option", Colors.RED)
    
    def handle_connection_menu(self) -> None:
        """Connection management menu"""
        while True:
            self.print_header("CONNECTION MANAGEMENT")
            
            connected = self.comm.is_connected()
            
            print(f"\n{Colors.CYAN}Connection Status:{Colors.RESET}")
            status = f"{Colors.GREEN}CONNECTED{Colors.RESET}" if connected else f"{Colors.RED}DISCONNECTED{Colors.RESET}"
            print(f"  Status:    {status}")
            print(f"  Port:      {self.comm.port or 'Not set'}")
            print(f"  Baudrate:  {self.comm.baudrate}")
            
            print("\nOptions:")
            print("  1. Connect" if not connected else "  1. Disconnect")
            print("  2. Change Port")
            print("  3. Auto-detect Port")
            print("  4. Test Connection")
            print("  5. Back to Main Menu")
            
            choice = input("\nSelect option: ").strip()
            
            if choice == '1':
                if connected:
                    self.comm.disconnect()
                    self.print_colored("Disconnected", Colors.YELLOW)
                else:
                    if self.comm.connect():
                        self.print_colored(f"Connected to {self.comm.port}", Colors.GREEN)
                    else:
                        self.print_colored("Connection failed", Colors.RED)
                self.wait_for_key()
            elif choice == '2':
                port = input("Enter serial port path: ").strip()
                if port:
                    if connected:
                        self.comm.disconnect()
                    self.comm.port = port
                    if self.comm.connect():
                        self.print_colored(f"Connected to {port}", Colors.GREEN)
                    else:
                        self.print_colored("Connection failed", Colors.RED)
                self.wait_for_key()
            elif choice == '3':
                detected = self.comm.auto_detect_port()
                if detected:
                    self.print_colored(f"Detected port: {detected}", Colors.GREEN)
                    if input("Connect to this port? (y/n): ").lower() == 'y':
                        if connected:
                            self.comm.disconnect()
                        if self.comm.connect(detected):
                            self.print_colored(f"Connected to {detected}", Colors.GREEN)
                        else:
                            self.print_colored("Connection failed", Colors.RED)
                else:
                    self.print_colored("No ports detected", Colors.RED)
                self.wait_for_key()
            elif choice == '4':
                if not connected:
                    self.print_colored("Not connected", Colors.RED)
                else:
                    print("Testing connection...")
                    info = self.comm.read_basic_info()
                    if info:
                        self.print_colored("✓ Connection OK", Colors.GREEN)
                        print(f"  Voltage: {info.total_voltage:.2f}V")
                        print(f"  SOC: {info.soc}%")
                    else:
                        self.print_colored("✗ Connection test failed", Colors.RED)
                self.wait_for_key()
            elif choice == '5':
                break
            else:
                self.print_colored("Invalid option", Colors.RED)
    
    def display_help(self) -> None:
        """Display help information"""
        self.print_header("HELP & DOCUMENTATION")
        
        print("""
JBD BMS Monitoring & Control System

This application provides complete monitoring and control capabilities for
JBD BMS (Battery Management System) via UART/Serial interface.

KEY FEATURES:

1. Real-time Monitoring
   - Live dashboard with all BMS parameters
   - Cell voltages, pack voltage, current, SOC
   - Temperature monitoring
   - Protection status
   - Health analysis and alerts

2. MOS Control
   - Independent control of charging and discharging MOSFETs
   - Safety confirmations required
   - Status verification after commands
   - Emergency shutdown capability

3. Data Logging
   - Automatic CSV logging with timestamps
   - Configurable intervals
   - Automatic file rotation
   - All parameters logged

4. Health Analysis
   - Cell voltage imbalance detection
   - Temperature monitoring and alerts
   - Capacity fade tracking
   - Overall system health scoring

SAFETY NOTES:

⚠ Always verify current FET status before making changes
⚠ Disabling charging prevents all charging current
⚠ Disabling discharging prevents all discharge/output current
⚠ Use emergency shutdown only when necessary

CONFIGURATION:

Edit bms_config.yaml to customize:
- Serial port and communication settings
- Battery specifications (cell count, voltages)
- Alert thresholds
- Logging settings
- Display preferences

QUICK START:

1. Connect BMS via USB-to-Serial adapter
2. Configure serial port in bms_config.yaml (or use auto-detect)
3. Run: python bms_monitor.py
4. Select "Connection Management" to connect
5. Use "Real-time Monitoring" to view live data

For more information, see README.md
""")
        
        self.wait_for_key()
    
    def run(self) -> None:
        """Run main interface loop"""
        while True:
            try:
                self.display_main_menu()
                choice = input("Select option: ").strip()
                
                if choice == '1':
                    if not self.comm.is_connected():
                        self.print_colored("Not connected to BMS", Colors.RED)
                        self.wait_for_key()
                    else:
                        self.handle_monitoring_mode()
                elif choice == '2':
                    if not self.comm.is_connected():
                        self.print_colored("Not connected to BMS", Colors.RED)
                        self.wait_for_key()
                    else:
                        self.handle_control_menu()
                elif choice == '3':
                    if not self.comm.is_connected():
                        self.print_colored("Not connected to BMS", Colors.RED)
                        self.wait_for_key()
                    else:
                        self.display_cell_analysis()
                elif choice == '4':
                    if not self.comm.is_connected():
                        self.print_colored("Not connected to BMS", Colors.RED)
                        self.wait_for_key()
                    else:
                        self.display_health_report()
                elif choice == '5':
                    self.handle_logging_menu()
                elif choice == '6':
                    self.handle_connection_menu()
                elif choice == '7':
                    print("\nDisplay Configuration:")
                    print(f"  Colors: {'Enabled' if self.use_colors else 'Disabled'}")
                    if input("Toggle colors? (y/n): ").lower() == 'y':
                        self.use_colors = not self.use_colors
                        self.config['display']['use_colors'] = self.use_colors
                        self.print_colored("Setting updated", Colors.GREEN)
                    self.wait_for_key()
                elif choice == '8':
                    self.display_help()
                elif choice == '9':
                    print("\nExiting...")
                    break
                else:
                    self.print_colored("Invalid option", Colors.RED)
                    time.sleep(1)
                    
            except KeyboardInterrupt:
                print(f"\n{Colors.YELLOW}Use option 9 to exit{Colors.RESET}")
                time.sleep(1)


# ============================================================================
# CLASS 7: BMSApplication - Main application orchestrator
# ============================================================================

class BMSApplication:
    """
    Main BMS application orchestrator
    
    Handles initialization, configuration, and graceful shutdown.
    """
    
    def __init__(self, config_path: str = "bms_config.yaml"):
        """
        Initialize BMS application
        
        Args:
            config_path: Path to configuration file
        """
        self.config_path = config_path
        self.config: Dict[str, Any] = {}
        self.communication: Optional[BMSCommunication] = None
        self.controller: Optional[BMSController] = None
        self.monitor: Optional[BMSMonitor] = None
        self.logger: Optional[DataLogger] = None
        self.interface: Optional[BMSInterface] = None
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print(f"\n{Colors.YELLOW}Shutdown signal received...{Colors.RESET}")
        self.shutdown()
        sys.exit(0)
    
    def load_config(self) -> bool:
        """
        Load configuration from YAML file
        
        Returns:
            True if loaded successfully
        """
        try:
            if not os.path.exists(self.config_path):
                print(f"{Colors.YELLOW}Warning: Config file not found, using defaults{Colors.RESET}")
                self.config = self._get_default_config()
                return True
            
            with open(self.config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            
            print(f"{Colors.GREEN}✓ Configuration loaded from {self.config_path}{Colors.RESET}")
            return True
            
        except Exception as e:
            print(f"{Colors.RED}Error loading config: {e}{Colors.RESET}")
            print(f"{Colors.YELLOW}Using default configuration{Colors.RESET}")
            self.config = self._get_default_config()
            return True
    
    def _get_default_config(self) -> Dict[str, Any]:
        """Get default configuration"""
        return {
            'serial': {
                'port': '',
                'baudrate': 9600,
                'timeout': 2.0,
                'retry_attempts': 3,
                'retry_delay': 0.5
            },
            'battery': {
                'cell_count': 24,
                'nominal_voltage': 3.7,
                'max_cell_voltage': 4.2,
                'min_cell_voltage': 3.0,
                'max_pack_voltage': 100.8,
                'min_pack_voltage': 72.0,
                'nominal_capacity': 100
            },
            'thresholds': {
                'cell_voltage_delta_warning': 0.05,
                'cell_voltage_delta_critical': 0.1,
                'temperature_warning': 50,
                'temperature_critical': 60,
                'current_warning': 90,
                'current_critical': 100
            },
            'logging': {
                'enabled': True,
                'interval': 5,
                'directory': './bms_logs',
                'max_file_size_mb': 10,
                'rotation': 'daily'
            },
            'monitoring': {
                'refresh_rate': 1.0,
                'history_buffer_size': 300,
                'alert_cooldown': 30
            },
            'display': {
                'use_colors': True,
                'compact_mode': False
            },
            'safety': {
                'require_confirmation': True,
                'verification_delay': 0.5
            }
        }
    
    def initialize(self) -> bool:
        """
        Initialize all components
        
        Returns:
            True if initialized successfully
        """
        print(f"\n{Colors.CYAN}Initializing JBD BMS Monitor...{Colors.RESET}\n")
        
        # Load configuration
        if not self.load_config():
            return False
        
        # Initialize communication
        serial_config = self.config.get('serial', {})
        self.communication = BMSCommunication(
            port=serial_config.get('port', ''),
            baudrate=serial_config.get('baudrate', 9600),
            timeout=serial_config.get('timeout', 2.0),
            retry_attempts=serial_config.get('retry_attempts', 3),
            retry_delay=serial_config.get('retry_delay', 0.5)
        )
        
        # Initialize controller
        safety_config = self.config.get('safety', {})
        self.controller = BMSController(
            self.communication,
            require_confirmation=safety_config.get('require_confirmation', True),
            verification_delay=safety_config.get('verification_delay', 0.5)
        )
        
        # Initialize monitor
        self.monitor = BMSMonitor(self.communication, self.config)
        
        # Initialize logger
        self.logger = DataLogger(self.config)
        
        # Initialize interface
        self.interface = BMSInterface(
            self.communication,
            self.controller,
            self.monitor,
            self.logger,
            self.config
        )
        
        print(f"{Colors.GREEN}✓ All components initialized{Colors.RESET}\n")
        return True
    
    def run(self) -> None:
        """Run main application"""
        if not self.interface:
            print(f"{Colors.RED}Error: Application not initialized{Colors.RESET}")
            return
        
        # Display welcome banner
        print(f"\n{Colors.CYAN}{Colors.BOLD}")
        print("=" * 80)
        print("  JBD BMS MONITORING & CONTROL SYSTEM")
        print("  Production-Ready Python Application")
        print("  24S 100A Configuration")
        print("=" * 80)
        print(f"{Colors.RESET}")
        
        # Run interface
        try:
            self.interface.run()
        except Exception as e:
            print(f"{Colors.RED}Unexpected error: {e}{Colors.RESET}")
        finally:
            self.shutdown()
    
    def shutdown(self) -> None:
        """Graceful shutdown"""
        print(f"\n{Colors.YELLOW}Shutting down...{Colors.RESET}")
        
        # Stop logging
        if self.logger:
            self.logger.stop_logging()
            print("✓ Logging stopped")
        
        # Disconnect
        if self.communication:
            self.communication.disconnect()
            print("✓ Disconnected from BMS")
        
        print(f"{Colors.GREEN}Goodbye!{Colors.RESET}\n")


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='JBD BMS Monitoring & Control System',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument(
        '--config',
        type=str,
        default='bms_config.yaml',
        help='Path to configuration file (default: bms_config.yaml)'
    )
    
    parser.add_argument(
        '--port',
        type=str,
        help='Serial port (overrides config file)'
    )
    
    parser.add_argument(
        '--status',
        action='store_true',
        help='Quick status check and exit'
    )
    
    args = parser.parse_args()
    
    # Create and initialize application
    app = BMSApplication(args.config)
    
    if not app.initialize():
        print(f"{Colors.RED}Initialization failed{Colors.RESET}")
        sys.exit(1)
    
    # Override port if specified
    if args.port:
        app.communication.port = args.port
    
    # Quick status mode
    if args.status:
        if not app.communication.connect():
            print(f"{Colors.RED}Failed to connect{Colors.RESET}")
            sys.exit(1)
        
        snapshot = app.monitor.collect_snapshot()
        if snapshot and snapshot.is_complete():
            app.monitor.display_dashboard(snapshot)
        else:
            print(f"{Colors.RED}Failed to read BMS data{Colors.RESET}")
            sys.exit(1)
        
        app.shutdown()
        sys.exit(0)
    
    # Run main application
    app.run()


if __name__ == '__main__':
    main()
