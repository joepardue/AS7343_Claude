# SPDX-FileCopyrightText: 2025 Your Name for Adafruit Industries
# SPDX-License-Identifier: MIT

"""
`as7343`
================================================================================

CircuitPython driver for the AS7343 14-channel spectral sensor

* Author(s): Your Name

Implementation Notes
--------------------

**Hardware:**

* AS7343 14-Channel Multi-Purpose Spectral Sensor
* https://ams.com/as7343

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads
* Adafruit's Bus Device library: 
  https://github.com/adafruit/Adafruit_CircuitPython_BusDevice

**Notes:**

* This driver supports the AS7343's auto-SMUX feature for simplified channel reading
* Automatic gain control helps prevent saturation and optimize sensitivity
* Integration time can be configured from microseconds to seconds
* All 12 spectral channels plus 4 clear channels are accessible
"""

import time
from adafruit_bus_device import i2c_device
from micropython import const
from typing import List, Dict, Tuple, Optional, Union

try:
    from typing import Literal
except ImportError:
    from typing_extensions import Literal

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_AS7343.git"

# Register addresses
_AS7343_WHOAMI = const(0x5A)  # Device ID register
_AS7343_DEVICE_ID = const(0x81)  # Expected ID value
_AS7343_ENABLE = const(0x80)
_AS7343_ATIME = const(0x81)
_AS7343_ASTEP_L = const(0xD4)
_AS7343_ASTEP_H = const(0xD5)
_AS7343_CFG0 = const(0xBF)  # Contains REG_BANK bit
_AS7343_CFG1 = const(0xAA)
_AS7343_CFG6 = const(0xF5)  # SMUX command register
_AS7343_CFG20 = const(0xD6)  # Auto-SMUX configuration register
_AS7343_STATUS = const(0x93)
_AS7343_ASTATUS = const(0x94)
_AS7343_STATUS2 = const(0x90)
_AS7343_CH0_DATA_L = const(0x95)
_AS7343_CH0_DATA_H = const(0x96)

# Enable register bits
_AS7343_PON = const(0x01)
_AS7343_SP_EN = const(0x02)
_AS7343_SMUXEN = const(0x10)

# CFG0 register bits
_AS7343_REG_BANK = const(0x10)  # Bit 4

# STATUS2 bits
_AS7343_AVALID = const(0x40)  # Bit 6

try:
    from enum import IntEnum
except ImportError:
    # For CircuitPython compatibility
    class IntEnum:
        pass

class Gain(IntEnum):
    """Valid gain values for the AS7343 sensor."""
    X0_5 = 0
    X1 = 1
    X2 = 2
    X4 = 3
    X8 = 4
    X16 = 5
    X32 = 6
    X64 = 7
    X128 = 8
    X256 = 9
    X512 = 10
    X1024 = 11
    X2048 = 12

class AutoSMUXMode(IntEnum):
    """Auto-SMUX mode options."""
    SIX_CHANNEL = 0
    TWELVE_CHANNEL = 2
    EIGHTEEN_CHANNEL = 3

# Map gain enum values to actual multiplier values
_GAIN_ENUM_TO_VALUE = {
    Gain.X0_5: 0.5,
    Gain.X1: 1,
    Gain.X2: 2,
    Gain.X4: 4,
    Gain.X8: 8,
    Gain.X16: 16,
    Gain.X32: 32,
    Gain.X64: 64,
    Gain.X128: 128,
    Gain.X256: 256,
    Gain.X512: 512,
    Gain.X1024: 1024,
    Gain.X2048: 2048
}

# Reverse mapping for setting gain
_GAIN_VALUE_TO_ENUM = {v: k for k, v in _GAIN_ENUM_TO_VALUE.items()}

class AS7343:
    """
    Driver for the AS7343 14-channel spectral sensor.
    
    The AS7343 is a 14-channel spectral sensor covering UV, visible and near-IR
    wavelengths from 340nm to 1030nm. It features automatic gain control,
    configurable integration time, and an innovative auto-SMUX feature that
    simplifies reading multiple channels.
    
    :param i2c_bus: The I2C bus the AS7343 is connected to
    :param address: The I2C device address. Defaults to 0x39
    
    **Quick Start: Basic Usage**
    
        .. code-block:: python
        
            import board
            import as7343
            
            i2c = board.I2C()  # uses board.SCL and board.SDA
            sensor = as7343.AS7343(i2c)
            
            # Set gain and integration time
            sensor.set_gain(as7343.Gain.X16)
            sensor.set_integration_time_ms(100)
            
            # Read all spectral channels
            channels = sensor.read_all_channels_auto()
            print(f"Channel values: {channels}")
            
            # Get calibrated readings
            calibrated = sensor.read_all_channels_calibrated()
            print(f"Calibrated values: {calibrated}")
            
            # Auto-adjust gain if needed
            gain_changed, new_gain = sensor.auto_adjust_gain(channels)
            if gain_changed:
                print(f"Gain adjusted to {new_gain}x")
            
            # Access individual channels
            blue_450nm = sensor.channel_450nm_fz
            green_555nm = sensor.channel_555nm_fy
            
            # Get all channels as a dictionary
            all_channels = sensor.all_channels
            print(f"F2 (425nm): {all_channels['F2_425nm']}")
    """
    
    def __init__(self, i2c_bus, address: int = 0x39) -> None:
        """
        Initialize the AS7343 sensor.
        
        Performs the following startup sequence:
        1. Verifies the device ID by switching to register bank 1
        2. Powers on the device and resets to default state
        3. Sets default integration time (~50ms)
        4. Sets default gain (256x)
        5. Initializes SMUX with ROM command 0
        
        :param i2c_bus: The I2C bus interface to use for communication
        :param address: The I2C address of the device (default 0x39)
        :raises RuntimeError: If the device ID doesn't match expected value
        """
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        self._buffer = bytearray(2)
        
        # Need to set correct register bank to read ID
        self._write_register(_AS7343_CFG0, _AS7343_REG_BANK)  # Set REG_BANK = 1
        time.sleep(0.01)
        
        # Check device ID
        device_id = self._read_register(_AS7343_WHOAMI)
        if device_id != _AS7343_DEVICE_ID:
            raise RuntimeError(f"Failed to find AS7343! Expected ID: {_AS7343_DEVICE_ID:#x}, Found: {device_id:#x}")
        
        # Set register bank back to 0 for normal operation
        self._write_register(_AS7343_CFG0, 0x00)  # Set REG_BANK = 0
        time.sleep(0.01)
        
        # Initialize device
        self._write_register(_AS7343_ENABLE, 0x00)  # Reset
        time.sleep(0.01)
        self._write_register(_AS7343_ENABLE, _AS7343_PON)  # Power on
        time.sleep(0.01)
        
        # Set default integration time and gain
        self._write_register(_AS7343_ATIME, 29)  # ~50ms
        self._write_register(_AS7343_ASTEP_L, 0x63)  # 599 low byte
        self._write_register(_AS7343_ASTEP_H, 0x02)  # 599 high byte
        self._write_register(_AS7343_CFG1, 9)  # 256x gain
        
        # Initialize SMUX with command 0 (ROM initialization)
        self._smux_command(0)
    
    def _read_register(self, address: int, length: int = 1) -> Union[int, bytearray]:
        """
        Read a register or sequence of registers.
        
        :param address: Register address to read from
        :param length: Number of bytes to read, defaults to 1
        :return: Register value(s) as int for single byte or bytearray for multiple
        """
        with self.i2c_device as i2c:
            if length == 1:
                self._buffer[0] = address
                i2c.write_then_readinto(self._buffer, self._buffer, out_end=1, in_end=1)
                return self._buffer[0]
            else:
                result = bytearray(length)
                i2c.write_then_readinto(bytes([address]), result)
                return result
    
    def _write_register(self, address: int, value: int) -> None:
        """
        Write a value to a register.
        
        :param address: Register address to write to
        :param value: Value to write (max 8 bits)
        """
        with self.i2c_device as i2c:
            self._buffer[0] = address
            self._buffer[1] = value & 0xFF
            i2c.write(self._buffer)
    
    def _smux_command(self, command: int) -> None:
        """
        Execute SMUX command.
        
        :param command: SMUX command value (0-7)
        """
        # Write command to CFG6
        self._write_register(_AS7343_CFG6, command << 3)
        
        # Enable SMUX
        enable = self._read_register(_AS7343_ENABLE)
        enable |= _AS7343_SMUXEN
        self._write_register(_AS7343_ENABLE, enable)
        
        # Wait for SMUX operation to complete
        while self._read_register(_AS7343_ENABLE) & _AS7343_SMUXEN:
            time.sleep(0.001)
    
    def start_measurement(self) -> None:
        """Start a spectral measurement."""
        enable = self._read_register(_AS7343_ENABLE)
        enable |= _AS7343_SP_EN
        self._write_register(_AS7343_ENABLE, enable)
    
    def measurement_complete(self) -> bool:
        """
        Check if measurement is complete.
        
        :return: True if measurement is complete, False otherwise
        """
        # Check STATUS2 register for AVALID bit
        status2 = self._read_register(_AS7343_STATUS2)
        return bool(status2 & _AS7343_AVALID)
    
    def read_all_channels(self) -> List[int]:
        """
        Read all 6 basic channels.
        
        This method performs a single measurement cycle using the current SMUX
        configuration and returns raw channel data.
        
        :return: List of 6 channel values as 16-bit integers
        """
        self.start_measurement()
        
        count = 0
        while not self.measurement_complete():
            time.sleep(0.01)
            count += 1
            if count > 100:  # Timeout after 1 second
                print("Timeout waiting for measurement!")
                break
        
        channels = []
        for i in range(6):
            addr = _AS7343_CH0_DATA_L + (i * 2)
            data = self._read_register(addr, 2)
            value = data[0] | (data[1] << 8)
            channels.append(value)
        return channels
    
    def enable_auto_smux(self, mode: Union[AutoSMUXMode, int] = AutoSMUXMode.TWELVE_CHANNEL) -> None:
        """
        Enable auto-SMUX mode for automatic channel switching.
        
        :param mode: Auto-SMUX mode as AutoSMUXMode enum or int:
                     - AutoSMUXMode.SIX_CHANNEL (0): 6 channel (single cycle)
                     - AutoSMUXMode.TWELVE_CHANNEL (2): 12 channel (2 cycles)  
                     - AutoSMUXMode.EIGHTEEN_CHANNEL (3): 18 channel (3 cycles with Flicker)
        :raises ValueError: If mode is not valid
        
        **Example Usage:**
        
            .. code-block:: python
            
                # Using enum (recommended)
                sensor.enable_auto_smux(as7343.AutoSMUXMode.TWELVE_CHANNEL)
                
                # Using int value
                sensor.enable_auto_smux(2)
        """
        if isinstance(mode, AutoSMUXMode):
            mode_value = mode.value
        elif isinstance(mode, int):
            if mode not in (0, 2, 3):
                raise ValueError("Auto-SMUX mode must be 0, 2, or 3")
            mode_value = mode
        else:
            raise TypeError("Mode must be AutoSMUXMode enum or int")
        
        cfg20 = mode_value << 5  # Set auto_smux bits [6:5]
        self._write_register(_AS7343_CFG20, cfg20)
    
    def read_all_channels_auto(self) -> List[int]:
        """
        Read all 12 spectral channels using auto-SMUX feature.
        
        This method uses the AS7343's auto-SMUX feature to automatically
        cycle through two measurement configurations, returning data for
        all 12 spectral channels. The channels are returned in this order:
        
        Index 0: FZ_450nm
        Index 1: FY_555nm
        Index 2: FXL_600nm
        Index 3: NIR_855nm
        Index 4: VIS (clear)
        Index 5: VIS2 (clear)
        Index 6: F2_425nm
        Index 7: F3_475nm
        Index 8: F4_515nm
        Index 9: F6_640nm
        Index 10: VIS3 (clear)
        Index 11: VIS4 (clear)
        
        :return: List of 12 channel values as 16-bit integers (0-65535)
        
        **Example Usage:**
        
            .. code-block:: python
            
                channels = sensor.read_all_channels_auto()
                
                # Access specific channels by index
                fz_450nm = channels[0]
                fy_555nm = channels[1]
                
                # Or use all_channels property for labeled access
                all_ch = sensor.all_channels
                blue_450nm = all_ch['FZ_450nm']
        """
        # Enable auto-SMUX mode 2 (12 channels)
        self.enable_auto_smux(2)
        
        # Start measurement
        self.start_measurement()
        
        # Wait for first cycle to complete
        while not self.measurement_complete():
            time.sleep(0.01)
        
        # Read first 6 channels
        channels = []
        for i in range(6):
            addr = _AS7343_CH0_DATA_L + (i * 2)
            data = self._read_register(addr, 2)
            value = data[0] | (data[1] << 8)
            channels.append(value)
        
        # Wait for second cycle to complete
        time.sleep(0.1)  # Give time for auto-SMUX to switch
        while not self.measurement_complete():
            time.sleep(0.01)
        
        # Read next 6 channels
        for i in range(6):
            addr = _AS7343_CH0_DATA_L + (i * 2)
            data = self._read_register(addr, 2)
            value = data[0] | (data[1] << 8)
            channels.append(value)
        
        return channels
    
    def set_gain(self, gain: Union[Gain, float]) -> None:
        """
        Set the sensor gain.
        
        :param gain: Gain value as Gain enum or float (0.5, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048)
        :raises ValueError: If gain value is not valid
        
        **Example Usage:**
        
            .. code-block:: python
            
                # Using enum (recommended)
                sensor.set_gain(as7343.Gain.X16)
                
                # Using float value
                sensor.set_gain(16.0)
        """
        if isinstance(gain, Gain):
            gain_enum = gain
        elif isinstance(gain, (int, float)):
            if gain not in _GAIN_VALUE_TO_ENUM:
                raise ValueError(f"Invalid gain value. Must be one of: {list(_GAIN_VALUE_TO_ENUM.keys())}")
            gain_enum = _GAIN_VALUE_TO_ENUM[gain]
        else:
            raise TypeError("Gain must be Gain enum or float value")
        
        self._write_register(_AS7343_CFG1, gain_enum & 0x1F)
        
        # Verify the gain was set correctly
        time.sleep(0.01)  # Small delay to ensure register is updated
        actual_gain = self.gain
        if actual_gain != gain:
            print(f"Warning: Gain set to {gain}x but read back as {actual_gain}x")
    
    @property
    def gain(self) -> float:
        """
        Get the current gain setting.
        
        :return: Current gain value as float multiplier
        
        **Example Usage:**
        
            .. code-block:: python
            
                current_gain = sensor.gain
                print(f"Current gain: {current_gain}x")
        """
        gain_reg = self._read_register(_AS7343_CFG1) & 0x1F
        
        # Find matching enum value
        for gain_enum, reg_value in list(Gain.__members__.items()):
            if reg_value.value == gain_reg:
                return _GAIN_ENUM_TO_VALUE[Gain(reg_value)]
        
        # If no match found, return a default value
        print(f"Warning: Unknown gain register value: {gain_reg}")
        return 1.0  # Default to 1x gain
    
    def check_saturation(self, channels: List[int]) -> Tuple[bool, List[int]]:
        """
        Check if any channels are saturated (near max value).
        
        :param channels: List of channel values to check
        :return: Tuple of (is_saturated, list of saturated channel indices)
        """
        max_value = 0xFFFF  # 16-bit max
        saturation_threshold = max_value * 0.95  # 95% of max
        
        saturated = []
        for i, value in enumerate(channels):
            if value > saturation_threshold:
                saturated.append(i)
        
        return len(saturated) > 0, saturated
    
    def auto_adjust_gain(self, channels: List[int]) -> Tuple[bool, float]:
        """
        Automatically adjust gain based on channel readings.
        
        This method checks if any channels are saturated or too low and adjusts
        the gain accordingly. Returns whether gain was changed and the new gain value.
        
        :param channels: List of channel values to evaluate
        :return: Tuple of (gain_changed, new_gain_value)
        
        **Example Usage:**
        
            .. code-block:: python
            
                channels = sensor.read_all_channels_auto()
                gain_changed, new_gain = sensor.auto_adjust_gain(channels)
                if gain_changed:
                    print(f"Gain adjusted to {new_gain}x")
                    # Re-read with new gain
                    channels = sensor.read_all_channels_auto()
        """
        current_gain = self.gain
        max_value = max(channels)
        
        # Get ordered list of gain values
        gain_values = sorted(_GAIN_ENUM_TO_VALUE.values())
        
        # If saturated, reduce gain
        # The saturation threshold is set to ~18,000 (out of 65,535 max)
        # to provide headroom before actual saturation occurs
        if max_value > 18000:  # Approximate saturation threshold
            if current_gain > 0.5:
                # Find next lower gain
                current_idx = gain_values.index(current_gain)
                if current_idx > 0:
                    new_gain = gain_values[current_idx - 1]
                    self.set_gain(new_gain)
                    return True, new_gain
        
        # If too low, increase gain  
        elif max_value < 1000 and current_gain < 2048:
            # Find next higher gain
            current_idx = gain_values.index(current_gain)
            if current_idx < len(gain_values) - 1:
                new_gain = gain_values[current_idx + 1]
                self.set_gain(new_gain)
                return True, new_gain
        
        return False, current_gain
    
    def set_integration_time(self, atime: int, astep: int) -> None:
        """
        Set integration time parameters.
        
        Integration time = (ATIME + 1) × (ASTEP + 1) × 2.78µs
        
        :param atime: ATIME register value (0-255)
        :param astep: ASTEP register value (0-65535)
        """
        if not 0 <= atime <= 255:
            raise ValueError("ATIME must be between 0 and 255")
        if not 0 <= astep <= 65535:
            raise ValueError("ASTEP must be between 0 and 65535")
            
        self._write_register(_AS7343_ATIME, atime & 0xFF)
        self._write_register(_AS7343_ASTEP_L, astep & 0xFF)
        self._write_register(_AS7343_ASTEP_H, (astep >> 8) & 0xFF)
    
    @property
    def integration_time_ms(self) -> float:
        """
        Get current integration time in milliseconds.
        
        :return: Integration time in milliseconds
        """
        atime = self._read_register(_AS7343_ATIME)
        astep_l = self._read_register(_AS7343_ASTEP_L)
        astep_h = self._read_register(_AS7343_ASTEP_H)
        astep = astep_l | (astep_h << 8)
        
        # Integration time = (ATIME + 1) × (ASTEP + 1) × 2.78µs
        return (atime + 1) * (astep + 1) * 2.78 / 1000
    
    def set_integration_time_ms(self, time_ms: float) -> None:
        """
        Set integration time in milliseconds (approximate).
        
        The actual integration time will be the closest achievable value
        based on available register settings.
        
        :param time_ms: Desired integration time in milliseconds
        """
        if time_ms <= 0:
            raise ValueError("Integration time must be positive")
            
        # Calculate ATIME and ASTEP for desired time
        # time_ms = (ATIME + 1) × (ASTEP + 1) × 2.78µs / 1000
        total_steps = int(time_ms * 1000 / 2.78)
        
        # Try to balance ATIME and ASTEP
        if total_steps < 256:
            atime = total_steps - 1
            astep = 0
        else:
            atime = 255
            astep = (total_steps // 256) - 1
            if astep > 65535:
                astep = 65535
        
        self.set_integration_time(atime, astep)
    
    def read_all_channels_calibrated(self) -> List[float]:
        """
        Read all channels and return calibrated values.
        
        Values are normalized by gain and integration time to provide
        counts per millisecond per gain unit.
        
        :return: List of calibrated channel values
        """
        channels = self.read_all_channels_auto()
        gain = self.gain
        integration_time = self.integration_time_ms
        
        # Add error checking
        if gain is None:
            print("Warning: gain is None, defaulting to 1.0")
            gain = 1.0
        
        if integration_time <= 0:
            print("Warning: invalid integration time, defaulting to 1.0")
            integration_time = 1.0
        
        # Normalize to counts per ms per gain
        calibrated = []
        for value in channels:
            try:
                calibrated_value = value / (gain * integration_time)
                calibrated.append(calibrated_value)
            except (TypeError, ZeroDivisionError) as e:
                print(f"Calibration error: {e}, using raw value")
                calibrated.append(value)
        
        return calibrated
    
    @property
    def all_channels(self) -> Dict[str, int]:
        """
        Get all channels in a dictionary with wavelength labels.
        
        :return: Dictionary mapping channel names to values
        """
        raw = self.read_all_channels_auto()
        # Rearrange to match spectral order
        return {
            'F2_425nm': raw[6],
            'FZ_450nm': raw[0], 
            'F3_475nm': raw[7],
            'F4_515nm': raw[8],
            'FY_555nm': raw[1],
            'FXL_600nm': raw[2],
            'F6_640nm': raw[9],
            'NIR_855nm': raw[3],
            'VIS': raw[4],
            'VIS2': raw[5],
            'VIS3': raw[10],
            'VIS4': raw[11]
        }
    
    # Individual channel properties
    @property
    def channel_450nm_fz(self) -> int:
        """FZ channel - 450nm (Blue)"""
        return self.read_all_channels_auto()[0]

    @property
    def channel_555nm_fy(self) -> int:
        """FY channel - 555nm (Green)"""  
        return self.read_all_channels_auto()[1]

    @property
    def channel_600nm_fxl(self) -> int:
        """FXL channel - 600nm (Orange)"""
        return self.read_all_channels_auto()[2]

    @property
    def channel_855nm_nir(self) -> int:
        """NIR channel - 855nm"""
        return self.read_all_channels_auto()[3]

    @property
    def channel_425nm_f2(self) -> int:
        """F2 channel - 425nm (Violet)"""
        return self.read_all_channels_auto()[6]

    @property
    def channel_475nm_f3(self) -> int:
        """F3 channel - 475nm (Blue)"""
        return self.read_all_channels_auto()[7]

    @property
    def channel_515nm_f4(self) -> int:
        """F4 channel - 515nm (Cyan)"""
        return self.read_all_channels_auto()[8]

    @property
    def channel_640nm_f6(self) -> int:
        """F6 channel - 640nm (Red)"""
        return self.read_all_channels_auto()[9]
