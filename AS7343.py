# as7343.py v21 - AS7343 CircuitPython library
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

**Hardware Limitations on Pimoroni Breakout:**

* GPIO functionality (register 0x6B): The AS7343 chip has a GPIO pin that can be configured as input/output, 
* but this pin is not accessible on the Pimoroni breakout board. The board only exposes VCC, GND, SDA, SCL, and INT pins.

* Interrupt functionality (register 0xF9): The AS7343 supports hardware interrupts for events like measurement completion, 
* threshold detection, and FIFO status. The INT pin is available on the Pimoroni breakout and can be used 
* with CircuitPython's pin interrupt features. Future implementation could include threshold interrupts and measurement ready signals.
"""
import time
from adafruit_bus_device import i2c_device
from micropython import const

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
_AS7343_CFG1 = const(0xC6)  # FIXED: Was 0xAA, corrected to 0xC6
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

# Valid gain values
GAIN_0_5X = 0.5
GAIN_1X = 1
GAIN_2X = 2
GAIN_4X = 4
GAIN_8X = 8
GAIN_16X = 16
GAIN_32X = 32
GAIN_64X = 64
GAIN_128X = 128
GAIN_256X = 256
GAIN_512X = 512
GAIN_1024X = 1024
GAIN_2048X = 2048

# Gain values mapping
_AS7343_GAIN_VALUES = {
    GAIN_0_5X: 0,
    GAIN_1X: 1,
    GAIN_2X: 2,
    GAIN_4X: 3,
    GAIN_8X: 4,
    GAIN_16X: 5,
    GAIN_32X: 6,
    GAIN_64X: 7,
    GAIN_128X: 8,
    GAIN_256X: 9,
    GAIN_512X: 10,
    GAIN_1024X: 11,
    GAIN_2048X: 12
}

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
            sensor.set_gain(16)  # 16x gain
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
    
    def __init__(self, i2c_bus, address=0x39):
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
    
    def _read_register(self, address, length=1):
        """
        Read a register or sequence of registers.
        
        :param address: Register address to read from
        :param length: Number of bytes to read, defaults to 1
        :return: Register value(s) as int for single byte or bytearray for multiple
        """
        with self.i2c_device as i2c:
            if length == 1:
                result = bytearray(1)
                i2c.write_then_readinto(bytes([address]), result)
                return result[0]
            else:
                result = bytearray(length)
                i2c.write_then_readinto(bytes([address]), result)
                return result
    
    def _write_register(self, address, value):
        """
        Write a value to a register.
        
        :param address: Register address to write to
        :param value: Value to write (max 8 bits)
        """
        with self.i2c_device as i2c:
            self._buffer[0] = address
            self._buffer[1] = value & 0xFF
            i2c.write(self._buffer)
    
    def _smux_command(self, command):
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
    
    def start_measurement(self):
        """Start a spectral measurement."""
        enable = self._read_register(_AS7343_ENABLE)
        enable |= _AS7343_SP_EN
        self._write_register(_AS7343_ENABLE, enable)
    
    def measurement_complete(self):
        """
        Check if measurement is complete.
        
        :return: True if measurement is complete, False otherwise
        """
        # Check STATUS2 register for AVALID bit
        status2 = self._read_register(_AS7343_STATUS2)
        return bool(status2 & _AS7343_AVALID)
    
    def read_all_channels(self):
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
    
    def enable_auto_smux(self, mode=2):
        """
        Enable auto-SMUX mode for automatic channel switching.
        
        :param mode: Auto-SMUX mode:
                     - 0: 6 channel (single cycle)
                     - 2: 12 channel (2 cycles)
                     - 3: 18 channel (3 cycles with Flicker)
        :raises ValueError: If mode is not valid
        """
        if mode not in (0, 2, 3):
            raise ValueError("Auto-SMUX mode must be 0, 2, or 3")
        
        cfg20 = mode << 5  # Set auto_smux bits [6:5]
        self._write_register(_AS7343_CFG20, cfg20)
    
    def read_all_channels_auto(self):
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
    
    def set_gain(self, gain):
        """
        Set the sensor gain.
        
        :param gain: Gain value (0.5, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048)
        :raises ValueError: If gain value is not valid
        """
        if gain not in _AS7343_GAIN_VALUES:
            raise ValueError(f"Invalid gain value. Must be one of: {list(_AS7343_GAIN_VALUES.keys())}")
        
        gain_reg = _AS7343_GAIN_VALUES[gain]
        self._write_register(_AS7343_CFG1, gain_reg & 0x1F)
        
        # Verify the gain was set correctly
        time.sleep(0.01)  # Small delay to ensure register is updated
        actual_gain = self.gain
        if actual_gain != gain:
            print(f"Warning: Gain set to {gain}x but read back as {actual_gain}x")
    
    @property
    def gain(self):
        """
        Get the current gain setting.
        
        :return: Current gain value as float multiplier
        """
        gain_reg = self._read_register(_AS7343_CFG1) & 0x1F
        for gain_value, reg_value in _AS7343_GAIN_VALUES.items():
            if reg_value == gain_reg:
                return gain_value
        # If no match found, return a default value
        print(f"Warning: Unknown gain register value: {gain_reg}")
        return 1.0  # Default to 1x gain
    
    def check_saturation(self, channels):
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
    
    def auto_adjust_gain(self, channels):
        """
        Automatically adjust gain based on channel readings.
        
        This method checks if any channels are saturated or too low and adjusts
        the gain accordingly. Returns whether gain was changed and the new gain value.
        
        :param channels: List of channel values to evaluate
        :return: Tuple of (gain_changed, new_gain_value)
        """
        current_gain = self.gain
        max_value = max(channels)
        
        # Get ordered list of gain values
        gain_values = sorted(_AS7343_GAIN_VALUES.keys())
        
        # Check for saturation using the check_saturation method
        is_saturated, saturated_channels = self.check_saturation(channels)
        
        # If saturated, reduce gain
        if is_saturated:
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
    
    def set_integration_time(self, atime, astep):
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
    def integration_time_ms(self):
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
    
    def set_integration_time_ms(self, time_ms):
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
    
    def read_all_channels_calibrated(self):
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
    def all_channels(self):
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
    def channel_450nm_fz(self):
        """FZ channel - 450nm (Blue)"""
        return self.read_all_channels_auto()[0]

    @property
    def channel_555nm_fy(self):
        """FY channel - 555nm (Green)"""  
        return self.read_all_channels_auto()[1]

    @property
    def channel_600nm_fxl(self):
        """FXL channel - 600nm (Orange)"""
        return self.read_all_channels_auto()[2]

    @property
    def channel_855nm_nir(self):
        """NIR channel - 855nm"""
        return self.read_all_channels_auto()[3]

    @property
    def channel_425nm_f2(self):
        """F2 channel - 425nm (Violet)"""
        return self.read_all_channels_auto()[6]

    @property
    def channel_475nm_f3(self):
        """F3 channel - 475nm (Blue)"""
        return self.read_all_channels_auto()[7]

    @property
    def channel_515nm_f4(self):
        """F4 channel - 515nm (Cyan)"""
        return self.read_all_channels_auto()[8]

    @property
    def channel_640nm_f6(self):
        """F6 channel - 640nm (Red)"""
        return self.read_all_channels_auto()[9]
