# as7343.py - AS7343 CircuitPython library
import time
from adafruit_bus_device import i2c_device
from micropython import const

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

# Gain values mapping
_AS7343_GAIN_VALUES = {
    0.5: 0,
    1: 1,
    2: 2,
    4: 3,
    8: 4,
    16: 5,
    32: 6,
    64: 7,
    128: 8,
    256: 9,
    512: 10,
    1024: 11,
    2048: 12
}

class AS7343:
    def __init__(self, i2c_bus, address=0x39):
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        
        # Need to set correct register bank to read ID
        self._write_register(_AS7343_CFG0, _AS7343_REG_BANK)  # Set REG_BANK = 1
        time.sleep(0.01)
        
        # Check device ID
        device_id = self._read_register(_AS7343_WHOAMI)
        print(f"Read device ID: {device_id:#x}")
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
        with self.i2c_device as i2c:
            i2c.write(bytes([address, value & 0xFF]))
    
    def _smux_command(self, command):
        """Execute SMUX command"""
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
        """Start a spectral measurement"""
        enable = self._read_register(_AS7343_ENABLE)
        enable |= _AS7343_SP_EN
        self._write_register(_AS7343_ENABLE, enable)
    
    def measurement_complete(self):
        """Check if measurement is complete"""
        # Check STATUS2 register for AVALID bit
        status2 = self._read_register(_AS7343_STATUS2)
        return bool(status2 & _AS7343_AVALID)
    
    def read_all_channels(self):
        """Read all 6 basic channels"""
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
        """Enable auto-SMUX mode
        mode: 0 = 6 channel (single cycle)
              2 = 12 channel (2 cycles) 
              3 = 18 channel (3 cycles with Flicker)
        """
        cfg20 = mode << 5  # Set auto_smux bits [6:5]
        self._write_register(_AS7343_CFG20, cfg20)
    
    def read_all_channels_auto(self):
        """Read all channels using auto-SMUX (12 channels)"""
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
        """Set the sensor gain (0.5, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048)"""
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
        """Get the current gain setting"""
        gain_reg = self._read_register(_AS7343_CFG1) & 0x1F
        for gain_value, reg_value in _AS7343_GAIN_VALUES.items():
            if reg_value == gain_reg:
                return gain_value
        # If no match found, return a default value
        print(f"Warning: Unknown gain register value: {gain_reg}")
        return 1.0  # Default to 1x gain
    
    def check_saturation(self, channels):
        """Check if any channels are saturated (near max value)"""
        max_value = 0xFFFF  # 16-bit max
        saturation_threshold = max_value * 0.95  # 95% of max
        
        saturated = []
        for i, value in enumerate(channels):
            if value > saturation_threshold:
                saturated.append(i)
        
        return len(saturated) > 0, saturated
    
    def auto_adjust_gain(self, channels):
        """Automatically adjust gain based on channel readings"""
        current_gain = self.gain
        max_value = max(channels)
        
        # If saturated, reduce gain
        if max_value > 18000:  # Approximate saturation threshold
            if current_gain > 0.5:
                # Find next lower gain
                gains = sorted(_AS7343_GAIN_VALUES.keys())
                current_idx = gains.index(current_gain)
                if current_idx > 0:
                    new_gain = gains[current_idx - 1]
                    self.set_gain(new_gain)
                    return True, new_gain
        
        # If too low, increase gain  
        elif max_value < 1000 and current_gain < 2048:
            # Find next higher gain
            gains = sorted(_AS7343_GAIN_VALUES.keys())
            current_idx = gains.index(current_gain)
            if current_idx < len(gains) - 1:
                new_gain = gains[current_idx + 1]
                self.set_gain(new_gain)
                return True, new_gain
        
        return False, current_gain
    
    def set_integration_time(self, atime, astep):
        """Set integration time parameters
        Integration time = (ATIME + 1) × (ASTEP + 1) × 2.78µs
        """
        self._write_register(_AS7343_ATIME, atime & 0xFF)
        self._write_register(_AS7343_ASTEP_L, astep & 0xFF)
        self._write_register(_AS7343_ASTEP_H, (astep >> 8) & 0xFF)
    
    @property
    def integration_time_ms(self):
        """Get current integration time in milliseconds"""
        atime = self._read_register(_AS7343_ATIME)
        astep_l = self._read_register(_AS7343_ASTEP_L)
        astep_h = self._read_register(_AS7343_ASTEP_H)
        astep = astep_l | (astep_h << 8)
        
        # Integration time = (ATIME + 1) × (ASTEP + 1) × 2.78µs
        return (atime + 1) * (astep + 1) * 2.78 / 1000
    
    def set_integration_time_ms(self, time_ms):
        """Set integration time in milliseconds (approximate)"""
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
        """Read all channels and return calibrated values considering gain and integration time"""
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
        """Return all channels in a standardized order"""
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