# code.py v21 - Full featured test for AS7343
import board
import time
import as7343

i2c = board.STEMMA_I2C()
try:
    sensor = as7343.AS7343(i2c)
    print("AS7343 sensor found!")
    
    # Set initial parameters
    sensor.set_gain(16)  # Start with 16x gain
    sensor.set_integration_time_ms(100)  # 100ms integration time
    
    while True:
        # Read all channels
        channels = sensor.read_all_channels_auto()
        
        # Auto-adjust gain if needed
        gain_changed, current_gain = sensor.auto_adjust_gain(channels)
        if gain_changed:
            print(f"Gain adjusted to: {current_gain}x")
            channels = sensor.read_all_channels_auto()
        
        # Get all channels in a nice format
        all_ch = sensor.all_channels
        
        # Print results
        print(f"\n=== Sensor Readings ===")
        print(f"Gain: {sensor.gain}x, Integration: {sensor.integration_time_ms:.1f}ms")
        print("\nSpectral Channels:")
        print(f"F2  (425nm): {all_ch['F2_425nm']:5d}")
        print(f"FZ  (450nm): {all_ch['FZ_450nm']:5d}")  
        print(f"F3  (475nm): {all_ch['F3_475nm']:5d}")
        print(f"F4  (515nm): {all_ch['F4_515nm']:5d}")
        print(f"FY  (555nm): {all_ch['FY_555nm']:5d}")
        print(f"FXL (600nm): {all_ch['FXL_600nm']:5d}")
        print(f"F6  (640nm): {all_ch['F6_640nm']:5d}")
        print(f"NIR (855nm): {all_ch['NIR_855nm']:5d}")
        
        print("\nClear Channels:")
        print(f"VIS:  {all_ch['VIS']:5d}")
        print(f"VIS2: {all_ch['VIS2']:5d}")
        print(f"VIS3: {all_ch['VIS3']:5d}")
        print(f"VIS4: {all_ch['VIS4']:5d}")
        
        # Get calibrated readings (counts/ms/gain)
        calibrated = sensor.read_all_channels_calibrated()
        max_cal = max(calibrated)
        if max_cal > 0:
            print("\nRelative Intensities:")
            ch_names = list(all_ch.keys())
            for i, cal_value in enumerate(calibrated):
                if i < len(ch_names):
                    name = ch_names[i]
                    bar_length = int(cal_value / max_cal * 40)
                    bar = '█' * bar_length
                    print(f"{name:10s} {bar}")
        
        time.sleep(2)
        
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exception(type(e), e, e.__traceback__)