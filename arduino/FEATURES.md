Tracking Box Device

Components: - SHT30 - SIM7600 GNSS (Location) - ESP32 - Accelerometer 6-axis LSM6DSL - Li-Po Battery - 3.7 waveshare E-ink Display - Buzzer

Core Functionality:

When the device are powered 

- Connect to network
- Send all sensor readings on firebase database in realtime and in bulk
- Enter sleep mode

When the device is in sleep mode 

- After 15 minutes of sleep, it will wake up and repeat the cycle for new readings unless, it detects a tilting or free falling
- The lsm6dsl will listen to acceleration changes (Z-axis)
- The device will wake up if it touches the threshold for Z-axis
