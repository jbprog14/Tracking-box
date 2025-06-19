Tracking Box Device

Components: - SHT30 - SIM7600 GNSS (Location) - ESP32 - Accelerometer 6-axis LSM6DSL - Li-Po Battery - 3.7 waveshare E-ink Display - Buzzer

Core Functionality:

1. Display information details on E-ink Display
2. Accelerometer will constantly listen for interruption despite ESP32's sleep mode
3. If Accelerometer detects tilt or interruption, ESP32 will wake up
4. During wake mode, SHT30 will detect Temp and Humidity, GNSS (Sim7600) will detect current location
5. SHT30, GNSS and Accelerometer readings will send directly into Firebase Database using all credentials from src/app/firebase.ts
6. Put ESP32 sleep mode when it passed 15 minutes

- IMPORTANT : On Every 15 minutes interval ESP32 will wake, unless it got interrupted by Accelerometer even if it's not 15 minutes passed, wake ESP32. \*

- DEFAULT : One device = One Tracker all these readings will be under a single unique ID along with owners information e.g Name, Address, Tracking Box no. into database. \*
