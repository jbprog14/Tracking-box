# Tracking Box Main Firmware

This is the complete combined Arduino sketch that integrates all individual sensor components into one unified tracking device system.

## Overview

The **TrackingBoxMain.ino** sketch combines functionality from all the individual test sketches:

- `SHT31test/` - Temperature & Humidity sensor
- `ink/` - E-ink display functionality
- `gps/` - SIM7600 GPS/GNSS tracking
- `lsm6dsl 6-axis/` - Accelerometer tilt detection

## Features Implemented

✅ **All FEATURES.md requirements satisfied:**

1. **Display information on E-ink Display** - Shows sensor readings, GPS coordinates, system status
2. **Accelerometer constant monitoring** - Configured for interrupt-based wake-up during deep sleep
3. **Wake on tilt detection** - ESP32 wakes immediately when accelerometer detects tilt
4. **Sensor reading on wake** - SHT30, GPS, and accelerometer data collected every wake cycle
5. **Firebase integration** - All sensor data sent to Firebase database with device credentials
6. **15-minute timer wake** - Automatic wake every 15 minutes regardless of interrupts
7. **Deep sleep power management** - Returns to low-power sleep after each operation cycle
8. **Unique device tracking** - Single device ID with owner information stored in database

## Hardware Requirements

### Components

- **ESP32** microcontroller
- **SHT30** temperature & humidity sensor
- **SIM7600** GNSS/GPS module
- **LSM6DSL** 6-axis accelerometer/gyroscope
- **3.7" Waveshare E-ink display**
- **Buzzer** for audio feedback
- **Li-Po battery** with voltage monitoring

### Pin Connections

```
SHT30:          SDA=18, SCL=19
LSM6DSL:        SDA=21, SCL=22
SIM7600:        RX=16, TX=17
E-ink Display:  CS=5, DC=0, RST=2, BUSY=4
Buzzer:         PIN=25
Accelerometer:  INT=26 (interrupt)
Battery ADC:    PIN=35
```

## Software Requirements

### Arduino Libraries

```cpp
// Core libraries
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <HardwareSerial.h>

// ESP32 specific
#include "esp_sleep.h"
#include "driver/rtc_io.h"

// Sensor libraries
#include "Adafruit_SHT31.h"

// Display libraries
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
```

### Required Library Installation

```bash
# Install via Arduino Library Manager:
- Adafruit SHT31 Library
- GxEPD2 by Jean-Marc Zingg
- ArduinoJson by Benoit Blanchon
```

## Configuration

### 1. WiFi Setup

```cpp
// Update these credentials in the sketch:
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
```

### 2. Device Information

```cpp
// Customize device details:
const String DEVICE_ID = "TRACKER_001";
const String OWNER_NAME = "Device Owner";
const String OWNER_ADDRESS = "Owner Address";
```

### 3. Firebase Configuration

The Firebase credentials are already configured from `src/app/firebase.ts`:

- Database URL: `tracking-box-e17a1-default-rtdb.asia-southeast1.firebasedatabase.app`
- API Key: Pre-configured in sketch

## Operation Cycle

### 1. **Wake Event**

- Timer-based: Every 15 minutes
- Interrupt-based: Accelerometer tilt detection

### 2. **Sensor Reading**

- SHT30: Temperature & humidity
- LSM6DSL: Accelerometer data & tilt status
- SIM7600: GPS coordinates (if fix available)
- Battery: Voltage level monitoring

### 3. **Data Transmission**

- Connect to WiFi network
- Send JSON payload to Firebase database
- Disconnect WiFi to save power

### 4. **Display Update**

- Update E-ink display with current readings
- Show system status and diagnostics

### 5. **Sleep Mode**

- Configure wake-up sources
- Enter deep sleep for power conservation

## Firebase Data Structure

Data is sent to Firebase with this JSON structure:

```json
{
  "deviceId": "TRACKER_001",
  "ownerName": "Device Owner",
  "ownerAddress": "Owner Address",
  "temperature": 25.6,
  "humidity": 65.2,
  "latitude": 14.5995,
  "longitude": 120.9842,
  "altitude": 10.5,
  "tiltDetected": false,
  "wakeReason": "Timer (15 minutes)",
  "batteryVoltage": 3.7,
  "timestamp": "12345678",
  "bootCount": 42,
  "gpsFixValid": true
}
```

## Power Management

- **Deep Sleep Current**: ~10µA (ESP32 + sensors)
- **Active Current**: ~200mA (during operation)
- **Wake Sources**: Timer (15 min) OR accelerometer interrupt
- **Battery Life**: Estimated 30+ days on 3000mAh Li-Po

## Debugging

### Serial Monitor Output

```
=====================================================
        TRACKING BOX DEVICE - STARTUP
=====================================================
Boot Count: 1
Wake Reason: Power On / First Boot
Timestamp: 1234

✓ Hardware initialization successful
✓ SHT30 sensor initialized
✓ LSM6DSL accelerometer initialized
✓ SIM7600 GPS module initialized
✓ E-ink display initialized

Reading all sensors...
✓ GPS fix acquired

✓ WiFi connected successfully
IP Address: 192.168.1.100

✓ Firebase upload successful
Response Code: 200

✓ Operation cycle completed successfully
Sleep configuration complete
Entering deep sleep mode...
```

## Troubleshooting

### Common Issues

1. **WiFi Connection Failed**

   - Check SSID/password credentials
   - Verify network availability
   - Check signal strength

2. **GPS No Fix**

   - Allow more time for satellite acquisition
   - Check antenna connection
   - Verify outdoor/window location

3. **Sensor Initialization Failed**

   - Check I2C wiring connections
   - Verify power supply (3.3V)
   - Test individual sensor sketches

4. **Firebase Upload Failed**

   - Verify internet connectivity
   - Check Firebase credentials
   - Ensure database rules allow writes

5. **Display Not Updating**
   - Check SPI connections
   - Verify display power
   - Test with ink/ sketch individually

## File Structure

```
TrackingBoxMain/
├── TrackingBoxMain.ino     # Main combined sketch
├── README.md               # This documentation
└── (header files)          # E-ink display headers from ink/
```

## Development Notes

- Based on individual sketches from `../SHT31test/`, `../ink/`, `../gps/`, `../lsm6dsl 6-axis/`
- Implements all requirements from `../../FEATURES.md`
- Uses Firebase configuration from `../../../src/app/firebase.ts`
- Optimized for low-power battery operation
- Includes comprehensive error handling and debugging

## SMS Fallback Communication

### Overview

The firmware now includes a **multi-tier communication fallback system** that ensures data transmission even in areas with poor connectivity:

1. **WiFi + Firebase** (Preferred - full functionality)
2. **Cellular + Firebase** (Limited functionality, no display refresh)
3. **SMS to Master Device** (Emergency fallback, minimal data)
4. **Offline mode with QR code** (No connectivity available)

### SMS Slave Mode

When both WiFi and Cellular connections fail, the device automatically switches to **SMS Slave Mode**:

- Formats sensor data as a comma-separated SMS message
- Sends SMS to a configured Master device phone number
- Master device receives SMS and forwards data to Firebase
- Minimal power consumption compared to continuous connection attempts

### SMS Message Format

```
DEVICE_ID,timestamp,temp,humidity,lat,lng,alt,tilt,fall,limitSwitch,accelX,accelY,accelZ,batteryVoltage,wakeUpReason
```

**Example SMS:**

```
box_001,1703123456789,25.5,60.0,14.562000,121.112100,15.0,0,0,1,0.020,-0.010,0.980,3.85,TIMER DUE (15mns.)
```

### Configuration

Update the Master device phone number in the sketch:

```cpp
// SMS FALLBACK CONFIGURATION
const String MASTER_PHONE_NUMBER = "+1234567890"; // Replace with actual Master device number
```

### Master Device Setup

Use the **MasterSMSToFirebase** sketch on a separate ESP32 with SIM7600 module:

1. Place Master device in location with reliable WiFi
2. Configure same Firebase credentials
3. Master receives SMS messages from multiple tracking devices
4. Automatically parses and forwards data to Firebase database

### Communication Flow

```
Tracking Device (Slave)
        ↓
    WiFi Failed?
        ↓
   Cellular Failed?
        ↓
    SMS to Master ──→ Master Device ──→ Firebase
```

### Benefits

- **Redundant Communication**: Multiple fallback options ensure data delivery
- **Emergency Coverage**: SMS works in areas with basic cellular coverage
- **Cost Effective**: SMS only used when other methods fail
- **Centralized**: Multiple devices can use single Master for Firebase access
- **Low Power**: Minimal SMS transmission preserves battery life

## License

This firmware is part of the Tracking Box Device project and follows the same licensing terms as the overall project.
