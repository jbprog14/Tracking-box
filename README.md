# 📦 Tracking Box Device - IoT Tracking System

A comprehensive IoT tracking device system built with Arduino/ESP32 firmware and React web dashboard, featuring multiple sensors, E-ink display, and real-time Firebase integration.

## 🎯 Project Overview

This project consists of two main components:

1. **Hardware Firmware** (Arduino/ESP32) - IoT tracking device with multiple sensors
2. **Web Dashboard** (React/Next.js) - Real-time monitoring and device management interface

## 🔧 Hardware Components

- **ESP32** - Main microcontroller
- **SHT30** - Temperature & Humidity sensor
- **LSM6DSL** - 6-axis Accelerometer/Gyroscope for tilt detection
- **SIM7600** - GNSS/GPS module for location tracking
- **Waveshare 3.7" E-ink Display** - Low-power information display
- **Li-Po Battery** - Power source with voltage monitoring
- **Buzzer** - Audio notifications

## ⚡ Core Functionality

### Device Operation

1. **Sleep Mode**: ESP32 sleeps to conserve battery
2. **Wake Triggers**:
   - Timer (every 15 minutes)
   - Accelerometer interrupt (motion/tilt detection)
3. **Sensor Reading**: When awake, reads all sensors
4. **Data Upload**: Sends readings to Firebase database
5. **Display Update**: Updates E-ink display with current information
6. **Return to Sleep**: Goes back to deep sleep mode

### Web Dashboard

- **Real-time Monitoring**: Live sensor data display
- **Device Management**: Edit device info, location, descriptions
- **Data Visualization**: Charts for temperature, humidity, battery, location history
- **QR Code Generation**: Easy device sharing
- **Responsive Design**: Works on desktop and mobile

## 🚀 Quick Start

### Web Dashboard

```bash
# Install dependencies
npm install

# Run development server
npm run dev

# Build for production
npm run build
```

### Arduino Firmware

1. **Install Required Libraries:**

   - ArduinoJson (required)
   - GxEPD2 (optional - for E-ink display)
   - Adafruit_SHT31 (optional - for temperature sensor)

2. **Configure Settings:**

   ```cpp
   // Enable/disable features
   #define ENABLE_EINK_DISPLAY 1    // Set to 0 to disable
   #define ENABLE_SHT30_SENSOR 0    // Set to 1 to enable

   // Update WiFi credentials
   const char* WIFI_SSID = "your_wifi_name";
   const char* WIFI_PASSWORD = "your_wifi_password";
   ```

3. **Upload Main Firmware:**
   - Open `arduino/Sketch/TrackingBoxMain/TrackingBoxMain.ino`
   - Select ESP32 board
   - Upload to device

## 📁 Project Structure

```
tracking-box/
├── arduino/
│   ├── FEATURES.md              # Hardware requirements
│   └── Sketch/
│       ├── TrackingBoxMain/     # Main production firmware
│       ├── ink/                 # E-ink display test
│       ├── gps/                 # GPS/GNSS test
│       ├── SHT31test/           # Temperature sensor test
│       └── lsm6dsl 6-axis/      # Accelerometer test
├── src/
│   ├── app/
│   │   ├── firebase.ts          # Firebase configuration
│   │   ├── page.tsx             # Main dashboard
│   │   └── qr-link/             # QR code page
│   └── components/
│       ├── TrackingBoxModal.tsx # Device detail view
│       └── EditInfoModal.tsx    # Device editing interface
└── public/                      # Static assets
```

## 🔌 Pin Configuration

### ESP32 Pin Mapping

```
SHT30 Sensor:     SDA=18, SCL=19
LSM6DSL Accel:    SDA=21, SCL=22
SIM7600 GPS:      RX=32, TX=33
E-ink Display:    CS=5, DC=17, RST=16, BUSY=4
Buzzer:           Pin 25
Accel Interrupt:  Pin 26
Battery Monitor:  Pin 35 (ADC)
```

## 🌐 Firebase Integration

The device automatically uploads sensor data to Firebase Realtime Database:

```json
{
  "tracking_box": {
    "box_001": {
      "details": {
        "name": "Tracking Box 001",
        "setLocation": "Default Location",
        "description": "Device description..."
      },
      "sensorData": {
        "temp": 25.6,
        "humidity": 60.2,
        "accelerometer": "NORMAL",
        "currentLocation": "40.7128,-74.0060",
        "batteryVoltage": 3.85,
        "wakeReason": "Timer (15 minutes)",
        "timestamp": 1640995200000,
        "bootCount": 42,
        "altitude": 156.0
      }
    }
  }
}
```

## 🔋 Power Management

- **Deep Sleep**: Device sleeps between readings to conserve battery
- **Wake Sources**: 15-minute timer OR accelerometer interrupt
- **Battery Monitoring**: Real-time voltage monitoring with web dashboard display
- **E-ink Display**: Ultra-low power consumption, retains image when powered off

## 🛠️ Testing & Development

### Individual Component Tests

- `ink.ino` - Test E-ink display functionality
- `gps.ino` - Test GPS/GNSS location tracking
- `SHT31test.ino` - Test temperature/humidity sensor
- `lsm6dsl 6-axis.ino` - Test accelerometer/gyroscope

### Development Mode

Set deep sleep to disabled for testing:

```cpp
// In setup() and loop(), deep sleep is commented out for testing
// Device runs continuous 30-second cycles instead of sleeping
```

## 📱 Web Dashboard Features

- **Device Overview**: Real-time status of all tracking devices
- **Sensor Charts**: Visual graphs for temperature, humidity, battery level
- **Location Tracking**: GPS coordinate history and mapping
- **Device Management**: Edit names, locations, descriptions
- **QR Code Sharing**: Generate QR codes for easy device access
- **Responsive Design**: Mobile-friendly interface

## 🔑 Admin Access

- **Username**: `Admin123`
- **Password**: `123123123a`

## 🚨 Troubleshooting

### Arduino Compilation Issues

- Ensure ArduinoJson library is installed (required)
- For E-ink: Install GxEPD2 library and set `ENABLE_EINK_DISPLAY 1`
- For SHT30: Install Adafruit_SHT31 library and set `ENABLE_SHT30_SENSOR 1`

### E-ink Display Not Updating

- Check pin connections match the configuration
- Verify GxEPD2 library is installed
- Ensure `ENABLE_EINK_DISPLAY` is set to `1`

### WiFi Connection Issues

- Update SSID and password in firmware
- Check WiFi signal strength
- Verify Firebase configuration

## 📄 License

This project is open source and available under the MIT License.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

---

Built with ❤️ for IoT tracking and monitoring applications.
