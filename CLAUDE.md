# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is an IoT tracking device system with three main components:
- **Tracking Device Firmware** (ESP32-based slave devices with sensors)
- **Master Device Firmware** (ESP32 with SIM7600 for SMS-to-Firebase relay)
- **Web Dashboard** (Next.js/React application for monitoring)

## Commands

### Web Dashboard Development
```bash
# Install dependencies
npm install

# Run development server (http://localhost:3000)
npm run dev

# Build for production
npm run build

# Run linting
npm run lint

# Type checking
npx tsc --noEmit

# Deploy to Cloudflare Pages
npm run deploy
```

### Arduino Development
Use Arduino IDE with the following libraries installed:
- ArduinoJson
- GxEPD2
- Adafruit_SHT31
- SparkFun_LSM6DSL
- TinyGSM (for SIM7600 cellular connectivity)

Main firmware files:
- Tracking Device: `arduino/Sketch/TrackingBoxMain/TrackingBoxMain.ino`
- Master Device: `arduino/Sketch/MasterDevice/MasterDevice.ino`

### Test Sketches
- `TrackingBoxDisplayTest/`: E-ink display testing
- `accel-gyro/`: LSM6DSL accelerometer testing for tilt/fall detection
- `gps-gnss-ip/`: SIM7600 GPS module testing
- `sht-gyro/`: Combined SHT31 and LSM6DSL sensor testing
- `epd7in3f-demo/`: E-ink display demo
- `M2S/`: SMS reading functionality
- `MasterSMSToFirebase/`: SMS receiver debug tool
- `sim7600_at_command_test/`: SIM7600 module AT command testing
- `dfr_firebase/`: Firebase connectivity testing

## Architecture

### Web Dashboard Structure
- **src/app/**: Next.js app directory with Firebase integration
  - `firebase.ts`: Firebase configuration and helper functions
  - `page.tsx`: Main dashboard with real-time monitoring
  - `qr-link/`: Device sharing via QR codes
- **src/components/**: React components for device management
  - `TrackingBoxModal.tsx`: Device detail view with charts
  - `EditInfoModal.tsx`: Device info editing
- **src/components/ui/**: Reusable UI components (Radix UI based)

### Arduino Firmware Architecture

#### Tracking Device (Slave) Operation
The ESP32 tracking devices operate in cycles:
1. Deep sleep (15 minutes default)
2. Wake on timer or motion interrupt
3. Read sensors (temperature, humidity, GPS, accelerometer)
4. Send data directly to Firebase via cellular data (SIM7600)
5. If cellular fails, fallback to SMS mode (legacy)
6. Update e-ink display (LAST STEP - refresh takes significant time)
7. Return to sleep

**Important**: E-ink display update is performed last because the refresh system is time-consuming and would interrupt critical data transmission processes.

#### Master Device Operation
A dedicated ESP32 with SIM7600 that:
1. Receives SMS messages from tracking devices
2. Parses sensor data from SMS format
3. Forwards data to Firebase via WiFi
4. Retrieves control states from Firebase (buzzer, solenoid, etc.)
5. Sends control commands back to Slave devices via SMS
6. Acts as a bidirectional relay for areas with poor connectivity

#### Communication Flow
```
1. Current Operation (Direct Cellular):
   Tracking Device <---> Firebase (via SIM7600 cellular data)

2. Legacy SMS Fallback Mode (preserved for compatibility):
   Slave Device --SMS--> Master Device --WiFi--> Firebase
   Slave Device <--SMS-- Master Device <--WiFi-- Firebase
```

**Important**: When Slave devices cannot connect to WiFi, they rely on the Master device for both sending sensor data AND receiving control commands (buzzer activation, solenoid control, etc.)

#### Pin Configurations
**Note**: There are discrepancies between documentation. Use these from `Pin Configs.txt`:
- SHT30: SDA=21, SCL=22
- LSM6DSL: SDA=21, SCL=22, INT=34
- SIM7600: RX=16, TX=17 (Note: Recent code may use pins 18/19 for UART2)
- E-ink: DIN=14, SCLK=13, CS=15, DC=27, RST=26, BUSY=25
- Battery ADC: Pin 36
- Buzzer: Pin 32
- Limit Switch: Pin 33
- Solenoid: Pin 2

### Firebase Data Structure
```
tracking_box/
  box_XXX/
    details/
      name, setLocation, description, referenceCode
    sensorData/
      temp, humidity, accelerometer, currentLocation, 
      batteryVoltage, wakeReason, timestamp, solenoid,
      limitSwitch, tilt, fall
    controlFlags/
      buzzer, solenoid  // Control states set by web dashboard
```

## Key Development Patterns

### React/Next.js
- Uses TypeScript with strict mode
- Tailwind CSS for styling with custom theme
- Firebase Realtime Database for live updates
- Radix UI components for accessible UI elements
- React Leaflet for map visualization
- Recharts for data visualization

### Arduino/ESP32
- Power optimization through deep sleep
- Interrupt-based wake system
- Modular sensor handling
- SMS fallback for connectivity issues
- Battery voltage monitoring with ADC
- Master-slave architecture for reliable data transmission
- E-ink display updates performed last due to time constraints

## SMS Communication Protocol

### Slave-to-Master Message Format
```
DEVICE_ID,timestamp,temp,humidity,lat,lng,alt,tilt,fall,limitSwitch,solenoid,buzzer,coarseFix,usingCGPS,accelX,accelY,accelZ,batteryVoltage,wakeUpReason
```

### Example Slave-to-Master SMS
```
box_001,1703123456789,25.5,60.0,14.562000,121.112100,15.0,0,0,1,0,0,0,0,0.020,-0.010,0.980,3.85,TIMER DUE (15mns.)
```

### Master-to-Slave Control Message Format
The Master device sends control commands back to Slave devices after checking Firebase:
```
CMD,buzzerState,solenoidState,additionalFlags
```

### Example Master-to-Slave SMS
```
CMD,1,0,0
```
Where:
- buzzerState: 0=off, 1=on
- solenoidState: 0=closed, 1=open
- additionalFlags: Reserved for future use

## Development Mode

For continuous testing without deep sleep:
- Comment out deep sleep calls in setup() and loop()
- Device will run continuous 30-second cycles
- Useful for debugging sensor readings and connectivity

### Testing Approach
- Individual component tests in `arduino/Sketch/` subdirectories
- Each sensor has dedicated test sketch for isolated debugging
- Master device can be tested with `MasterSMSToFirebase` sketch
- Web dashboard development server supports hot reload

## Important Configuration

### Web Dashboard
- Firebase is pre-configured in `src/app/firebase.ts`
- Admin login: Username: `Admin123`, Password: `123123123a`
- Tailwind config includes custom CSS variables for theming

### Arduino Firmware
- WiFi credentials must be set in firmware before upload (when using WiFi mode)
- SIM7600 APN configuration required for cellular connectivity
- Device ID and owner info configured in main sketch
- Development mode available (disables deep sleep)
- Battery voltage calibration may be needed based on voltage divider

## Recent Architecture Changes

### Direct Firebase Connection (Current)
The system has been migrated from SMS-based communication to direct Firebase connection via cellular data:
- Each tracking device now uses SIM7600 for direct Firebase updates
- SMS functionality preserved as fallback/legacy mode
- Master device role reduced but maintained for backward compatibility
- Device ID validation and auto-generation for duplicate prevention
- Runtime ID persistence across deep sleep cycles using RTC memory