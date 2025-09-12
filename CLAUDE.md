# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is an IoT tracking device system with three main components:
- **Tracking Device Firmware** (ESP32-based devices with multiple sensors and cellular connectivity)
- **Master Device Firmware** (ESP32 with SIM7600 for SMS-to-Firebase relay - legacy/fallback)
- **Web Dashboard** (Next.js/React application for real-time monitoring)

## Commands

### Web Dashboard Development
```bash
# Install dependencies
npm install

# Run development server (http://localhost:3000)
npm run dev

# Build for production
npm run build

# Build for Cloudflare Pages deployment
npm run pages:build

# Preview Cloudflare Pages build locally
npm run preview

# Deploy to Cloudflare Pages
npm run deploy

# Run linting
npm run lint

# Type checking
npx tsc --noEmit
```

### Arduino Development
Use Arduino IDE with the following libraries installed:
- ArduinoJson
- GxEPD2
- Adafruit_SHT31
- SparkFun_LSM6DSL
- TinyGSM (for SIM7600 cellular connectivity)

Main firmware file:
- Tracking Device: `arduino/Sketch/TrackingBoxMain/TrackingBoxMain.ino`

### Test Sketches
- `TrackingBoxDisplayTest/`: E-ink display testing
- `TrackingBoxMain_SMS_BACKUP.ino`: Legacy SMS-based version (preserved for fallback)
- `accel-gyro/accel-gyro/`: LSM6DSL accelerometer testing for tilt/fall detection  
- `gps-gnss-ip/`: SIM7600 GPS module testing
- `sht-gyro/`: Combined SHT31 and LSM6DSL sensor testing
- `epd7in3f-demo/`: E-ink display demo
- `sim7600_at_command_test/`: SIM7600 module AT command testing
- `sim7600_hardware_serial_diagnostic/`: Hardware serial diagnostic for SIM7600
- `dfr_firebase/`: Direct Firebase connectivity testing via cellular
- `dfr_firebase_connect/`: Firebase connection testing
- `ShippingLabelDisplay/`: QR code generation for shipping labels
- `MasterSMSToFirebase/`: Master device SMS relay testing

## Architecture

### Web Dashboard Structure
- **src/app/**: Next.js app directory with Firebase integration
  - `firebase.ts`: Firebase configuration and helper functions
  - `page.tsx`: Main dashboard with real-time monitoring
  - `qr-link/`: Device sharing via QR codes
  - `qr/[deviceId]/`: Device-specific QR tracking pages
  - `api/tracking/[deviceId]/sensor/`: API endpoints for sensor data
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
**Source**: `arduino/Pin Configs.txt` and verified working configuration
- SHT30: SDA=21, SCL=22
- LSM6DSL: SDA=21, SCL=22, INT1=34
- SIM7600: RX=18, TX=19 (UART2 - verified working)
- E-ink: DIN=14, SCLK=13, CS=15, DC=27, RST=26, BUSY=25
- Battery ADC: Pin 36
- Buzzer: Pin 32
- Limit Switch: Pin 33
- Solenoid: Pin 2
- LED: Pin 4

### Firebase Data Structure
```
tracking_box/
  box_XXX/
    details/
      name, setLocation, setLocationLabel, description, referenceCode,
      packDate, packWeight, productFrom, packerShipper, supplierIdTracking,
      senderName, senderAddress, recipientName, recipientAddress,
      routingCode, postalCode, trackingNumber, serviceType
    sensorData/  // Can be single object (PUT) or multiple with push IDs (POST)
      temp, humidity, accelerometer, currentLocation, 
      batteryVoltage, wakeReason, timestamp, solenoid,
      limitSwitch, tilt, fall, bootCount, altitude,
      buzzerIsActive, buzzerDismissed
    controlFlags/
      buzzer, solenoid, timestamp  // Control states set by web dashboard
    alerts/
      motion/{pushID}/    // Motion detection alerts
      critical/{pushID}/  // Security breach alerts
      safe/{pushID}/      // Delivery confirmation alerts
    dismissAlert/
      dismissed, timestamp
```

## Real-time Alert System

The dashboard implements a sophisticated alert system:
- **Motion Alerts**: Triggered by accelerometer events (shake/tilt)
- **Critical Alerts**: Security breaches (geofencing violations)
- **Safe Alerts**: Delivery confirmations (limit switch events)

All alerts:
- Display as yellow toast notifications (15 seconds)
- Auto-cleanup from Firebase after display
- Include device ID, location, and contextual message

## API Endpoints

### Sensor Data API
`POST /api/tracking/[deviceId]/sensor`
- Accepts JSON sensor data from devices
- Updates Firebase sensorData path
- Returns control flags for device actions

`GET /api/tracking/[deviceId]/sensor`
- Returns latest sensor readings
- Used by dashboard for real-time updates

## Key Development Patterns

### React/Next.js
- Uses TypeScript with strict mode and path aliases (`@/*` maps to `./src/*`)
- Tailwind CSS for styling with custom theme and animations (`tailwindcss-animate`)
- Shadcn/UI component library with "new-york" style and Lucide React icons
- Firebase Realtime Database for live updates
- Radix UI components for accessible UI elements
- React Leaflet for map visualization
- Recharts for data visualization with 5 predefined chart colors
- React Hot Toast for notifications
- React QR Code for QR code generation
- Class Variance Authority for component styling utilities

### Arduino/ESP32
- Power optimization through deep sleep
- Interrupt-based wake system
- Modular sensor handling
- SMS fallback for connectivity issues
- Battery voltage monitoring with ADC
- Master-slave architecture for reliable data transmission
- E-ink display updates performed last due to time constraints
- RTC memory persistence across deep sleep cycles

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
- Cloudflare Pages deployment configured in `wrangler.toml`
- Build output for Cloudflare: `.vercel/output/static`
- Components configured via `components.json` for Shadcn/UI

### Arduino Firmware
- WiFi credentials must be set in firmware before upload (when using WiFi mode)
- SIM7600 APN configuration required for cellular connectivity
- Device ID and owner info configured in main sketch
- Development mode available (disables deep sleep)
- Battery voltage calibration may be needed based on voltage divider
- Device ID auto-generation system prevents duplicates
- MAC address-based unique ID generation when preferred ID taken
- Buffer size configured at 2048 bytes for Firebase JSON responses
- UART2 (GPIO 18/19) verified as working configuration for SIM7600

## Recent Architecture Changes

### Direct Firebase Connection (Current)
The system has been migrated from SMS-based communication to direct Firebase connection via cellular data:
- Each tracking device now uses SIM7600 for direct Firebase updates
- SMS functionality preserved as fallback/legacy mode
- Master device role reduced but maintained for backward compatibility
- Device ID validation and auto-generation for duplicate prevention
- Runtime ID persistence across deep sleep cycles using RTC memory

### Location Services
- Primary: GNSS/GPS via SIM7600G module
- Fallback: CLBS (Cell Location Based Services) when GPS unavailable
- Coordinate parsing supports both decimal and DMS formats
- Reverse geocoding via OpenStreetMap Nominatim API