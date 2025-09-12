# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is an IoT tracking device system with two main components:
- **Tracking Device Firmware** (ESP32-based devices with multiple sensors and WiFi/cellular connectivity)
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
- GxEPD2 (E-ink display)
- Adafruit_SHT31 (Temperature/humidity sensor)
- Adafruit_LSM6DSL (Accelerometer/gyroscope)
- WiFiManager (WiFi configuration)

Main firmware file:
- Tracking Device: `arduino/Sketch/TrackingBoxMain/TrackingBoxMain.ino`

### Test Sketches
- `TrackingBoxDisplayTest/`: E-ink display testing
- `accel-gyro/accel-gyro/`: LSM6DSL accelerometer testing for tilt/fall detection  
- `gps-gnss-ip/`: SIM7600 GPS module testing
- `sht-gyro/`: Combined SHT31 and LSM6DSL sensor testing
- `epd7in3f-demo/`: E-ink display demo
- `sim7600_at_command_test/`: SIM7600 module AT command testing
- `sim7600_hardware_serial_diagnostic/`: Hardware serial diagnostic for SIM7600
- `dfr_firebase/`: Direct Firebase connectivity testing via cellular
- `dfr_firebase_connect/`: Firebase connection testing
- `ShippingLabelDisplay/`: QR code generation for shipping labels

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

#### Tracking Device Operation
The ESP32 tracking devices operate in cycles:
1. Deep sleep (15 minutes default)
2. Wake on timer, motion interrupt, or limit switch
3. Read sensors (temperature, humidity, GPS, accelerometer)
4. Connect to WiFi (uses WiFiManager for configuration)
5. Send data directly to Firebase via HTTPS
6. If WiFi fails, fallback to cellular data (SIM7600)
7. Retrieve control flags (buzzer, solenoid) from Firebase
8. Update e-ink display (LAST STEP - refresh takes significant time)
9. Return to sleep

**Important**: E-ink display update is performed last because the refresh system is time-consuming and would interrupt critical data transmission processes.

#### Dual-Mode Connectivity
The system prioritizes WiFi connection for data transmission:
1. **Primary**: WiFi connection to Firebase (preferred for cost/speed)
2. **Fallback**: Cellular data via SIM7600 when WiFi unavailable
3. **Configuration**: WiFiManager provides AP mode for initial WiFi setup

#### Pin Configurations
**Source**: `arduino/Pin Configs.txt` - Verified working configuration
- SHT30: SDA=21, SCL=22
- LSM6DSL: SDA=21, SCL=22, INT1=34  
- SIM7600: RX=18, TX=19 (Hardware Serial UART2)
- E-ink: DIN=14, SCLK=13, CS=15, DC=27, RST=26, BUSY=25
- Battery ADC: Pin 36
- Buzzer: Pin 32
- Limit Switch: Pin 33 (wake interrupt)
- Solenoid: Pin 2
- LED Indicator: Pin 4

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
- WiFiManager for user-friendly WiFi configuration
- Cellular fallback for connectivity issues
- Battery voltage monitoring with ADC
- E-ink display updates performed last due to time constraints
- RTC memory persistence across deep sleep cycles

## Location Services
- **Primary**: GNSS/GPS via SIM7600G module
- **Fallback**: CLBS (Cell Location Based Services) when GPS unavailable
- **Coordinate Parsing**: Supports both decimal degrees and DMS formats
- **Reverse Geocoding**: OpenStreetMap Nominatim API for location names

## Development Mode

For continuous testing without deep sleep:
- Comment out deep sleep calls in setup() and loop()
- Device will run continuous 30-second cycles
- Useful for debugging sensor readings and connectivity

### Testing Approach
- Individual component tests in `arduino/Sketch/` subdirectories
- Each sensor has dedicated test sketch for isolated debugging
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
- WiFi credentials configured via WiFiManager AP mode (no hardcoding needed)
- SIM7600 APN configuration: "internet" (modify for your carrier)
- Device ID auto-generation with MAC address fallback for uniqueness
- Development mode available (disables deep sleep for testing)
- Battery voltage monitoring with ADC calibration
- RTC memory persistence for data across deep sleep cycles
- Buffer size: 2048 bytes for Firebase JSON responses
- Wake interrupt sources: Timer (15 min), Motion (LSM6DSL), Limit switch

## Recent Architecture Changes

### WiFi-First Connectivity Strategy
The system prioritizes WiFi over cellular for cost optimization:
- WiFiManager provides user-friendly WiFi configuration via AP mode
- Automatic fallback to cellular (SIM7600) when WiFi unavailable
- Connection retry logic with exponential backoff
- RTC memory stores WiFi failure count to optimize reconnection attempts

### Enhanced Wake System
- **Timer Wake**: 15-minute intervals for regular updates
- **Motion Wake**: LSM6DSL interrupt on movement/tilt detection
- **Limit Switch Wake**: Immediate wake on box opening/closing
- Wake reason tracked and reported to Firebase for analytics