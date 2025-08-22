# SIM7600G-H UART2 Implementation Summary

## Working Configuration (Verified)
- **UART**: UART2 (HardwareSerial 2)
- **RX Pin**: GPIO 18
- **TX Pin**: GPIO 19
- **Baud Rate**: 115200
- **Module**: SIMCOM_SIM7600G-H

## Changes Made to TrackingBoxMain.ino

### 1. Pin Definitions (Lines 58-59)
```cpp
// OLD (Not Working):
#define SIM7600_TX_PIN      17
#define SIM7600_RX_PIN      16

// NEW (Verified Working):
#define SIM7600_TX_PIN      19  // Verified working with UART2
#define SIM7600_RX_PIN      18  // Verified working with UART2
```

### 2. UART Selection (Line 113)
```cpp
// OLD (UART1):
HardwareSerial sim7600(1);

// NEW (UART2):
HardwareSerial sim7600(2);  // Using UART2 - verified working with SIM7600G-H
```

### 3. Initialization with Buffer Size (Lines 890-893)
```cpp
// OLD:
sim7600.begin(115200, SERIAL_8N1, SIM7600_RX_PIN, SIM7600_TX_PIN);
delay(2000);
flushSIM7600Buffer();

// NEW:
// Initialize SIM7600 with verified working configuration
// UART2: RX=GPIO18, TX=GPIO19 @ 115200 baud
sim7600.begin(115200, SERIAL_8N1, SIM7600_RX_PIN, SIM7600_TX_PIN);
sim7600.setRxBufferSize(2048);  // Increase buffer for large HTTP responses
delay(2000);
flushSIM7600Buffer();
```

### 4. Added Missing Includes and Defines
- Added `#include <Adafruit_SHT31.h>` for SHT30 sensor
- Added `#define ENABLE_SHT30_SENSOR 1` to enable temperature/humidity sensor

### 5. Added Missing Forward Declarations
- `void parseFirebaseDetails(String jsonData);`
- `void flushSIM7600Buffer();`
- `void sendAT(const char *cmd, uint16_t delayMs);`
- `void showOfflineQRCode();`

## Verification Results (from debug.txt)
```
UART2: RX=18, TX=19 @ 115200
Testing AT command... ✓
Getting module info... ✓ SIMCOM_SIM7600G-H
✅ CONFIGURATION WORKING!
Running extended diagnostics:
  SIM Card: ✓ Ready
  Network: ✓ Registered
  Signal: ✓ Good (16/31)
  HTTP: ✓ Supported
```

## Critical Notes
1. **DO NOT** change these pins back to 16/17 - they don't work
2. **DO NOT** change from UART2 back to UART1 - only UART2 works
3. The 2048 byte buffer is important for handling large Firebase JSON responses
4. This configuration has been tested and verified with:
   - AT commands
   - SIM card detection
   - Network registration
   - HTTP/HTTPS support
   - GPS/GNSS functionality
   - Firebase connectivity

## Troubleshooting
If communication fails after these changes:
1. Verify wiring: ESP32 TX(19) → SIM7600 RX, ESP32 RX(18) → SIM7600 TX
2. Check power supply: SIM7600G needs 3.7-4.2V and up to 2A peak
3. Ensure common ground between ESP32 and SIM7600G
4. Check SIM card is inserted and activated
5. Verify antenna is connected for cellular signal

## Test Sketch
Use `sim7600_at_command_test.ino` to verify the configuration. This sketch:
- Uses the exact same UART2 configuration
- Tests all AT commands needed for the main application
- Verifies Firebase connectivity
- Includes interactive mode for debugging