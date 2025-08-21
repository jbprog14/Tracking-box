/*
 * =====================================================================
 * TRACKING BOX DEVICE - DIRECT FIREBASE FIRMWARE
 * =====================================================================
 * 
 * This sketch implements a streamlined, single-cycle operation for the
 * tracking device using direct Firebase communication via cellular data.
 * 
 * On every wake-up, it performs the following:
 * 1. Initialize all hardware.
 * 2. Gather a full set of sensor readings.
 * 3. Send sensor data directly to Firebase via cellular HTTP.
 * 4. Check for control commands from Firebase.
 * 5. Update the E-Ink display with all data.
 * 6. Enter deep sleep until next wake trigger.
 * 
 * FIREBASE COMMUNICATION:
 * The device communicates directly with Firebase Realtime Database
 * using the SIM7600G's cellular data connection via HTTP/HTTPS.
 * 
 * CELLULAR LOCATION SERVICES:
 * SIM7600 cellular module is used for GNSS/GPS tracking and CLBS (Cell Location
 * Based Services) as a fallback when GPS signal is unavailable.
 * 
 * The device wakes from deep sleep based on three triggers:
 * - A 15-minute timer.
 * - The box lid being opened (limit switch).
 * - A significant shock or movement detected by the LSM6DSL accelerometer.
 * 
 * =====================================================================
 */

#include <ArduinoJson.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_LSM6DSL.h>
#include <Preferences.h>  // For permanent storage
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "DEV_Config.h"
#include "EPD.h"
#include "GUI_Paint.h"
#include "qrcode.h"   // QR code generator for display
#include <math.h>  // for haversine
// Direct AT commands are used for GNSS/GPS and CLBS location services
#include <time.h>
#define DEBUG_GNSS 1   // Set to 1 to enable verbose GNSS diagnostics (adds delay)

// =====================================================================
// PIN DEFINITIONS
// =====================================================================
#define BATTERY_ADC_PIN     36
#define SHT30_SDA_PIN       21
#define SHT30_SCL_PIN       22
#define LSM6DSL_SDA_PIN     21
#define LSM6DSL_SCL_PIN     22
#define LSM6DSL_INT1_PIN    34
#define SIM7600_TX_PIN      17
#define SIM7600_RX_PIN      16
#define LIMIT_SWITCH_PIN    33
#define BUZZER_PIN          32
#define SOLENOID_PIN        2   // GPIO2 – electronic lock/solenoid signal
#define LED_INDICATOR_PIN   4   // GPIO4 – LED indicator for wake/sleep status

// =====================================================================
// LSM6DSL CONSTANTS
// =====================================================================
// LSM6DSL Registers
#define LSM6DSL_CTRL1_XL         0x10
#define LSM6DSL_CTRL3_C          0x12
#define LSM6DSL_TAP_CFG          0x58
#define LSM6DSL_WAKE_UP_THS      0x5B // Wake-up threshold register
#define LSM6DSL_WAKE_UP_DUR      0x5C // Wake-up duration register
#define LSM6DSL_MD1_CFG          0x5E // Interrupt 1 routing register

// LSM6DSL I2C Addresses
#define LSM6DSL_ADDR1            0x6A // SDO/SA0 is low
#define LSM6DSL_ADDR2            0x6B // SDO/SA0 is high

// =====================================================================
// SYSTEM CONFIGURATION
// =====================================================================
#define SLEEP_MINUTES       15
#define uS_TO_S_FACTOR      1000000ULL
#define SLEEP_TIME_US       (SLEEP_MINUTES * 60 * uS_TO_S_FACTOR)
// New tilt / free-fall thresholds using accel-gyro technique
#define TILT_THRESHOLD_Z_AXIS       0.0  // g – if Z-axis accel < this → tilted 90°
#define FALL_THRESHOLD_MAGNITUDE    1.1  // g – CHANGE in accel magnitude to trigger a shock event

// =====================================================================
// DEVICE & FIREBASE CONFIGURATION
// =====================================================================
const String DEVICE_ID = "box_001";  // Base/preferred device ID

// Device ID validation - auto-generates unique ID if base ID exists
String actualDeviceID = "";  // Runtime device ID (either original or auto-generated)
RTC_DATA_ATTR char rtcActualDeviceID[32] = "";  // Persist across deep sleep
RTC_DATA_ATTR bool rtcDeviceIDValidated = false;  // Flag to track if ID was validated

// FIREBASE CONFIGURATION
const char* FIREBASE_URL = "https://tracking-box-e17a1-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* APN = "internet";  // Change to your carrier's APN

// =====================================================================
// GLOBAL OBJECTS & VARIABLES
// =====================================================================
volatile bool limitSwitchTriggered = false;  // Flag for limit switch interrupt
volatile bool motionDetected = false;  // Flag for motion interrupt
#if ENABLE_SHT30_SENSOR
Adafruit_SHT31 sht30 = Adafruit_SHT31();
#endif
Adafruit_LSM6DSL lsm6ds = Adafruit_LSM6DSL();
HardwareSerial sim7600(1);
Preferences preferences;  // For permanent storage
// SIM7600 module is used for both GNSS/GPS location services and cellular data for Firebase
uint8_t lsm6dsl_address = 0x6A;
// Flags indicating whether each sensor initialised correctly (ported from sht-gyro example)
bool sht30_ok   = false;
bool lsm6dsl_ok = false;
// RTC memory to store last accelerometer reading across deep sleep cycles
RTC_DATA_ATTR float rtcLastAccelX = 0.0;
RTC_DATA_ATTR float rtcLastAccelY = 0.0;
RTC_DATA_ATTR float rtcLastAccelZ = 0.0;
RTC_DATA_ATTR bool rtcBaselineSet = false;
RTC_DATA_ATTR bool rtcLastTiltState = false; // To track tilt state changes, like in the test sketch
RTC_DATA_ATTR uint32_t rtcBootCount = 0;     // persists across deep-sleep cycles

// RTC memory to store device details
RTC_DATA_ATTR char rtcDeviceSetLocation[64] = "Unknown";
RTC_DATA_ATTR char rtcDeviceName[64] = "Unknown";
RTC_DATA_ATTR bool rtcDeviceDetailsValid = false;

// RTC memory to store buzzer state
RTC_DATA_ATTR bool rtcBuzzerActive = false;
RTC_DATA_ATTR bool rtcBuzzerDismissed = false;

// RTC memory to store solenoid state
RTC_DATA_ATTR bool rtcSolenoidActive = false;
RTC_DATA_ATTR unsigned long rtcSolenoidStartTime = 0;

// RTC memory for security breach tracking
RTC_DATA_ATTR bool rtcSecurityBreachDetected = false;  // Tracks if limit switch was ever breached

// RTC memory for unique reference code
RTC_DATA_ATTR char rtcReferenceCode[11] = ""; // 10 chars + null terminator
RTC_DATA_ATTR bool rtcReferenceCodeGenerated = false;

// RTC memory for last Firebase update time
RTC_DATA_ATTR unsigned long rtcLastFirebaseUpdate = 0;

// --------------------------------------------------------------
// GEO HELPERS
// --------------------------------------------------------------
// Simple haversine – returns great-circle distance in meters
double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0; // Earth radius metres
  double dLat = (lat2 - lat1) * DEG_TO_RAD;
  double dLon = (lon2 - lon1) * DEG_TO_RAD;
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
                 sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

// --------------------------------------------------------------
// Parse coordinate string that may be in:
//   1) Decimal "lat, lon"       e.g. 14.5620,121.1121
//   2) Decimal with spaces       e.g. "14.5620, 121.1121"
//   3) Simple DMS string         e.g. "14°33'43.1\"N 121°06'43.3\"E"
// Only two components (lat,lon) are supported; altitude ignored.
// --------------------------------------------------------------
bool parseCoordPair(const String &raw, double &lat, double &lon) {
  String str = raw;
  str.trim();

  // --- Case 1: decimal with comma ---
  int comma = str.indexOf(',');
  if (comma != -1) {
    lat = str.substring(0, comma).toFloat();
    lon = str.substring(comma + 1).toFloat();
    if (!isnan(lat) && !isnan(lon) && lat != 0.0) return true;
  }

  // --- Case 2: DMS pattern (very lightweight parser) ---
  // Expect four numbers: deg min sec for lat + dir, then same for lon
  double deg[2] = {0, 0}, min[2] = {0, 0}, sec[2] = {0, 0};
  char dir[2] = {'N', 'E'};

  // Replace degree, quote symbols with spaces for easier splitting
  String cleaned = str;
  cleaned.replace("°", " ");
  cleaned.replace("'", " ");
  cleaned.replace("\"", " ");

  // Split by space
  double nums[6];
  int numIdx = 0;
  int start = 0;
  while (numIdx < 6) {
    int space = cleaned.indexOf(' ', start);
    if (space == -1) space = cleaned.length();
    String token = cleaned.substring(start, space);
    token.trim();
    if (token.length() > 0 && isdigit(token[0])) {
      nums[numIdx++] = token.toFloat();
    }
    start = space + 1;
    if (start >= cleaned.length()) break;
  }
  if (numIdx == 6) {
    deg[0] = nums[0]; min[0] = nums[1]; sec[0] = nums[2];
    deg[1] = nums[3]; min[1] = nums[4]; sec[1] = nums[5];
    // Find N/S and E/W letters
    int nPos = str.indexOf('N');
    int sPos = str.indexOf('S');
    int ePos = str.indexOf('E');
    int wPos = str.indexOf('W');
    if (sPos != -1) dir[0] = 'S';
    if (wPos != -1) dir[1] = 'W';

    auto dmsToDec = [](double d, double m, double s, char c) {
      double dec = d + m / 60.0 + s / 3600.0;
      if (c == 'S' || c == 'W') dec = -dec;
      return dec;
    };

    lat = dmsToDec(deg[0], min[0], sec[0], dir[0]);
    lon = dmsToDec(deg[1], min[1], sec[1], dir[1]);
    return true;
  }

  return false; // unsupported format
}

// This structure holds all data, both from sensors and fetched from Firebase.
struct TrackerData {
  // Sensor-derived data
  float temperature = 0.0;
  float humidity = 0.0;
  double latitude = 0.0;
  double longitude = 0.0;
  float altitude = 0.0;
  bool tiltDetected = false;
  bool fallDetected = false;   // new free-fall / shock flag
  float batteryVoltage = 0.0;
  bool gpsFixValid = false;
  bool usingCGPS = false;  // NEW: indicates CGPSInfo method was used this cycle
  bool limitSwitchPressed = false;
  float accelX = 0.0;
  float accelY = 0.0;
  float accelZ = 0.0;
  String currentLocation = "Unknown";
  // Data fetched from Firebase
  String deviceName = "Unknown";
  String deviceSetLocation = "Unknown"; // coordinates
  String deviceSetLabel = "";            // human readable
  String deviceDescription = "No Description";
  String wakeUpReason = "Power On";
  bool buzzerIsActive = false;
  bool buzzerDismissed = false;
  bool solenoidActive = false; // NEW: current requested state from Firebase
  bool securityBreachActive = false;  // Tracks if security has been breached (limit switch opened)
  uint32_t bootCount = 0;   // number of wake-ups since power-on
  bool coarseFix = false;   // true if only CLBS/IP based fix available
  String referenceCode = "";  // Unique 10-character reference code
  
  // Shipping label data from Firebase
  String senderName = "";
  String senderAddress = "";
  String recipientName = "";
  String recipientAddress = "";
  String packWeight = "";
  String routingCode = "";
  String postalCode = "";
  String trackingNumber = "";
  String serviceType = "";
};

TrackerData currentData;

// Flag to request a restart after solenoid operation completes
bool restartAfterSolenoid = false;

// Flag to indicate we need to start a new cycle due to interrupt
bool startNewCycle = false;

// =====================================================================
// INTERRUPT HANDLERS
// =====================================================================
// Interrupt handler for limit switch
void IRAM_ATTR limitSwitchISR() {
  limitSwitchTriggered = true;
}

// Interrupt handler for motion detection
void IRAM_ATTR motionISR() {
  motionDetected = true;
}

// Function to check if operation should be interrupted
bool shouldInterruptOperation() {
  if (limitSwitchTriggered || motionDetected) {
    if (limitSwitchTriggered) {
      Serial.println("⚠️ LIMIT SWITCH TRIGGERED - Interrupting operation!");
    }
    if (motionDetected) {
      Serial.println("⚠️ MOTION DETECTED - Interrupting operation!");
    }
    return true;
  }
  return false;
}

// Forward declaration
void determineWakeUpReason();
void updateDisplay();
bool sendSensorDataToFirebase();
bool initializeCellularData();
void sendATCommand(const char* cmd, int timeout);
String sendATCommandResponse(const char* cmd, int timeout);
bool checkFirebaseControls();
void parseFirebaseControls(String jsonData);
void generateReferenceCode();
bool sendFirebaseHTTP(String path, String jsonData, String method);
String readFirebaseHTTP(String path);
bool checkDeviceIDExists(String deviceID);
String generateNextDeviceID(String currentID);
String validateAndGetUniqueDeviceID();

// =====================================================================
// MAIN SETUP (single cycle) – call new E-ink init just before display
// =====================================================================
void setup() {
  Serial.begin(115200);
  
  // Turn ON LED indicator immediately - device is awake
  pinMode(LED_INDICATOR_PIN, OUTPUT);
  digitalWrite(LED_INDICATOR_PIN, HIGH);
  
  delay(1000);
  Serial.println("WAKE");

  // Increment persistent boot counter
  rtcBootCount++;
  currentData.bootCount = rtcBootCount;

  // Load or generate reference code from permanent storage
  preferences.begin("tracking", false);  // Open in read/write mode
  String storedRefCode = preferences.getString("refCode", "");
  
  if (storedRefCode.length() == 0) {
    // No reference code exists in permanent storage, generate one
    generateReferenceCode();
    // Store it permanently
    preferences.putString("refCode", String(rtcReferenceCode));
    Serial.println("✓ Reference code saved to permanent storage");
    Serial.println("⚠️ This reference code is PERMANENT and cannot be changed!");
  } else {
    // Load existing reference code from permanent storage
    storedRefCode.toCharArray(rtcReferenceCode, sizeof(rtcReferenceCode));
    rtcReferenceCodeGenerated = true;
    Serial.println("✓ Loaded permanent reference code: " + storedRefCode);
  }
  preferences.end();
  
  currentData.referenceCode = String(rtcReferenceCode);

  determineWakeUpReason();

  // Initialize all hardware
  initializeAllHardware();
  Serial.println("✅ Hardware Initialized.");
  
  // Initialize cellular data connection for Firebase
  if (initializeCellularData()) {
    Serial.println("✅ Cellular data initialized for Firebase.");
  } else {
    Serial.println("❌ Failed to initialize cellular data.");
  }
  
  // Validate Device ID uniqueness (only if not already validated)
  if (!rtcDeviceIDValidated || strlen(rtcActualDeviceID) == 0) {
    Serial.println("\n🔍 Validating Device ID uniqueness...");
    actualDeviceID = validateAndGetUniqueDeviceID();
    actualDeviceID.toCharArray(rtcActualDeviceID, sizeof(rtcActualDeviceID));
    rtcDeviceIDValidated = true;
    
    // Save to preferences for permanent storage
    preferences.begin("tracking", false);
    preferences.putString("deviceID", actualDeviceID);
    preferences.end();
    
    if (actualDeviceID != DEVICE_ID) {
      Serial.println("⚠️ Original ID '" + DEVICE_ID + "' was already taken");
      Serial.println("✅ Assigned new unique ID: " + actualDeviceID);
    } else {
      Serial.println("✅ Using original ID: " + actualDeviceID);
    }
  } else {
    // Load from RTC memory (survives deep sleep)
    actualDeviceID = String(rtcActualDeviceID);
    Serial.println("✅ Using saved Device ID: " + actualDeviceID);
  }
  
  // Check for control commands from Firebase (skip on first boot)
  if (rtcBootCount > 1) {
    Serial.println("🌐 Checking Firebase for control commands...");
    checkFirebaseControls();
  } else {
    Serial.println("🌐 Skipping Firebase check on first boot");
  }
  
  // Restore device details from RTC memory if available
  if (rtcDeviceDetailsValid) {
    currentData.deviceName = String(rtcDeviceName);
    currentData.deviceSetLocation = String(rtcDeviceSetLocation);
    Serial.println("✓ Restored device details from RTC memory:");
    Serial.println("  Device name: " + currentData.deviceName);
    Serial.println("  Set location: " + currentData.deviceSetLocation);
  } else {
    Serial.println("⚠️ No device details in RTC memory - using defaults");
  }
  
  // Collect sensor data for Firebase transmission
  collectSensorReading();
  Serial.println("✅ Sensor Readings Collected.");
  
  // Send sensor data directly to Firebase
  if (sendSensorDataToFirebase()) {
    Serial.println("✅ Sensor data sent to Firebase.");
    
    // CRITICAL: If buzzer OR solenoid is active, DO NOT SLEEP
    if (currentData.buzzerIsActive || rtcBuzzerActive || currentData.solenoidActive || rtcSolenoidActive) {
      Serial.println("\n⚠️ ACTIVE CONTROL - STAYING AWAKE");
      
      if (currentData.buzzerIsActive || rtcBuzzerActive) {
        Serial.println("🚨 BUZZER ACTIVE - monitoring for dismiss");
      }
      if (currentData.solenoidActive || rtcSolenoidActive) {
        Serial.println("🔓 SOLENOID ACTIVE - running lock cycle");
      }
      
      // Keep monitoring while either is active
      unsigned long lastCheck = millis();
      const unsigned long CHECK_INTERVAL = 5000; // Check every 5 seconds
      unsigned long solenoidRunTime = 0;
      
      // If solenoid just activated, record start time
      if ((currentData.solenoidActive || rtcSolenoidActive) && rtcSolenoidStartTime == 0) {
        rtcSolenoidStartTime = millis();
        digitalWrite(SOLENOID_PIN, HIGH); // Activate solenoid
        Serial.println("🔓 Solenoid activated at " + String(rtcSolenoidStartTime));
      }
      
      // Display QR code immediately when entering active control mode
      if (!rtcBuzzerActive && !rtcSolenoidActive) {
        showOfflineQRCode();
      }
      
      while (currentData.buzzerIsActive || rtcBuzzerActive || currentData.solenoidActive || rtcSolenoidActive) {
        // Handle buzzer
        if (currentData.buzzerIsActive || rtcBuzzerActive) {
          digitalWrite(BUZZER_PIN, HIGH);
        }
        
        // Handle solenoid (15-second activation)
        if (currentData.solenoidActive || rtcSolenoidActive) {
          solenoidRunTime = millis() - rtcSolenoidStartTime;
          
          if (solenoidRunTime >= 15000) { // 15 seconds elapsed
            Serial.println("🔒 Solenoid deactivating after 15 seconds");
            digitalWrite(SOLENOID_PIN, LOW);
            currentData.solenoidActive = false;
            rtcSolenoidActive = false;
            rtcSolenoidStartTime = 0;
            
            // Solenoid cycle is complete
            // Update Firebase with status
          } else {
            // Keep solenoid active
            digitalWrite(SOLENOID_PIN, HIGH);
            Serial.println("🔓 Solenoid active for " + String(solenoidRunTime / 1000) + " seconds");
          }
        }
        
        // Check for new control commands
        if (millis() - lastCheck >= CHECK_INTERVAL) {
          lastCheck = millis();
          Serial.println("🌐 Checking Firebase for control commands...");
          
          // Check for control commands from Firebase
          checkFirebaseControls();
          
          // If both are deactivated, exit loop
          if (!currentData.buzzerIsActive && !rtcBuzzerActive && 
              !currentData.solenoidActive && !rtcSolenoidActive) {
            Serial.println("✅ All controls deactivated - preparing for sleep");
            break;
          }
        }
        
        // Check for interrupts
        if (shouldInterruptOperation()) {
          Serial.println("💤 Entering deep sleep to handle interrupt...");
          // Turn off buzzer and solenoid before sleep
          digitalWrite(BUZZER_PIN, LOW);
          digitalWrite(SOLENOID_PIN, LOW);
          digitalWrite(LED_INDICATOR_PIN, LOW);
          // Enter deep sleep - will wake with proper reason
          prepareForDeepSleep();
          esp_deep_sleep_start();
        }
        
        delay(100); // Small delay to prevent busy waiting
      }
    }
    
    // Update display with QR code
    showOfflineQRCode();
    
    Serial.println("Cycle complete → deep sleep (Firebase mode).");
  } else {
    Serial.println("❌ Firebase send failed. Operating in offline mode.");
    showOfflineQRCode();
    Serial.println("Cycle complete → deep sleep (offline mode).");
  }
  
  // CRITICAL: Set buzzer to correct state before deep sleep
  digitalWrite(BUZZER_PIN, rtcBuzzerActive ? HIGH : LOW);
  
  // Turn OFF LED indicator before sleep
  digitalWrite(LED_INDICATOR_PIN, LOW);
  Serial.println("💡 LED indicator OFF - Entering deep sleep");
  
  // Only sleep after buzzer is properly handled
  prepareForDeepSleep();
  esp_deep_sleep_start();
}

void loop() {
  // The loop is intentionally left empty.
  // The device performs a single cycle in setup() and then deep sleeps.
}

// Direct Firebase communication implemented

// ---------------------------------------------------------------------------
// SOLENOID CONTROL
// ---------------------------------------------------------------------------
// Solenoid state fetching handled via Firebase

// Solenoid activation handled via Firebase

// Lock breach evaluation handled via Firebase

// Buzzer monitoring handled via Firebase

// Solenoid activation wait handled via Firebase

// =====================================================================
// SENSOR READING FUNCTIONS
// =====================================================================
void collectSensorReading() {
  readTemperatureHumidity();
  readAccelerometerData();
  // ------------------------------------------------------------------
  // 1. Try to obtain a GNSS fix first (preferred, highest accuracy)
  // ------------------------------------------------------------------
  readGPSLocation();

  // ------------------------------------------------------------------
  // 2. If GNSS failed (no valid fix) fall back to a fast, coarse
  //    Cell-Tower Location (CLBS). We keep the original sensor-OK guard
  //    so the fallback only runs when the modem & sensors are healthy.
  // ------------------------------------------------------------------
  if (!currentData.gpsFixValid && lsm6dsl_ok) {
    readCellLocation();
  }

  readBatteryVoltage();
  
  // ------------------------------------------------------------------
  // Tilt & free-fall/shock detection – logic from accel-gyro.ino
  // This is now the single source of truth for this calculation.
  // ------------------------------------------------------------------
  if (lsm6dsl_ok) {
    // Tilt detection with state-change logging (from the test sketch)
    bool newTiltState = (currentData.accelZ < TILT_THRESHOLD_Z_AXIS);
    if (newTiltState != rtcLastTiltState) {
      if (newTiltState) {
        Serial.println("🚨 TILT DETECTED");
      } else {
        Serial.println("✅ Tilt Cleared");
      }
    }
    currentData.tiltDetected = newTiltState;
    rtcLastTiltState = newTiltState; // Persist for the next wake cycle

    float accelMagnitude = sqrt(currentData.accelX * currentData.accelX +
                                currentData.accelY * currentData.accelY +
                                currentData.accelZ * currentData.accelZ);
    
    // Shock/fall detection is now based on the CHANGE from the last reading
    if (rtcBaselineSet) {
      float lastMagnitude = sqrt(rtcLastAccelX * rtcLastAccelX + rtcLastAccelY * rtcLastAccelY + rtcLastAccelZ * rtcLastAccelZ);
      float delta = abs(accelMagnitude - lastMagnitude);
      
      // --- Diagnostic Logging ---
      Serial.printf("Last Accel Mag: %.2f, Current Accel Mag: %.2f, Delta: %.2f\n", lastMagnitude, accelMagnitude, delta);
      // --------------------------

      currentData.fallDetected = (delta > FALL_THRESHOLD_MAGNITUDE);
      if (currentData.fallDetected) {
        Serial.printf("⚡️ SHOCK DETECTED! Delta from last reading: %.2fg\n", delta);
      }
    } else {
      // This is the first reading cycle, so no shock is detected.
      // We just establish the baseline for the next cycle.
      currentData.fallDetected = false;
      rtcBaselineSet = true;
      Serial.println("Setting initial accelerometer baseline for shock detection.");
    }

    // Update RTC memory with the current reading for the next cycle
    rtcLastAccelX = currentData.accelX;
    rtcLastAccelY = currentData.accelY;
    rtcLastAccelZ = currentData.accelZ;

  } else {
    currentData.tiltDetected = false;
    currentData.fallDetected = false;
  }

  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  currentData.limitSwitchPressed = !digitalRead(LIMIT_SWITCH_PIN);
  
  // Attach interrupt for limit switch (triggers on FALLING edge when lid opens)
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchISR, FALLING);
  Serial.println("✓ Limit switch interrupt attached");
  
  // Track security breach - once breached, it stays breached until physically resolved
  if (!currentData.limitSwitchPressed) {
    // Lid is open - security breach!
    if (!rtcSecurityBreachDetected) {
      rtcSecurityBreachDetected = true;
      // When a new breach is detected, clear any previous dismissal
      rtcBuzzerDismissed = false;
      currentData.buzzerDismissed = false;
      Serial.println("🚨 NEW SECURITY BREACH DETECTED - Lid opened!");
      Serial.println("🚨 Clearing any previous dismissal flags");
    }
  } else {
    // Lid is closed - check if we can clear the security breach
    if (rtcSecurityBreachDetected) {
      Serial.println("🔒 Lid is now closed, but security breach remains active until location is verified safe");
      // Note: Security breach will only be cleared when both:
      // 1. Lid is closed (limitSwitchPressed = true)
      // 2. Device is back in safe zone OR user dismisses the alert
    }
  }
  currentData.securityBreachActive = rtcSecurityBreachDetected;

  if (currentData.gpsFixValid || currentData.coarseFix) {
    currentData.currentLocation = String(currentData.latitude, 4) + ", " + String(currentData.longitude, 4);
  } else {
    currentData.currentLocation = "GPS Initializing. Please Wait. . .";
  }

  // Debug: print all sensor and state variables
  Serial.printf("DEBUG | Temp=%.2fC Hum=%.2f%% Batt=%.2fV Acc=%.3fg %.3fg %.3fg GPSValid=%d LimitSwitch=%d Tilt=%d Fall=%d\n",
                currentData.temperature,
                currentData.humidity,
                currentData.batteryVoltage,
                currentData.accelX,
                currentData.accelY,
                currentData.accelZ,
                currentData.gpsFixValid,
                currentData.limitSwitchPressed,
                currentData.tiltDetected,
                currentData.fallDetected
  );
}

void readTemperatureHumidity() {
  #if ENABLE_SHT30_SENSOR
  if (sht30_ok) {
    currentData.temperature = sht30.readTemperature();
    currentData.humidity    = sht30.readHumidity();

    if (isnan(currentData.temperature)) currentData.temperature = -999;
    if (isnan(currentData.humidity))    currentData.humidity    = -999;
  } else {
    currentData.temperature = -999;
    currentData.humidity    = -999;
  }
  #else
  // Use simulated data when SHT30 is compile-time disabled
  currentData.temperature = 25.0 + (random(-50, 50) / 10.0);
  currentData.humidity    = 50.0 + (random(-200, 200) / 10.0);
  #endif
}

void readAccelerometerData() {
  if (lsm6dsl_ok) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    lsm6ds.getEvent(&accel, &gyro, &temp);

    // Convert from m/s^2 to g's
    currentData.accelX = accel.acceleration.x / SENSORS_GRAVITY_STANDARD;
    currentData.accelY = accel.acceleration.y / SENSORS_GRAVITY_STANDARD;
    currentData.accelZ = accel.acceleration.z / SENSORS_GRAVITY_STANDARD;
  } else {
    currentData.accelX = currentData.accelY = currentData.accelZ = 0.0;
  }
}

void readGPSLocation() {
  Serial.println("Reading GPS location...");
  currentData.usingCGPS = true; // Mark that CGPS method is in use
  
  // Enable GPS using the CGPS command set (more reliable on SIM7600).
  // Reset GPS first then configure GNSS before starting.
  sendGPSCommand("AT+CGPS=0");
  delay(500);
  sendGPSCommand("AT+CGNSSMODE=15,1");
  delay(2000);                      
  sendGPSCommand("AT+CGPSNMEA=200191");
  delay(2000);                      
  sendGPSCommand("AT+CGPSNMEARATE=1");
  delay(2000);                      
  sendGPSCommand("AT+CGPS=1,1");   // Start GPS in standalone mode
  delay(2000);                      // Allow the receiver to power-up

  // Enable unsolicited CGPSINFO while we wait so we can observe sentences
  sendAT("AT+CGPSINFOCFG=1,31", 2000);
  Serial.println("\nWaiting 10 seconds for GPS to get signal...");
  
  // Check for interrupts during 10 second GPS wait
  for (int i = 0; i < 100; i++) {
    if (shouldInterruptOperation()) {
      Serial.println("💤 GPS acquisition aborted - entering deep sleep to handle interrupt...");
      digitalWrite(LED_INDICATOR_PIN, LOW);
      prepareForDeepSleep();
      esp_deep_sleep_start();
    }
    delay(100);
  }
  
  sendAT("AT+CGPSINFOCFG=0,31", 2000);
  // Power-mode and NMEA configuration diagnostics
  sendAT("AT+CGPSPMD?", 2000);
  sendAT("AT+CGPSNMEA?", 2000);

  // Request current location data
  flushSIM7600Buffer();

  sim7600.println("AT+CGNSSINFO"); // Query both for robustness
  
  // Check for interrupts during 2 second wait
  for (int i = 0; i < 20; i++) {
    if (shouldInterruptOperation()) {
      Serial.println("💤 GPS query aborted - entering deep sleep to handle interrupt...");
      digitalWrite(LED_INDICATOR_PIN, LOW);
      prepareForDeepSleep();
      esp_deep_sleep_start();
    }
    delay(100);
  }
                      
  sim7600.println("AT+CGPSINFO");
  String response = waitForGPSResponse(5000);

  if (response.indexOf("+CGNSSINFO:") != -1 && isValidGPSFix(response)) {
    // +CGPSINFO: <lat>,<N/S>,<lon>,<E/W>,<date>,<utc>,<alt>,<speed>,<course>
    String latitude      = extractGPSField(response, 1);
    String lat_direction = extractGPSField(response, 2);
    String longitude     = extractGPSField(response, 3);
    String lon_direction = extractGPSField(response, 4);
    String altitude      = extractGPSField(response, 7);

    currentData.latitude  = convertToDecimalDegrees(latitude,  lat_direction);
    currentData.longitude = convertToDecimalDegrees(longitude, lon_direction);
    currentData.altitude  = altitude.toFloat();
    currentData.gpsFixValid = true;
    currentData.coarseFix = false; // This is a high-accuracy fix
    Serial.println("✓ GPS fix acquired");
  } else {
    Serial.println("⚠ No GPS fix available after waiting.");
    currentData.gpsFixValid = false;
  }
}

// Helper function for compatibility with existing GPS code
void sendAT(const char *cmd, uint16_t delayMs) {
  sendATCommand(cmd, delayMs);
}

void readBatteryVoltage() {
  // Read battery voltage through ADC with voltage divider
  int adcReading = analogRead(BATTERY_ADC_PIN);
  currentData.batteryVoltage = (adcReading * 3.3 * 2.0) / 4095.0;
}

// ---------------------------------------------------------------------------
// Fast Cell-tower location (CLBS)
// ---------------------------------------------------------------------------
bool readCellLocation() {
  flushSIM7600Buffer();
  sim7600.println("AT+CLBS=1,1");
  String resp = waitForGPSResponse(5000);

  int idx = resp.indexOf("+CLBS:");
  if (idx == -1) return false;

  // Expect format: +CLBS: <err>,<lat>,<lon>,<date>,<time>
  int firstComma = resp.indexOf(',', idx);
  if (firstComma == -1) return false;
  int err = resp.substring(idx + 7, firstComma).toInt();
  if (err != 0) return false;

  int secondComma = resp.indexOf(',', firstComma + 1);
  if (secondComma == -1) return false;
  int thirdComma = resp.indexOf(',', secondComma + 1);
  if (thirdComma == -1) return false;

  String latStr = resp.substring(firstComma + 1, secondComma);
  String lonStr = resp.substring(secondComma + 1, thirdComma);

  double lat = latStr.toDouble();
  double lon = lonStr.toDouble();
  if (lat == 0.0 || lon == 0.0) return false;

  currentData.latitude = lat;
  currentData.longitude = lon;
  currentData.altitude = 0;
  currentData.gpsFixValid = false;   // not a GNSS fix
  currentData.coarseFix = true;

  Serial.printf("✓ CLBS coarse fix: %.5f, %.5f\n", lat, lon);
  return true;
}

// =====================================================================
// HARDWARE INITIALIZATION
// =====================================================================
bool initializeAllHardware() {
  pinMode(BUZZER_PIN, OUTPUT);
  // Restore buzzer state from RTC memory
  digitalWrite(BUZZER_PIN, rtcBuzzerActive ? HIGH : LOW);
  if (rtcBuzzerActive) {
    Serial.println("🔔 Restoring buzzer state: ON");
  }
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  
  pinMode(SOLENOID_PIN, OUTPUT); // Initialize solenoid pin
  digitalWrite(SOLENOID_PIN, LOW); // Ensure solenoid is off initially
  
  pinMode(LED_INDICATOR_PIN, OUTPUT); // Initialize LED indicator
  digitalWrite(LED_INDICATOR_PIN, HIGH); // Turn ON LED - device is awake
  Serial.println("💡 LED indicator ON - Device awake");

  Wire.begin(SHT30_SDA_PIN, SHT30_SCL_PIN);
  Wire.setClock(100000);
  #if ENABLE_SHT30_SENSOR
    if (sht30.begin(0x44)) {
      sht30_ok = true;
      Serial.println("✓ SHT30 initialized successfully");
    } else {
      sht30_ok = false;
      Serial.println("✗ SHT30 initialization failed");
    }
  #endif

  if (initLSM6DSL()) {
    lsm6dsl_ok = true;
    // Attach interrupt for motion detection
    pinMode(LSM6DSL_INT1_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(LSM6DSL_INT1_PIN), motionISR, RISING);
    Serial.println("✓ Motion interrupt attached to GPIO" + String(LSM6DSL_INT1_PIN));
  } else {
    lsm6dsl_ok = false;
    Serial.println("✗ LSM6DSL accelerometer initialization failed");
  }

  sim7600.begin(115200, SERIAL_8N1, SIM7600_RX_PIN, SIM7600_TX_PIN);
  delay(2000);
  flushSIM7600Buffer();

  // Auxiliary voltage setup removed – external active antenna no longer required.

  return true;
}

bool initLSM6DSL() {
  uint8_t addresses[] = {LSM6DSL_ADDR1, LSM6DSL_ADDR2};
  bool found = false;

  for (int i = 0; i < 2; i++) {
    if (lsm6ds.begin_I2C(addresses[i], &Wire)) {
      lsm6dsl_address = addresses[i];
      found = true;
      break;
    }
  }

  if (!found) {
    Serial.println("✗ Failed to find LSM6DSL chip");
    return false;
  }
  
  Serial.println("✓ LSM6DSL Found!");

  // Apply wake-on-motion configuration from reference sketch
  // CTRL1_XL: 104 Hz, 2g
  writeLSM6DSLRegister(LSM6DSL_CTRL1_XL, 0x40);
  // CTRL3_C: Enable block data update, register auto-increment
  writeLSM6DSLRegister(LSM6DSL_CTRL3_C, 0x44);
  // TAP_CFG: Enable interrupts
  writeLSM6DSLRegister(LSM6DSL_TAP_CFG, 0x80);
  // WAKE_UP_THS: Set motion threshold (16 * 31.25mg = 500mg)
  writeLSM6DSLRegister(LSM6DSL_WAKE_UP_THS, 0x10);
  // WAKE_UP_DUR: No minimum duration
  writeLSM6DSLRegister(LSM6DSL_WAKE_UP_DUR, 0x03);
  // MD1_CFG: Route wake-up interrupt to INT1
  writeLSM6DSLRegister(LSM6DSL_MD1_CFG, 0x20);

  delay(100); // Stabilise
  Serial.println("✓ LSM6DSL initialised (with wake-up interrupt)");

  Serial.println("Last Wakeup cause: " + currentData.wakeUpReason);

  return true;
}

void writeLSM6DSLRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(lsm6dsl_address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// =====================================================================
// FIREBASE COMMUNICATION via cellular data
// =====================================================================

// ---------------------------------------------------------------------------
void determineWakeUpReason() {
  esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
  switch(wakeup_cause) {
    case ESP_SLEEP_WAKEUP_TIMER:
      currentData.wakeUpReason = "TIMER DUE (15mns.)";
      break;
    case ESP_SLEEP_WAKEUP_EXT0:
      currentData.wakeUpReason = "MOTION DETECTED"; // Corrected label
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      currentData.wakeUpReason = "LOCK BREACH";  // final reason may be overwritten later if real lock-breach
      break;
    default:
      currentData.wakeUpReason = "FIRST BOOT";
      break;
    }
  Serial.print("Last Wakeup cause: ");
  Serial.println(currentData.wakeUpReason);
}

// =====================================================================
// SIM7600 & GPS HELPERS
// =====================================================================
void flushSIM7600Buffer() {
  while (sim7600.available()) {
    sim7600.read();
  }
}

void sendGPSCommand(const char* cmd) {
  flushSIM7600Buffer();
  sim7600.println(cmd);
}

String waitForGPSResponse(unsigned long timeout) {
  String response = "";
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    // Check for interrupts
    if (shouldInterruptOperation()) {
      Serial.println("💤 GPS response wait aborted - entering deep sleep to handle interrupt...");
      digitalWrite(LED_INDICATOR_PIN, LOW);
      prepareForDeepSleep();
      esp_deep_sleep_start();
    }
    if (sim7600.available()) {
      response += sim7600.readString();
    }
    delay(50); // Small delay to prevent tight loop
  }
  return response;
}

String getGPSInfoLine(String response) { // Renamed: only handles CGPSINFO now
  int start = response.indexOf("+CGPSINFO:");
  if (start == -1) {
    return "";
  }
  int end = response.indexOf('\r', start);
  if (end == -1) {
    end = response.length();
  }
  return response.substring(start, end);
}

bool isValidGPSFix(String response) {
  String infoLine = getGPSInfoLine(response);
  if (infoLine.length() == 0) return false;

  // For CGPSINFO, a valid fix is indicated by non-empty latitude field.
  int colonPos = infoLine.indexOf(':');
  if (colonPos == -1) return false;
  String data = infoLine.substring(colonPos + 1);
  data.trim();
  int firstComma = data.indexOf(',');
  if (firstComma == -1) return false;
  String latField = data.substring(0, firstComma);
  latField.trim();
  return latField.length() > 0;
}

String extractGPSField(String response, int index) {
  String infoLine = getGPSInfoLine(response);
  if (infoLine.length() == 0) return "";

  int dataStart = infoLine.indexOf(':');
  if (dataStart == -1) return "";
  String data = infoLine.substring(dataStart + 2); // Skip ": "

  int current_index = 1;
  int last_pos = 0;

  while(current_index < index) {
    last_pos = data.indexOf(',', last_pos);
    if (last_pos == -1) return "";
    last_pos++;
    current_index++;
  }

  int next_comma = data.indexOf(',', last_pos);
  if (next_comma == -1) {
    return data.substring(last_pos);
  } else {
    return data.substring(last_pos, next_comma);
  }
}

double convertToDecimalDegrees(String coordinate, String direction) {
  if (coordinate.length() == 0) {
    return 0.0;
  }
  double raw_val = coordinate.toDouble();
  int dd = int(raw_val / 100);
  double mm = raw_val - (dd * 100);
  double dec_deg = dd + mm / 60.0;
  if (direction == "S" || direction == "W") {
    dec_deg = -dec_deg;
  }
  return dec_deg;
}

// =====================================================================
// DEEP SLEEP CONFIGURATION
// =====================================================================
void prepareForDeepSleep() {
  Serial.println("Configuring deep sleep triggers...");

  // No cleanup needed for Firebase mode

  // Wake up on timer
  esp_sleep_enable_timer_wakeup(SLEEP_TIME_US);

  // Wake up on motion (LSM6DSL INT1 is on GPIO34)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 1); // RTC_GPIO 0, wake on HIGH

  // Wake up on limit switch (lid open - pin is LOW when open)
  uint64_t limitSwitchMask = 1ULL << LIMIT_SWITCH_PIN;
  esp_sleep_enable_ext1_wakeup(limitSwitchMask, ESP_EXT1_WAKEUP_ANY_HIGH); // should be ALL_LOW if switch pulls to gnd when open
  
  // Isolate GPIO12 pin from external circuits during deep sleep to prevent flash issues.
  rtc_gpio_isolate(GPIO_NUM_12);
}


// Function to draw barcode placeholder
void drawBarcode(int xPos, int yPos, int width, int height) {
    // Draw barcode background
    Paint_DrawRectangle(xPos, yPos, xPos + width, yPos + height, 
                       EPD_7IN3F_WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    
    // Draw vertical bars pattern
    int barWidth = 2;
    int currentX = xPos + 10;
    
    // Simple pattern of bars
    for (int i = 0; i < 80 && currentX < (xPos + width - 10); i++) {
        if (i % 3 == 0 || i % 5 == 0) {
            int thisBarWidth = (i % 7 == 0) ? barWidth * 2 : barWidth;
            Paint_DrawRectangle(currentX, yPos, currentX + thisBarWidth, yPos + height, 
                               EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
            currentX += thisBarWidth + 1;
        } else {
            currentX += barWidth;
        }
    }
}

// =====================================================================
// E-PAPER DISPLAY – Shipping Label Format with Real Data
// =====================================================================
void updateDisplay() {
  Serial.println("Updating E-Ink display with shipping label format...");

  // Basic initialisation
  DEV_Module_Init();
  EPD_7IN3F_Init();
  EPD_7IN3F_Clear(EPD_7IN3F_WHITE);

  // Allocate full frame buffer for complete display
  UBYTE *imgBuf;
  UDOUBLE imgSize = ((EPD_7IN3F_WIDTH % 2 == 0) ? (EPD_7IN3F_WIDTH / 2) : (EPD_7IN3F_WIDTH / 2 + 1)) * EPD_7IN3F_HEIGHT;
  imgBuf = (UBYTE *)malloc(imgSize);
  if (!imgBuf) {
    Serial.println("✗ Failed to allocate display buffer");
    return;
  }

  Paint_NewImage(imgBuf, EPD_7IN3F_WIDTH, EPD_7IN3F_HEIGHT, 0, EPD_7IN3F_WHITE);
  Paint_SetScale(7);          // 7-colour mode
  Paint_SelectImage(imgBuf);
  Paint_Clear(EPD_7IN3F_WHITE);

  // Variables for layout
  const uint16_t leftMargin = 40;
  const uint16_t rightColumnX = 480;
  uint16_t y = 25;
  char buf[128];

  // ========== SENDER INFORMATION ==========
  // Use "Not Provided" if field is empty
  String senderName = currentData.senderName.length() > 0 ? currentData.senderName : "Not Provided";
  String senderAddr = currentData.senderAddress.length() > 0 ? currentData.senderAddress : "Not Provided";
  
  Paint_DrawString_EN(leftMargin, y, senderName.c_str(), &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  y += 25;
  
  if (senderAddr.length() > 60) senderAddr = senderAddr.substring(0, 57) + "...";
  Paint_DrawString_EN(leftMargin, y, senderAddr.c_str(), &Font12, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  y += 30;
  
  // Horizontal divider
  Paint_DrawLine(30, y, EPD_7IN3F_WIDTH - 30, y, EPD_7IN3F_BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
  y += 20;
  
  // ========== RECIPIENT INFORMATION ==========
  Paint_DrawString_EN(leftMargin, y, "Ship To:", &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  y += 30;
  
  String recipName = currentData.recipientName.length() > 0 ? currentData.recipientName : "Not Provided";
  String recipAddr = currentData.recipientAddress.length() > 0 ? currentData.recipientAddress : "Not Provided";
  
  Paint_DrawString_EN(leftMargin, y, recipName.c_str(), &Font24, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  y += 35;
  
  if (recipAddr.length() > 60) recipAddr = recipAddr.substring(0, 57) + "...";
  Paint_DrawString_EN(leftMargin, y, recipAddr.c_str(), &Font20, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  y += 35;
  
  // ========== PACKAGE WEIGHT ==========
  String weight = currentData.packWeight.length() > 0 ? currentData.packWeight : "Not Provided";
  Paint_DrawString_EN(leftMargin, y, weight.c_str(), &Font24, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  y += 40;
  
  // ========== MAXICODE AND ROUTING INFO ==========
  // MAXICODE placeholder
  Paint_DrawRectangle(leftMargin, y, leftMargin + 70, y + 70, EPD_7IN3F_BLACK, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
  Paint_DrawString_EN(leftMargin + 15, y + 25, "MAXI", &Font12, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  Paint_DrawString_EN(leftMargin + 15, y + 45, "CODE", &Font12, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  
  // Routing Code and Postal Code
  String routing = currentData.routingCode.length() > 0 ? currentData.routingCode : "Not Provided";
  String postal = currentData.postalCode.length() > 0 ? currentData.postalCode : "Not Provided";
  
  Paint_DrawString_EN(leftMargin + 90, y + 10, routing.c_str(), &Font20, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  Paint_DrawString_EN(leftMargin + 90, y + 40, postal.c_str(), &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);

  // ========== RIGHT COLUMN - SERVICE TYPE AND QR CODE ==========
  uint16_t rightY = 85;
  
  // Service Type
  String serviceType = currentData.serviceType.length() > 0 ? currentData.serviceType : "STANDARD";
  Paint_DrawRectangle(rightColumnX, rightY, rightColumnX + 100, rightY + 40, EPD_7IN3F_BLACK, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
  Paint_DrawString_EN(rightColumnX + 15, rightY + 10, serviceType.c_str(), &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  
  // SERVICE label
  Paint_DrawRectangle(rightColumnX + 110, rightY, rightColumnX + 230, rightY + 40, EPD_7IN3F_BLACK, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
  Paint_DrawString_EN(rightColumnX + 125, rightY + 10, "SERVICE", &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  
  // QR Code below service boxes
  rightY += 60;
  String qrUrl = "https://tracking-box.vercel.app/qr/" + actualDeviceID;
  
  // Generate QR code
  uint8_t qrcodeData[qrcode_getBufferSize(3)];
  QRCode qrcode;
  qrcode_initText(&qrcode, qrcodeData, 3, ECC_LOW, qrUrl.c_str());
  
  // Draw QR code
  const int scale = 3;
  const int qrSize = qrcode.size;
  const int qrOffsetX = rightColumnX + 60;
  const int qrOffsetY = rightY;
  
  for (int qy = 0; qy < qrSize; qy++) {
    for (int qx = 0; qx < qrSize; qx++) {
      if (qrcode_getModule(&qrcode, qx, qy)) {
        Paint_DrawRectangle(qrOffsetX + qx * scale,
                            qrOffsetY + qy * scale,
                            qrOffsetX + (qx + 1) * scale,
                            qrOffsetY + (qy + 1) * scale,
                            EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
      }
    }
  }
  
  // ========== TRACKING NUMBER ==========
  y = 320;
  Paint_DrawLine(30, y, EPD_7IN3F_WIDTH - 30, y, EPD_7IN3F_BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
  y += 20;
  
  String tracking = currentData.trackingNumber.length() > 0 ? currentData.trackingNumber : "Not Provided";
  int textWidth = tracking.length() * 14;
  int centerX = (EPD_7IN3F_WIDTH - textWidth) / 2;
  Paint_DrawString_EN(centerX, y, tracking.c_str(), &Font24, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  y += 40;
  
  // ========== BARCODE ==========
  drawBarcode(40, y, EPD_7IN3F_WIDTH - 80, 50);
  y += 55;
  
  // Barcode numbers
  Paint_DrawString_EN(50, y, "0522", &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  Paint_DrawString_EN(EPD_7IN3F_WIDTH - 100, y, "2077", &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  
  // ========== BOTTOM INFO ==========
  y += 30;
  // Show reference code at bottom
  snprintf(buf, sizeof(buf), "REF: %s", currentData.referenceCode.c_str());
  Paint_DrawString_EN(leftMargin, y, buf, &Font12, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  
  // Show current GPS location (small font)
  snprintf(buf, sizeof(buf), "GPS: %.6f, %.6f", currentData.latitude, currentData.longitude);
  Paint_DrawString_EN(rightColumnX, y, buf, &Font8, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);

  // ------------------------------
  // Push buffer to display
  // ------------------------------
  EPD_7IN3F_Display(imgBuf);
  EPD_7IN3F_Sleep();
  free(imgBuf);
  imgBuf = nullptr;

  Serial.println("✓ Display updated with combined layout");
}

// ---------------------------------------------------------------------------
// OFFLINE PAGE – Shows QR code when offline
// ---------------------------------------------------------------------------
void showOfflineQRCode() {
  // Show the display with QR code for offline access
  updateDisplay();
}

// =====================================================================
// END OF TRACKING BOX MAIN FIRMWARE
// ===================================================================== 

// Direct Firebase communication implemented

// ---------------------------------------------------------------------------
// CELLULAR LOCATION SERVICES (CLBS) - GPS FALLBACK
// ---------------------------------------------------------------------------
// Note: Using cellular data transmission functions for Firebase communication
// Cellular location services (CLBS) retained for GPS fallback functionality

// =====================================================================
// FIREBASE COMMUNICATION FUNCTIONS
// =====================================================================
// These functions implement direct Firebase communication via cellular data.
// The device sends sensor data directly to Firebase Realtime Database.
// =====================================================================

// Initialize cellular data connection
bool initializeCellularData() {
  Serial.println("\n=== INITIALIZING CELLULAR DATA ===");
  
  // Basic initialization sequence
  sendATCommand("AT", 2000);                                    // Test communication
  sendATCommand("AT+CFUN=1", 5000);                            // Set full functionality
  sendATCommand("AT+CPIN?", 2000);                             // Check SIM status
  sendATCommand("AT+CREG?", 2000);                             // Check network registration
  sendATCommand("AT+CGATT=1", 10000);                          // Attach to GPRS service
  
  // Configure SSL/TLS for HTTPS
  sendATCommand("AT+CSSLCFG=\"sslversion\",0,3", 2000);        // Set TLS 1.2
  sendATCommand("AT+CSSLCFG=\"authmode\",0,0", 2000);          // Disable cert verification
  sendATCommand("AT+CSSLCFG=\"ignorelocaltime\",0,1", 2000);   // Ignore RTC time
  
  // Configure PDP context
  String apnCmd = "AT+CGDCONT=1,\"IP\",\"" + String(APN) + "\"";
  sim7600.println(apnCmd);
  delay(2000);
  sendATCommand("AT+CGACT=1,1", 10000);                        // Activate PDP context
  
  // Check signal strength
  sendATCommand("AT+CSQ", 2000);                               // Signal quality
  
  Serial.println("✅ Cellular data initialization complete!");
  return true;
}

bool sendSensorDataToFirebase() {
  Serial.println("Attempting to send sensor data to Firebase...");
  
  // Create JSON payload for Firebase
  String jsonData = "{";
  jsonData += "\"temp\":" + String(currentData.temperature, 1) + ",";
  jsonData += "\"humidity\":" + String(currentData.humidity, 1) + ",";
  jsonData += "\"currentLocation\":\"" + String(currentData.latitude, 6) + "," + String(currentData.longitude, 6) + "\",";
  jsonData += "\"altitude\":" + String(currentData.altitude, 1) + ",";
  jsonData += "\"tilt\":" + String(currentData.tiltDetected ? "true" : "false") + ",";
  jsonData += "\"fall\":" + String(currentData.fallDetected ? "true" : "false") + ",";
  jsonData += "\"limitSwitch\":" + String(currentData.limitSwitchPressed ? "true" : "false") + ",";
  jsonData += "\"solenoid\":" + String(currentData.solenoidActive ? "true" : "false") + ",";
  jsonData += "\"accelerometer\":{";
  jsonData += "\"x\":" + String(currentData.accelX, 3) + ",";
  jsonData += "\"y\":" + String(currentData.accelY, 3) + ",";
  jsonData += "\"z\":" + String(currentData.accelZ, 3) + ",";
  jsonData += "\"tiltDetected\":" + String(currentData.tiltDetected ? "true" : "false");
  jsonData += "},";
  jsonData += "\"batteryVoltage\":" + String(currentData.batteryVoltage, 2) + ",";
  jsonData += "\"wakeUpReason\":\"" + currentData.wakeUpReason + "\",";
  jsonData += "\"timestamp\":" + String(millis()) + ",";
  jsonData += "\"bootCount\":" + String(currentData.bootCount) + ",";
  jsonData += "\"referenceCode\":\"" + currentData.referenceCode + "\",";
  jsonData += "\"securityBreachActive\":" + String(currentData.securityBreachActive ? "true" : "false");
  jsonData += "}";
  
  // Send to Firebase
  String path = "/tracking_box/" + actualDeviceID + "/sensorData";
  bool success = sendFirebaseHTTP(path, jsonData, "PUT");
  
  if (success) {
    Serial.println("✅ Data sent to Firebase successfully");
    rtcLastFirebaseUpdate = millis();
    
    // Check for control commands after sending data
    delay(2000);
    checkFirebaseControls();
  }
  
  return success;
}

// Send data to Firebase via HTTP
bool sendFirebaseHTTP(String path, String jsonData, String method) {
  Serial.println("\n=== SENDING TO FIREBASE ===");
  Serial.println("Path: " + path);
  Serial.println("Method: " + method);
  Serial.println("JSON Length: " + String(jsonData.length()));
  
  // HTTP sequence for Firebase
  sendATCommand("AT+HTTPTERM", 1000);
  sendATCommand("AT+HTTPINIT", 2000);
  
  // Set URL
  String url = String(FIREBASE_URL) + path + ".json";
  String urlCmd = "AT+HTTPPARA=\"URL\",\"" + url + "\"";
  sim7600.println(urlCmd);
  delay(2000);
  
  // Set content type
  sendATCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 1000);
  
  // Send data
  String dataCmd = "AT+HTTPDATA=" + String(jsonData.length()) + ",10000";
  Serial.println("Sending: " + dataCmd);
  sim7600.println(dataCmd);
  
  // Wait for DOWNLOAD prompt
  delay(1000);
  
  // Send JSON data byte by byte
  for (int i = 0; i < jsonData.length(); i++) {
    sim7600.write(jsonData[i]);
    delayMicroseconds(100);
  }
  
  delay(1000);
  
  // Execute HTTP action (0=GET, 1=PUT, 2=POST)
  int action = (method == "GET") ? 0 : (method == "PUT") ? 1 : 2;
  String actionCmd = "AT+HTTPACTION=" + String(action);
  sendATCommand(actionCmd.c_str(), 10000);
  
  // Check for response
  delay(2000);
  String response = sendATCommandResponse("AT+HTTPREAD=0,500", 3000);
  
  bool success = (response.indexOf("200") != -1 || response.indexOf("OK") != -1);
  
  if (success) {
    Serial.println("✅ Firebase request successful");
  } else {
    Serial.println("❌ Firebase request failed");
  }
  
  sendATCommand("AT+HTTPTERM", 1000);
  return success;
}

// Read data from Firebase
String readFirebaseHTTP(String path) {
  Serial.println("\n=== READING FROM FIREBASE ===");
  Serial.println("Path: " + path);
  
  sendATCommand("AT+HTTPTERM", 1000);
  sendATCommand("AT+HTTPINIT", 2000);
  
  // Set URL
  String url = String(FIREBASE_URL) + path + ".json";
  String urlCmd = "AT+HTTPPARA=\"URL\",\"" + url + "\"";
  sim7600.println(urlCmd);
  delay(2000);
  
  // Execute GET request
  sendATCommand("AT+HTTPACTION=0", 5000);
  
  // Read response
  delay(3000);
  String response = sendATCommandResponse("AT+HTTPREAD=0,1000", 3000);
  
  // Extract JSON from response
  int jsonStart = response.indexOf('{');
  int jsonEnd = response.lastIndexOf('}');
  
  if (jsonStart != -1 && jsonEnd != -1) {
    response = response.substring(jsonStart, jsonEnd + 1);
  } else {
    response = "";
  }
  
  sendATCommand("AT+HTTPTERM", 1000);
  return response;
}

// Send AT command with response
String sendATCommandResponse(const char* cmd, int timeout) {
  Serial.println("Sending: " + String(cmd));
  flushSIM7600Buffer();
  sim7600.println(cmd);
  
  String response = "";
  unsigned long startTime = millis();
  
  while (millis() - startTime < timeout) {
    if (sim7600.available()) {
      response += sim7600.readString();
    }
    delay(10);
  }
  
  if (response.length() > 0) {
    Serial.print("Response: " + response);
  }
  
  return response;
}


// =====================================================================
// FIREBASE CONTROL FUNCTIONS
// =====================================================================
bool checkFirebaseControls() {
  Serial.println("Checking Firebase for control commands...");
  
  // Read control flags from Firebase
  String controlPath = "/tracking_box/" + actualDeviceID + "/controlFlags";
  String controlData = readFirebaseHTTP(controlPath);
  
  if (controlData.length() > 0) {
    Serial.println("Control data received: " + controlData);
    parseFirebaseControls(controlData);
    
    // Also read device details
    String detailsPath = "/tracking_box/" + actualDeviceID + "/details";
    String detailsData = readFirebaseHTTP(detailsPath);
    
    if (detailsData.length() > 0) {
      parseFirebaseDetails(detailsData);
    }
    
    return true;
  }
  
  return false;
}

void parseFirebaseDetails(String jsonData) {
  // Parse device details from Firebase
  // Format: {"name":"...","setLocation":"...","description":"..."}
  
  int nameStart = jsonData.indexOf("\"name\":\"") + 8;
  if (nameStart > 7) {
    int nameEnd = jsonData.indexOf("\"", nameStart);
    if (nameEnd != -1) {
      currentData.deviceName = jsonData.substring(nameStart, nameEnd);
      strncpy(rtcDeviceName, currentData.deviceName.c_str(), sizeof(rtcDeviceName) - 1);
      rtcDeviceDetailsValid = true;
      Serial.println("✅ Updated device name: " + currentData.deviceName);
    }
  }
  
  int locStart = jsonData.indexOf("\"setLocation\":\"") + 15;
  if (locStart > 14) {
    int locEnd = jsonData.indexOf("\"", locStart);
    if (locEnd != -1) {
      currentData.deviceSetLocation = jsonData.substring(locStart, locEnd);
      strncpy(rtcDeviceSetLocation, currentData.deviceSetLocation.c_str(), sizeof(rtcDeviceSetLocation) - 1);
      rtcDeviceDetailsValid = true;
      Serial.println("✅ Updated setLocation: " + currentData.deviceSetLocation);
    }
  }
}

void parseFirebaseControls(String jsonData) {
  // Parse control flags from Firebase
  // Format: {"buzzer":true,"solenoid":false,"dismissed":true}
  
  if (jsonData.length() == 0) return;
  
  // Parse buzzer state
  bool newBuzzerState = false;
  if (jsonData.indexOf("\"buzzer\":true") != -1) {
    newBuzzerState = true;
  }
  
  // Parse solenoid state
  bool newSolenoidState = false;
  if (jsonData.indexOf("\"solenoid\":true") != -1) {
    newSolenoidState = true;
  }
  
  // Parse dismiss state
  bool newDismissState = false;
  if (jsonData.indexOf("\"dismissed\":true") != -1) {
    newDismissState = true;
  }
  
  // Parse clear breach state
  bool clearBreach = false;
  if (jsonData.indexOf("\"clearBreach\":true") != -1) {
    clearBreach = true;
  }
  
  Serial.println("✅ CONTROL COMMAND RECEIVED - Applying immediately...");
  Serial.printf("Control states: Buzzer=%d, Solenoid=%d, Dismiss=%d, ClearBreach=%d\n", 
                newBuzzerState, newSolenoidState, newDismissState, clearBreach);
  
  // Update dismiss state
  currentData.buzzerDismissed = newDismissState;
  rtcBuzzerDismissed = newDismissState;
  
  if (newDismissState) {
    Serial.println("✅ Buzzer dismissed by user");
  }
  
  // Clear security breach if Firebase instructs us to
  // This happens when lid is closed AND device is back in safe zone
  if (clearBreach && rtcSecurityBreachDetected) {
    Serial.println("🔓 CLEARING SECURITY BREACH - Device is secured and in safe zone");
    rtcSecurityBreachDetected = false;
    currentData.securityBreachActive = false;
  }
  
  // Apply buzzer state from Firebase
  currentData.buzzerIsActive = newBuzzerState;
  rtcBuzzerActive = newBuzzerState;
  digitalWrite(BUZZER_PIN, newBuzzerState ? HIGH : LOW);
  
  if (newBuzzerState) {
    Serial.println("🔔 BUZZER ACTIVATED by Firebase command!");
    Serial.println("Buzzer pin " + String(BUZZER_PIN) + " set to HIGH");
  } else {
    Serial.println("🔕 Buzzer turned OFF by Firebase command");
    Serial.println("Buzzer pin " + String(BUZZER_PIN) + " set to LOW");
  }
  
  // Apply solenoid state from Firebase
  if (newSolenoidState != currentData.solenoidActive) {
    currentData.solenoidActive = newSolenoidState;
    rtcSolenoidActive = newSolenoidState;
    
    if (newSolenoidState) {
      // Starting new solenoid activation
      rtcSolenoidStartTime = 0; // Will be set in the monitoring loop
      Serial.println("🔓 Solenoid activation requested by Firebase");
    } else {
      // Solenoid deactivation
      digitalWrite(SOLENOID_PIN, LOW);
      rtcSolenoidStartTime = 0;
      Serial.println("🔒 Solenoid deactivated by Firebase");
    }
  }
  
  Serial.println("Control states applied:");
  Serial.println("  Buzzer: " + String(newBuzzerState ? "ON" : "OFF"));
  Serial.println("  Solenoid: " + String(newSolenoidState ? "ON" : "OFF"));
  Serial.println("  Dismiss: " + String(newDismissState ? "DISMISSED" : "NOT DISMISSED"));
  Serial.println("=================================");
}

// =====================================================================
// UTILITY FUNCTIONS
// =====================================================================

// Generate a unique 10-character reference code
void generateReferenceCode() {
  const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
  const int charsetSize = 62;  // 26 uppercase + 26 lowercase + 10 digits
  
  // Initialize ESP32 hardware random number generator
  randomSeed(esp_random());
  
  for (int i = 0; i < 10; i++) {
    rtcReferenceCode[i] = charset[random(0, charsetSize)];
  }
  rtcReferenceCode[10] = '\0'; // Null terminator
  
  rtcReferenceCodeGenerated = true;
  
  Serial.println("✓ Generated unique reference code: " + String(rtcReferenceCode));
}

// =====================================================================
// DEVICE ID VALIDATION FUNCTIONS
// =====================================================================

// Check if a device ID already exists in Firebase
bool checkDeviceIDExists(String deviceID) {
  Serial.println("Checking if ID exists in Firebase: " + deviceID);
  
  // Check if any data exists under /tracking_box/{deviceID}
  String path = "/tracking_box/" + deviceID;
  String response = readFirebaseHTTP(path);
  
  // Firebase returns "null" for non-existent paths
  if (response.length() == 0 || response.indexOf("null") != -1) {
    return false;  // ID doesn't exist
  }
  
  // If response contains any JSON data, ID exists
  if (response.indexOf("{") != -1) {
    return true;  // ID exists with data
  }
  
  return false;  // Default to not exists
}

// Generate the next sequential device ID
String generateNextDeviceID(String currentID) {
  // Extract base and number from ID like "box_001" or "device_99"
  int lastUnderscore = currentID.lastIndexOf('_');
  
  if (lastUnderscore == -1) {
    // No underscore found, add "_002"
    return currentID + "_002";
  }
  
  String base = currentID.substring(0, lastUnderscore + 1);
  String numStr = currentID.substring(lastUnderscore + 1);
  
  // Check if the part after underscore is a number
  bool isNumber = true;
  for (unsigned int i = 0; i < numStr.length(); i++) {
    if (!isDigit(numStr[i])) {
      isNumber = false;
      break;
    }
  }
  
  if (!isNumber) {
    // Not a number, append "_002"
    return currentID + "_002";
  }
  
  // Parse and increment the number
  int num = numStr.toInt();
  num++;
  
  // Format with leading zeros (maintain original length)
  String newNumStr = String(num);
  while (newNumStr.length() < numStr.length()) {
    newNumStr = "0" + newNumStr;
  }
  
  return base + newNumStr;
}

// Validate and get a unique device ID
String validateAndGetUniqueDeviceID() {
  String testID = DEVICE_ID;
  int attempts = 0;
  const int MAX_ATTEMPTS = 100;  // Prevent infinite loop
  
  Serial.println("\n🔍 Starting Device ID validation...");
  
  while (attempts < MAX_ATTEMPTS) {
    Serial.println("Testing ID: " + testID);
    
    if (!checkDeviceIDExists(testID)) {
      Serial.println("✅ ID is available: " + testID);
      return testID;
    }
    
    Serial.println("❌ ID already exists: " + testID);
    testID = generateNextDeviceID(testID);
    attempts++;
    delay(500);  // Small delay between checks
  }
  
  // Fallback: use timestamp suffix if all attempts fail
  String fallbackID = DEVICE_ID + "_" + String(millis());
  Serial.println("⚠️ Max attempts reached, using fallback ID: " + fallbackID);
  return fallbackID;
}