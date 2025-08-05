/*
 * =====================================================================
 * TRACKING BOX DEVICE - SMS-ONLY FIRMWARE
 * =====================================================================
 * 
 * This sketch implements a streamlined, single-cycle operation for the
 * tracking device using SMS communication exclusively.
 * 
 * On every wake-up, it performs the following:
 * 1. Initialize all hardware.
 * 2. Gather a full set of sensor readings.
 * 3. Send sensor data via SMS to Master device.
 * 4. Check for control commands from Master via SMS.
 * 5. Update the E-Ink display with all data.
 * 6. Enter deep sleep until next wake trigger.
 * 
 * SMS COMMUNICATION:
 * The device operates as a "Slave" and sends compiled sensor data via
 * SMS to a Master device for Firebase forwarding.
 * SMS format: comma-separated values matching Firebase schema.
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
// DEVICE & SMS CONFIGURATION
// =====================================================================
const String DEVICE_ID = "box_001";

// SMS CONFIGURATION
// Configure the phone number of the Master device that will receive SMS
// and forward data to Firebase
const String MASTER_PHONE_NUMBER = "+639184652918"; // SMS Master device phone number

// =====================================================================
// GLOBAL OBJECTS & VARIABLES
// =====================================================================
volatile bool limitSwitchTriggered = false;  // Flag for interrupt handler
#if ENABLE_SHT30_SENSOR
Adafruit_SHT31 sht30 = Adafruit_SHT31();
#endif
Adafruit_LSM6DSL lsm6ds = Adafruit_LSM6DSL();
HardwareSerial sim7600(1);
Preferences preferences;  // For permanent storage
// Note: TinyGSM objects removed as cellular data transmission is no longer used
// SIM7600 module is still used for GNSS/GPS and CLBS location services
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

// RTC memory to store device details for SMS mode
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

// RTC memory for SMS cleanup timing
RTC_DATA_ATTR uint32_t rtcSMSCleanupCounter = 0;

// RTC memory for unique reference code
RTC_DATA_ATTR char rtcReferenceCode[11] = ""; // 10 chars + null terminator
RTC_DATA_ATTR bool rtcReferenceCodeGenerated = false;

// SMS cleanup timing removed - now handled based on storage usage before deep sleep

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
};

TrackerData currentData;

// Flag to request a restart after solenoid operation completes
bool restartAfterSolenoid = false;

// =====================================================================
// INTERRUPT HANDLER
// =====================================================================
// Interrupt handler for limit switch - triggers immediate reset
void IRAM_ATTR limitSwitchISR() {
  limitSwitchTriggered = true;
  // Force immediate restart when limit switch is triggered
  ESP.restart();
}

// Forward declaration
void determineWakeUpReason();
void updateDisplay();
bool sendSensorDataViaSMS();
bool sendSMS(String phoneNumber, String message);
bool waitForSMSPrompt();
String formatSensorDataForSMS(const TrackerData &data);
void checkForControlSMS();
void parseControlSMS(String smsContent);
void cleanupSMSMemory();
bool isSMSMemoryNearlyFull();
void generateReferenceCode();

// =====================================================================
// MAIN SETUP (single cycle) – call new E-ink init just before display
// =====================================================================
void setup() {
  Serial.begin(115200);
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
  
  // SMS cleanup has been moved to before deep sleep to preserve control messages from Master
  
  // Check for control SMS from Master (skip on first boot - no data sent yet)
  if (rtcBootCount > 1) {
    Serial.println("📨 Checking for pending control SMS from Master...");
    delay(2000); // Small delay to ensure any pending SMS are received
    checkForControlSMS();
  } else {
    Serial.println("📨 Skipping SMS check on first boot");
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
  
  // Collect sensor data for SMS transmission
  collectSensorReading();
  Serial.println("✅ Sensor Readings Collected.");
  
  // In SMS mode, we don't evaluate lock breach - Master decides everything
  // Just collect sensor data and send to Master
  
  // Attempt to send data via SMS
  if (sendSensorDataViaSMS()) {
    Serial.println("✅ Sensor data sent via SMS to Master device.");
    
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
        
        // Handle solenoid (60-second activation)
        if (currentData.solenoidActive || rtcSolenoidActive) {
          solenoidRunTime = millis() - rtcSolenoidStartTime;
          
          if (solenoidRunTime >= 60000) { // 60 seconds (1 minute) elapsed
            Serial.println("🔒 Solenoid deactivating after 60 seconds");
            digitalWrite(SOLENOID_PIN, LOW);
            currentData.solenoidActive = false;
            rtcSolenoidActive = false;
            rtcSolenoidStartTime = 0;
            
            // Notify Master that solenoid cycle is complete
            // Master will update Firebase
          } else {
            // Keep solenoid active
            digitalWrite(SOLENOID_PIN, HIGH);
            Serial.println("🔓 Solenoid active for " + String(solenoidRunTime / 1000) + " seconds");
          }
        }
        
        // Check for new control commands
        if (millis() - lastCheck >= CHECK_INTERVAL) {
          lastCheck = millis();
          Serial.println("📨 Checking for control commands...");
          
          // Check for control SMS from Master
          checkForControlSMS();
          
          // If both are deactivated, exit loop
          if (!currentData.buzzerIsActive && !rtcBuzzerActive && 
              !currentData.solenoidActive && !rtcSolenoidActive) {
            Serial.println("✅ All controls deactivated - preparing for sleep");
            break;
          }
        }
        
        delay(100); // Small delay to prevent busy waiting
      }
    }
    
    // Update display with QR code
    showOfflineQRCode();
    
    Serial.println("Cycle complete → deep sleep (SMS mode).");
  } else {
    Serial.println("❌ SMS send failed. Operating in offline mode.");
    showOfflineQRCode();
    Serial.println("Cycle complete → deep sleep (offline mode).");
  }
  
  // CRITICAL: Set buzzer to correct state before deep sleep
  digitalWrite(BUZZER_PIN, rtcBuzzerActive ? HIGH : LOW);
  
  // Only sleep after buzzer is properly handled
  prepareForDeepSleep();
  esp_deep_sleep_start();
}

void loop() {
  // The loop is intentionally left empty.
  // The device performs a single cycle in setup() and then deep sleeps.
}

// Firebase functions removed - SMS-only mode

// ---------------------------------------------------------------------------
// SOLENOID CONTROL
// ---------------------------------------------------------------------------
// Solenoid state fetching removed - handled via SMS

// Solenoid activation removed - handled via SMS

// Lock breach evaluation removed - Master device handles all logic in SMS mode

// Buzzer monitoring removed - handled in main SMS loop

// Solenoid activation wait removed - handled by Master in SMS mode

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
  
  // Attach interrupt to limit switch for immediate wake/reset
  // Trigger on FALLING edge (HIGH to LOW) when lid is opened
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchISR, FALLING);
  
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
      // Note: Security breach will only be cleared by Master when both:
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
  delay(10 * 1000UL); // 30 seconds blocking to acquire fix
  sendAT("AT+CGPSINFOCFG=0,31", 2000);
  // Power-mode and NMEA configuration diagnostics
  sendAT("AT+CGPSPMD?", 2000);
  sendAT("AT+CGPSNMEA?", 2000);

  // Request current location data
  flushSIM7600Buffer();

  sim7600.println("AT+CGNSSINFO"); // Query both for robustness
  delay(2000);                      
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

// Helper to send an AT command and echo response for a given duration (ms)
void sendAT(const char *cmd, uint16_t delayMs) {
  Serial.print("\n>> "); Serial.println(cmd);
  flushSIM7600Buffer();
  sim7600.println(cmd);
  unsigned long timeout = millis() + delayMs;
  while (millis() < timeout) {
    while (sim7600.available()) {
      Serial.write(sim7600.read());
    }
  }
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
  // Attach interrupt for immediate wake/reset when limit switch is triggered
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchISR, FALLING);
  
  pinMode(SOLENOID_PIN, OUTPUT); // Initialize solenoid pin
  digitalWrite(SOLENOID_PIN, LOW); // Ensure solenoid is off initially

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
// SMS-ONLY COMMUNICATION - WiFi removed
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
    if (sim7600.available()) {
      response += sim7600.readString();
    }
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

  // Check if SMS memory is nearly full and clean up if needed
  if (isSMSMemoryNearlyFull()) {
    Serial.println("⚠️ SMS memory is nearly full (>80%), cleaning up before sleep...");
    cleanupSMSMemory();
  } else {
    Serial.println("✅ SMS memory has sufficient space, no cleanup needed");
  }

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


// =====================================================================
// E-PAPER DISPLAY – Waveshare 7.3" 800×400 (7-color) helper
// =====================================================================
void updateDisplay() {
  Serial.println("Updating E-Ink display with combined layout...");

  // Basic initialisation
  DEV_Module_Init();
  EPD_7IN3F_Init();
  EPD_7IN3F_Clear(EPD_7IN3F_WHITE);

  // Allocate a quarter-frame buffer (800×200)
  UBYTE *imgBuf;
  UDOUBLE imgSize = ((EPD_7IN3F_WIDTH % 2 == 0) ? (EPD_7IN3F_WIDTH / 2) : (EPD_7IN3F_WIDTH / 2 + 1)) * (EPD_7IN3F_HEIGHT / 2);
  imgBuf = (UBYTE *)malloc(imgSize);
  if (!imgBuf) {
    Serial.println("✗ Failed to allocate display buffer");
    return;
  }

  Paint_NewImage(imgBuf, EPD_7IN3F_WIDTH, EPD_7IN3F_HEIGHT / 2, 0, EPD_7IN3F_WHITE);
  Paint_SetScale(7);          // 7-colour mode
  Paint_SelectImage(imgBuf);
  Paint_Clear(EPD_7IN3F_WHITE);

  // ------------------------------
  // LEFT SIDE: Device Details (using smaller font)
  // ------------------------------
  uint16_t y = 5;  // Start Y-coord
  const uint16_t lineGap = 20;  // Reduced line gap for smaller font
  const uint16_t leftMargin = 10;
  const uint16_t leftWidth = 500;  // Use 500px for left side

  // Title
  Paint_DrawString_EN(leftMargin, y, "TRACKING DETAILS", &Font20, EPD_7IN3F_WHITE, EPD_7IN3F_RED);
  y += lineGap + 5;

  // Reference Code (in bold/larger font)
  char buf[64];
  snprintf(buf, sizeof(buf), "TRACKING REFERENCE CODE: %s", currentData.referenceCode.c_str());
  Paint_DrawString_EN(leftMargin, y, buf, &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLUE);
  y += lineGap + 5;

  // Owner
  snprintf(buf, sizeof(buf), "Owner: %s", currentData.deviceName.c_str());
  Paint_DrawString_EN(leftMargin, y, buf, &Font12, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  y += lineGap;

  // Current Location (truncate if too long)
  String currLoc = currentData.currentLocation;
  if (currLoc.length() > 35) currLoc = currLoc.substring(0, 32) + "...";
  snprintf(buf, sizeof(buf), "Current: %s", currLoc.c_str());
  Paint_DrawString_EN(leftMargin, y, buf, &Font12, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  y += lineGap;

  // Drop-off Location (truncate if too long)
  String dropLoc = currentData.deviceSetLocation;
  if (dropLoc.length() > 35) dropLoc = dropLoc.substring(0, 32) + "...";
  snprintf(buf, sizeof(buf), "Drop-off: %s", dropLoc.c_str());
  Paint_DrawString_EN(leftMargin, y, buf, &Font12, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  y += lineGap * 2;  // Extra space since we removed sensor data

  // Bottom message
  Paint_DrawString_EN(leftMargin, y, "PRIORITY MAIL - HANDLE WITH CARE", &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_RED);

  // ------------------------------
  // Draw vertical separator line
  // ------------------------------
  Paint_DrawLine(leftWidth, 0, leftWidth, 200, EPD_7IN3F_BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);

  // ------------------------------
  // RIGHT SIDE: QR Code
  // ------------------------------
  String dynamicUrl = "https://tracking-box.vercel.app/qr/" + DEVICE_ID + "/";
  const char *url = dynamicUrl.c_str();
  
  // Generate QR code
  uint8_t qrcodeData[qrcode_getBufferSize(3)];
  QRCode qrcode;
  qrcode_initText(&qrcode, qrcodeData, 3, ECC_LOW, url);

  // Position QR code in the right section
  const int scale = 5;  // Slightly smaller to fit better
  const int qrSize = qrcode.size;
  const int qrPix = qrSize * scale;
  const int rightSectionStart = leftWidth + 10;
  const int rightSectionWidth = EPD_7IN3F_WIDTH - rightSectionStart;
  const int qrOffsetX = rightSectionStart + (rightSectionWidth - qrPix) / 2;
  const int qrOffsetY = 20;

  // Draw QR code
  for (int y = 0; y < qrSize; y++) {
    for (int x = 0; x < qrSize; x++) {
      if (qrcode_getModule(&qrcode, x, y)) {
        Paint_DrawRectangle(qrOffsetX + x * scale,
                            qrOffsetY + y * scale,
                            qrOffsetX + (x + 1) * scale,
                            qrOffsetY + (y + 1) * scale,
                            EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
      }
    }
  }

  // QR code caption - centered under QR code
  // Calculate text positions for centering
  const char* scanText = "Scan for live tracking";
  const char* urlText = "tracking-box.vercel.app";
  
  // Approximate character widths: Font12 ~7px, Font8 ~5px
  int scanTextWidth = strlen(scanText) * 7;  // Font12 width estimation
  int urlTextWidth = strlen(urlText) * 5;    // Font8 width estimation
  
  // Center the text under the QR code
  int scanTextX = qrOffsetX + (qrPix - scanTextWidth) / 2;
  int urlTextX = qrOffsetX + (qrPix - urlTextWidth) / 2;
  
  Paint_DrawString_EN(scanTextX, qrOffsetY + qrPix + 10,
                      scanText, &Font12,
                      EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  
  Paint_DrawString_EN(urlTextX, qrOffsetY + qrPix + 30,
                      urlText, &Font8,
                      EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);

  // ------------------------------
  // Push buffer to display
  // ------------------------------
  EPD_7IN3F_DisplayPart(imgBuf, 0, 0, EPD_7IN3F_WIDTH, EPD_7IN3F_HEIGHT / 2);
  EPD_7IN3F_Sleep();
  free(imgBuf);
  imgBuf = nullptr;

  Serial.println("✓ Display updated with combined layout");
}

// ---------------------------------------------------------------------------
// OFFLINE PAGE – Now just redirects to the combined display
// ---------------------------------------------------------------------------
void showOfflineQRCode() {
  // Since we're always in SMS mode, we always show the combined display
  updateDisplay();
}

// =====================================================================
// END OF TRACKING BOX MAIN FIRMWARE
// ===================================================================== 

// Firebase functions removed - SMS-only mode

// ---------------------------------------------------------------------------
// CELLULAR LOCATION SERVICES (CLBS) - GPS FALLBACK
// ---------------------------------------------------------------------------
// Note: Cellular data transmission functions removed - using SMS fallback instead
// Cellular location services (CLBS) retained for GPS fallback functionality

// =====================================================================
// SMS COMMUNICATION FUNCTIONS
// =====================================================================
// These functions implement the SMS communication system.
// The device sends sensor data via SMS to a Master device that forwards it to Firebase.
// SMS Format: "DEVICE_ID,timestamp,temp,humidity,lat,lng,alt,tilt,fall,limitSwitch,solenoid,accelX,accelY,accelZ,batteryVoltage,wakeUpReason,referenceCode"
// =====================================================================
bool sendSensorDataViaSMS() {
  Serial.println("Attempting to send sensor data via SMS...");
  
  // SMS cleanup moved to before deep sleep to preserve control messages
  
  // Format and send sensor data matching MasterSMSToFirebase format
  String smsMessage = formatSensorDataForSMS(currentData);
  Serial.println("SMS Message: " + smsMessage);
  
  bool success = sendSMS(MASTER_PHONE_NUMBER, smsMessage);
  
  if (success) {
    // After sending, check for control commands from Master multiple times
    Serial.println("📱 Monitoring for control commands (3 checks, 5 seconds apart)...");
    
    // First check after 5 seconds
    Serial.println("\n[Check 1/3]");
    delay(5000);
    checkForControlSMS();
    
    // Second check after another 5 seconds
    Serial.println("\n[Check 2/3]");
    delay(5000);
    checkForControlSMS();
    
    // Third check after another 5 seconds
    Serial.println("\n[Check 3/3]");
    delay(5000);
    checkForControlSMS();
    
    Serial.println("\n✅ Control command monitoring complete");
  }
  
  return success;
}

bool sendSMS(String phoneNumber, String message) {
  Serial.println("Sending SMS to: " + phoneNumber);
  
  // Ensure SIM7600 is initialized
  flushSIM7600Buffer();
  
  // Set SMS mode
  sim7600.println("AT+CMGF=1");
  delay(500);
  
  // Set character set
  sim7600.println("AT+CSCS=\"GSM\"");
  delay(500);
  
  // Set SMS storage to SIM card
  sim7600.println("AT+CPMS=\"SM\",\"SM\",\"SM\"");
  delay(500);
  
  // Set recipient
  sim7600.print("AT+CMGS=\"");
  sim7600.print(phoneNumber);
  sim7600.println("\"");
  delay(1000);
  
  // Wait for prompt
  if (!waitForSMSPrompt()) {
    Serial.println("❌ Failed to get SMS prompt");
    return false;
  }
  
  // Send message content
  sim7600.print(message);
  delay(500);
  
  // Send Ctrl+Z to finish SMS
  sim7600.write(26);
  delay(5000);
  
  // Check response
  String response = "";
  unsigned long timeout = millis() + 15000; // 15 second timeout
  
  while (millis() < timeout) {
    if (sim7600.available()) {
      response += sim7600.readString();
    }
  }
  
  Serial.println("SMS Response: " + response);
  
  // Check if SMS was sent successfully
  bool success = (response.indexOf("OK") != -1 || response.indexOf("+CMGS:") != -1);
  
  if (success) {
    Serial.println("✓ SMS sent successfully");
  } else {
    Serial.println("❌ Failed to send SMS");
  }
  
  return success;
}

bool waitForSMSPrompt() {
  unsigned long timeout = millis() + 5000; // 5 second timeout
  String response = "";
  
  while (millis() < timeout) {
    if (sim7600.available()) {
      char c = sim7600.read();
      response += c;
      if (response.indexOf(">") != -1) {
        return true; // Found prompt
      }
    }
  }
  
  return false;
}

String formatSensorDataForSMS(const TrackerData &data) {
  // Format matching MasterSMSToFirebase expectations (now 17 fields):
  // DEVICE_ID,timestamp,temp,humidity,lat,lng,alt,tilt,fall,limitSwitch,solenoid,accelX,accelY,accelZ,batteryVoltage,wakeUpReason,referenceCode,securityBreach
  
  // Get current timestamp in milliseconds
  uint64_t timestamp = millis();
  
  String message = DEVICE_ID + ",";
  message += String(timestamp) + ",";
  message += String(data.temperature, 1) + ",";
  message += String(data.humidity, 1) + ",";
  message += String(data.latitude, 6) + ",";
  message += String(data.longitude, 6) + ",";
  message += String(data.altitude, 1) + ",";
  message += String(data.tiltDetected ? "1" : "0") + ",";
  message += String(data.fallDetected ? "1" : "0") + ",";
  message += String(data.limitSwitchPressed ? "1" : "0") + ",";
  message += String(data.solenoidActive ? "1" : "0") + ",";
  message += String(data.accelX, 3) + ",";
  message += String(data.accelY, 3) + ",";
  message += String(data.accelZ, 3) + ",";
  message += String(data.batteryVoltage, 2) + ",";
  message += data.wakeUpReason + ",";
  message += data.referenceCode + ",";
  message += String(data.securityBreachActive ? "1" : "0");
  
  return message;
}


// =====================================================================
// SMS CONTROL RECEIVING FUNCTIONS
// =====================================================================
void checkForControlSMS() {
  Serial.println("Checking for control SMS from Master...");
  
  // Configure to receive SMS
  sim7600.println("AT+CMGF=1");  // Text mode
  delay(500);
  sim7600.println("AT+CPMS=\"SM\",\"SM\",\"SM\"");  // Use SIM storage
  delay(500);
  
  // Try multiple times to catch all messages
  bool cmdFound = false;
  int attempts = 0;
  
  while (!cmdFound && attempts < 3) {
    attempts++;
    
    // List unread messages
    sim7600.println("AT+CMGL=\"REC UNREAD\"");
    delay(1000); // Reduced from 2000ms
    
    if (sim7600.available()) {
      String response = sim7600.readString();
      if (attempts == 1) {
        Serial.println("SMS Response: " + response);
      }
    
    // Look for SETNAME messages first
    int setnameIndex = response.indexOf("SETNAME,");
    if (setnameIndex != -1) {
      // Extract the SETNAME message
      int endIndex = response.indexOf('\r', setnameIndex);
      if (endIndex == -1) endIndex = response.indexOf('\n', setnameIndex);
      if (endIndex == -1) endIndex = response.length();
      
      String setnameMsg = response.substring(setnameIndex, endIndex);
      Serial.println("SETNAME message received: " + setnameMsg);
      
      // Parse SETNAME,deviceId,name
      if (setnameMsg.startsWith("SETNAME,")) {
        setnameMsg = setnameMsg.substring(8); // Remove "SETNAME,"
        int comma1 = setnameMsg.indexOf(',');
        
        if (comma1 != -1) {
          String deviceId = setnameMsg.substring(0, comma1);
          String name = setnameMsg.substring(comma1 + 1);
          
          // Update device name in memory
          currentData.deviceName = name;
          strncpy(rtcDeviceName, currentData.deviceName.c_str(), sizeof(rtcDeviceName) - 1);
          rtcDeviceDetailsValid = true;
          
          Serial.println("✅ Updated device name: " + currentData.deviceName);
        }
      }
      
      // Find and delete this SMS
      int msgIndexStart = response.lastIndexOf("+CMGL: ", setnameIndex);
      if (msgIndexStart != -1) {
        int msgIndexEnd = response.indexOf(",", msgIndexStart + 7);
        int smsIndex = response.substring(msgIndexStart + 7, msgIndexEnd).toInt();
        sim7600.print("AT+CMGD=");
        sim7600.println(smsIndex);
        delay(500);
      }
    }
    
    // Look for SETLOC messages
    int setlocIndex = response.indexOf("SETLOC,");
    if (setlocIndex != -1) {
      // Extract the SETLOC message
      int endIndex = response.indexOf('\r', setlocIndex);
      if (endIndex == -1) endIndex = response.indexOf('\n', setlocIndex);
      if (endIndex == -1) endIndex = response.length();
      
      String setlocMsg = response.substring(setlocIndex, endIndex);
      Serial.println("SETLOC message received: " + setlocMsg);
      
      // Parse SETLOC,deviceId,lat,lon
      if (setlocMsg.startsWith("SETLOC,")) {
        setlocMsg = setlocMsg.substring(7); // Remove "SETLOC,"
        int comma1 = setlocMsg.indexOf(',');
        int comma2 = setlocMsg.indexOf(',', comma1 + 1);
        
        if (comma1 != -1 && comma2 != -1) {
          String deviceId = setlocMsg.substring(0, comma1);
          String latStr = setlocMsg.substring(comma1 + 1, comma2);
          String lonStr = setlocMsg.substring(comma2 + 1);
          
          // Update setLocation in memory
          currentData.deviceSetLocation = latStr + ", " + lonStr;
          strncpy(rtcDeviceSetLocation, currentData.deviceSetLocation.c_str(), sizeof(rtcDeviceSetLocation) - 1);
          rtcDeviceDetailsValid = true;
          
          Serial.println("✅ Updated setLocation: " + currentData.deviceSetLocation);
        }
      }
      
      // Find and delete this SMS
      int msgIndexStart = response.lastIndexOf("+CMGL: ", setlocIndex);
      if (msgIndexStart != -1) {
        int msgIndexEnd = response.indexOf(",", msgIndexStart + 7);
        int smsIndex = response.substring(msgIndexStart + 7, msgIndexEnd).toInt();
        sim7600.print("AT+CMGD=");
        sim7600.println(smsIndex);
        delay(500);
      }
    }
    
    // Look for control messages starting with "CMD,"
    int cmdIndex = response.indexOf("CMD,");
    if (cmdIndex != -1) {
      // Extract the control message
      int endIndex = response.indexOf('\r', cmdIndex);
      if (endIndex == -1) endIndex = response.indexOf('\n', cmdIndex);
      if (endIndex == -1) endIndex = response.length();
      
      String controlMsg = response.substring(cmdIndex, endIndex);
      Serial.println("Control message received: " + controlMsg);
      
      // Parse and apply the control command
      parseControlSMS(controlMsg);
      
      // Find and delete the SMS
      int msgIndexStart = response.lastIndexOf("+CMGL: ", cmdIndex);
      if (msgIndexStart != -1) {
        int msgIndexEnd = response.indexOf(",", msgIndexStart + 7);
        int smsIndex = response.substring(msgIndexStart + 7, msgIndexEnd).toInt();
        
        // Delete the SMS
        sim7600.print("AT+CMGD=");
        sim7600.println(smsIndex);
        delay(1000);
        
        // Verify deletion
        String delResponse = waitForGPSResponse(1000);
        if (delResponse.indexOf("OK") != -1) {
          Serial.println("✅ Control SMS deleted successfully");
        } else {
          Serial.println("❌ Failed to delete control SMS");
        }
        cmdFound = true; // Mark that we found and processed the CMD
      }
    }
    
    if (!cmdFound && attempts == 1) {
      Serial.println("No control SMS found in response.");
    }
    
    // If no CMD found, wait a bit before next attempt
    if (!cmdFound && attempts < 3) {
      Serial.println("Waiting for CMD message... (attempt " + String(attempts) + "/3)");
      delay(1000); // Reduced from 3 seconds
    }
  } else {
    if (attempts == 1) {
      Serial.println("No SMS response received from modem.");
    }
  }
  } // End of while loop
  
  if (!cmdFound) {
    Serial.println("No CMD control message received after 3 attempts.");
  }
}

void parseControlSMS(String smsContent) {
  // Format: "CMD,buzzer,solenoid,dismiss,clearBreach"
  // Example: "CMD,1,0,1,0" means buzzer on, solenoid off, dismissed true, don't clear breach
  
  if (!smsContent.startsWith("CMD,")) return;
  
  // Remove "CMD," prefix
  smsContent = smsContent.substring(4);
  
  // Parse values
  int firstComma = smsContent.indexOf(',');
  int secondComma = smsContent.indexOf(',', firstComma + 1);
  int thirdComma = smsContent.indexOf(',', secondComma + 1);
  
  if (firstComma != -1 && secondComma != -1) {
    String buzzerStr = smsContent.substring(0, firstComma);
    String solenoidStr = smsContent.substring(firstComma + 1, secondComma);
    String dismissStr;
    String clearBreachStr = "0"; // Default to not clearing breach
    
    if (thirdComma != -1) {
      // New format with 4 fields
      dismissStr = smsContent.substring(secondComma + 1, thirdComma);
      clearBreachStr = smsContent.substring(thirdComma + 1);
    } else {
      // Old format with 3 fields (backward compatibility)
      dismissStr = smsContent.substring(secondComma + 1);
    }
    
    // Apply control states
    bool newBuzzerState = (buzzerStr == "1");
    bool newSolenoidState = (solenoidStr == "1");
    bool newDismissState = (dismissStr == "1");
    bool clearBreach = (clearBreachStr == "1");
    
    Serial.println("✅ CONTROL COMMAND RECEIVED - Applying immediately...");
    Serial.printf("Control states: Buzzer=%d, Solenoid=%d, Dismiss=%d, ClearBreach=%d\n", 
                  newBuzzerState, newSolenoidState, newDismissState, clearBreach);
    
    // Update dismiss state
    currentData.buzzerDismissed = newDismissState;
    rtcBuzzerDismissed = newDismissState;
    
    if (newDismissState) {
      Serial.println("✅ Buzzer dismissed by user");
    }
    
    // Clear security breach if Master instructs us to
    // This happens when lid is closed AND device is back in safe zone
    if (clearBreach && rtcSecurityBreachDetected) {
      Serial.println("🔓 CLEARING SECURITY BREACH - Device is secured and in safe zone");
      rtcSecurityBreachDetected = false;
      currentData.securityBreachActive = false;
    }
    
    // Apply buzzer state from Master (Master has already done all calculations)
    currentData.buzzerIsActive = newBuzzerState;
    rtcBuzzerActive = newBuzzerState;
    digitalWrite(BUZZER_PIN, newBuzzerState ? HIGH : LOW);
    
    if (newBuzzerState) {
      Serial.println("🔔 BUZZER ACTIVATED by Master command!");
      Serial.println("Buzzer pin " + String(BUZZER_PIN) + " set to HIGH");
    } else {
      Serial.println("🔕 Buzzer turned OFF by Master command");
      Serial.println("Buzzer pin " + String(BUZZER_PIN) + " set to LOW");
    }
    
    // Apply solenoid state from Master
    if (newSolenoidState != currentData.solenoidActive) {
      currentData.solenoidActive = newSolenoidState;
      rtcSolenoidActive = newSolenoidState;
      
      if (newSolenoidState) {
        // Starting new solenoid activation
        rtcSolenoidStartTime = 0; // Will be set in the monitoring loop
        Serial.println("🔓 Solenoid activation requested by Master");
      } else {
        // Solenoid deactivation
        digitalWrite(SOLENOID_PIN, LOW);
        rtcSolenoidStartTime = 0;
        Serial.println("🔒 Solenoid deactivated by Master");
      }
    }
    
    Serial.println("Control states applied:");
    Serial.println("  Buzzer: " + String(newBuzzerState ? "ON" : "OFF"));
    Serial.println("  Solenoid: " + String(newSolenoidState ? "ON" : "OFF"));
    Serial.println("  Dismiss: " + String(newDismissState ? "DISMISSED" : "NOT DISMISSED"));
    Serial.println("=================================");
  }
}

// =====================================================================
// SMS MEMORY CLEANUP
// =====================================================================

// Clean up SMS memory by deleting all messages
void cleanupSMSMemory() {
  Serial.println("\n🧹 Cleaning up SMS memory...");
  
  flushSIM7600Buffer();
  
  // Delete all messages (read, unread, sent, unsent)
  // AT+CMGD=1,4 deletes all messages
  sim7600.println("AT+CMGD=1,4");
  
  // Wait for response (reduced from 10 seconds total to 3 seconds)
  String response = waitForGPSResponse(3000);
  
  if (response.indexOf("OK") != -1) {
    Serial.println("✅ SMS memory cleaned successfully");
  } else {
    Serial.println("❌ Primary cleanup failed, trying alternative method...");
    
    // Alternative: Try deleting by type (faster with shorter delays)
    sim7600.println("AT+CMGD=1,1"); // Delete read messages
    delay(1000);
    
    sim7600.println("AT+CMGD=1,2"); // Delete sent messages
    delay(1000);
    
    sim7600.println("AT+CMGD=1,3"); // Delete unsent messages
    delay(1000);
  }
  
  // Show SMS storage status
  showSMSStorageStatus();
}

// Show SMS storage status
void showSMSStorageStatus() {
  flushSIM7600Buffer();
  sim7600.println("AT+CPMS?");
  delay(1000);
  String response = waitForGPSResponse(1000);
  Serial.println("SMS Storage Status: " + response);
}

// Check if SMS memory is nearly full (>80% used)
bool isSMSMemoryNearlyFull() {
  flushSIM7600Buffer();
  sim7600.println("AT+CPMS?");
  delay(1000);
  String response = waitForGPSResponse(1000);
  
  // Response format: +CPMS: "SM",used,total,"SM",used,total,"SM",used,total
  // Example: +CPMS: "SM",10,30,"SM",10,30,"SM",10,30
  
  if (response.indexOf("+CPMS:") != -1) {
    // Find the first set of numbers after "SM"
    int firstQuote = response.indexOf("\"SM\"");
    if (firstQuote != -1) {
      int firstComma = response.indexOf(",", firstQuote);
      if (firstComma != -1) {
        int secondComma = response.indexOf(",", firstComma + 1);
        if (secondComma != -1) {
          String usedStr = response.substring(firstComma + 1, secondComma);
          int thirdComma = response.indexOf(",", secondComma + 1);
          if (thirdComma != -1) {
            String totalStr = response.substring(secondComma + 1, thirdComma);
            
            int used = usedStr.toInt();
            int total = totalStr.toInt();
            
            if (total > 0) {
              float percentUsed = (float)used / (float)total * 100.0;
              Serial.println("SMS Memory: " + String(used) + "/" + String(total) + " (" + String(percentUsed, 1) + "% used)");
              
              // Return true if more than 80% full
              return percentUsed > 80.0;
            }
          }
        }
      }
    }
  }
  
  // If we can't parse, assume it's not full
  Serial.println("Could not parse SMS storage status");
  return false;
}

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