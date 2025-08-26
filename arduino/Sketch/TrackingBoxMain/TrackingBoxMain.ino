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
// Direct AT commands are used for GNSS/GPS and CLBS location services
#include <time.h>
#include <Adafruit_SHT31.h>
#define DEBUG_GNSS 1   // Set to 1 to enable verbose GNSS diagnostics (adds delay)
#define ENABLE_SHT30_SENSOR 1  // Set to 1 to enable SHT30 temperature/humidity sensor

// =====================================================================
// PIN DEFINITIONS
// =====================================================================
#define BATTERY_ADC_PIN     36
#define SHT30_SDA_PIN       21
#define SHT30_SCL_PIN       22
#define LSM6DSL_SDA_PIN     21
#define LSM6DSL_SCL_PIN     22
#define LSM6DSL_INT1_PIN    34
#define SIM7600_TX_PIN      19  // Verified working with UART2
#define SIM7600_RX_PIN      18  // Verified working with UART2
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

// Device ID validation - auto-generates unique ID based on MAC address
String actualDeviceID = "";  // Runtime device ID (MAC-based)
RTC_DATA_ATTR char rtcActualDeviceID[32] = "";  // Persist across deep sleep
RTC_DATA_ATTR bool rtcDeviceIDValidated = false;  // Flag to track if ID was validated
String deviceMacAddress = "";  // Store the device's MAC address

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
HardwareSerial sim7600(2);  // Using UART2 - verified working with SIM7600G-H
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

// RTC memory for automated buzzer/solenoid tracking
RTC_DATA_ATTR bool rtcBuzzerActive = false;  // Track if buzzer was activated by lock breach
RTC_DATA_ATTR bool rtcSolenoidActive = false;  // Track if solenoid was activated for delivery
RTC_DATA_ATTR unsigned long rtcSolenoidStartTime = 0;  // Track solenoid activation time


// RTC memory for unique reference code
RTC_DATA_ATTR char rtcReferenceCode[11] = ""; // 10 chars + null terminator
RTC_DATA_ATTR bool rtcReferenceCodeGenerated = false;

// RTC memory for last Firebase update time
RTC_DATA_ATTR unsigned long rtcLastFirebaseUpdate = 0;

// --------------------------------------------------------------
// GEO HELPERS
// --------------------------------------------------------------

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

// Calculate distance between two GPS coordinates using Haversine formula
// Returns distance in meters
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0; // Earth radius in meters
  const double phi1 = lat1 * PI / 180.0;
  const double phi2 = lat2 * PI / 180.0;
  const double deltaPhi = (lat2 - lat1) * PI / 180.0;
  const double deltaLambda = (lon2 - lon1) * PI / 180.0;

  const double a = sin(deltaPhi / 2) * sin(deltaPhi / 2) +
                  cos(phi1) * cos(phi2) *
                  sin(deltaLambda / 2) * sin(deltaLambda / 2);
  const double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return R * c; // Distance in meters
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

// Forward declarations
void determineWakeUpReason();
void updateDisplay();
bool sendSensorDataToFirebase();
bool initializeCellularData();
void sendATCommand(const char* cmd, int timeout);
String sendATCommandResponse(const char* cmd, int timeout);
void parseFirebaseDetails(String jsonData);
void generateReferenceCode();
void sendMotionAlert();
void sendMotionEventAlert(String eventType, String message);
void sendDeliveryNotification();
void activateSolenoidForDelivery(unsigned long duration);
void handleLockBreachEarly();
void handleBuzzerActivation(unsigned long duration);
double calculateDistance(double lat1, double lon1, double lat2, double lon2);
bool sendFirebaseHTTP(String path, String jsonData, String method);
String readFirebaseHTTP(String path);
bool checkDeviceIDExists(String deviceID);
String generateNextDeviceID(String currentID);
String getDeviceMacAddress();
String generateDeviceIDFromMAC();
void flushSIM7600Buffer();
void sendAT(const char *cmd, uint16_t delayMs);
void showOfflineQRCode();

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
  
  // Get device MAC address for unique identification
  deviceMacAddress = getDeviceMacAddress();
  Serial.println("Device MAC Address: " + deviceMacAddress);

  // Load or generate reference code and device ID from permanent storage
  preferences.begin("tracking", false);  // Open in read/write mode
  
  // Load reference code
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
  
  // Load saved device ID from permanent storage
  String storedDeviceID = preferences.getString("deviceID", "");
  if (storedDeviceID.length() > 0) {
    // Device ID exists in permanent storage, use it
    actualDeviceID = storedDeviceID;
    actualDeviceID.toCharArray(rtcActualDeviceID, sizeof(rtcActualDeviceID));
    rtcDeviceIDValidated = true;
    Serial.println("✓ Loaded permanent device ID: " + actualDeviceID);
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
  
  // OPTIMIZED LOCK BREACH HANDLING - Handle immediately after cellular init
  if (currentData.wakeUpReason == "LOCK BREACH" && rtcBootCount > 1) {
    handleLockBreachEarly();
    // handleLockBreachEarly() handles buzzer/solenoid based on location
    // Normal cycle continues after to fetch details and update display
  }
  
  // Validate Device ID uniqueness only if we don't have a saved ID
  if (actualDeviceID.length() == 0) {
    // No saved device ID found in permanent storage or RTC memory
    if (!rtcDeviceIDValidated || strlen(rtcActualDeviceID) == 0) {
      Serial.println("\n🔍 No saved device ID found. Generating MAC-based Device ID...");
      actualDeviceID = generateDeviceIDFromMAC();
      actualDeviceID.toCharArray(rtcActualDeviceID, sizeof(rtcActualDeviceID));
      rtcDeviceIDValidated = true;
      
      // Save to preferences for permanent storage
      preferences.begin("tracking", false);
      preferences.putString("deviceID", actualDeviceID);
      preferences.end();
      
      Serial.println("✅ Generated MAC-based ID: " + actualDeviceID);
      
      // Device ID is validated - sensor data will be sent in the main flow
      Serial.println("📝 New device ID registered: " + actualDeviceID);
      Serial.println("ℹ️ Device details will be set from web dashboard");
      Serial.println("ℹ️ Sensor data will be sent after collection");
    } else {
      // Load from RTC memory (survives deep sleep)
      actualDeviceID = String(rtcActualDeviceID);
      Serial.println("✅ Using saved Device ID from RTC: " + actualDeviceID);
    }
  } else {
    // Device ID was already loaded from permanent storage
    Serial.println("✅ Using permanent Device ID: " + actualDeviceID);
  }
  
  // Send motion alert if device woke from motion detection
  if (currentData.wakeUpReason == "MOTION DETECTED" && rtcBootCount > 1) {
    sendMotionAlert();
  }
  
  // Control commands removed - buzzer/solenoid handled automatically by location detection
  
  // Restore device details from RTC memory if available
  if (rtcDeviceDetailsValid) {
    currentData.deviceName = String(rtcDeviceName);
    currentData.deviceSetLocation = String(rtcDeviceSetLocation);
    Serial.println("✓ Restored device details from RTC memory:");
    Serial.println("  Device name: " + currentData.deviceName);
    Serial.println("  Set location: " + currentData.deviceSetLocation);
  } else {
    Serial.println("⚠️ No device details in RTC memory - fetching from Firebase");
    // Read device details from Firebase
    String detailsPath = "/tracking_box/" + actualDeviceID + "/details";
    String detailsData = readFirebaseHTTP(detailsPath);
    
    if (detailsData.length() > 0 && detailsData.indexOf("null") == -1) {
      parseFirebaseDetails(detailsData);
    }
  }
  
  // Collect sensor data for Firebase transmission
  collectSensorReading();
  Serial.println("✅ Sensor Readings Collected.");
  
  // Send alerts for special motion events (shock, tilt) if detected
  if (currentData.fallDetected && rtcBootCount > 1) {
    sendMotionEventAlert("shock", "Shock/impact detected");
  }
  if (currentData.tiltDetected && rtcBootCount > 1) {
    sendMotionEventAlert("tilt", "Device tilted");
  }
  
  // Fetch latest shipping label data from Firebase before display update
  Serial.println("🌐 Fetching shipping label data from Firebase...");
  String detailsPath = "/tracking_box/" + actualDeviceID + "/details";
  String detailsData = readFirebaseHTTP(detailsPath);
  if (detailsData.length() > 0 && detailsData.indexOf("null") == -1) {
    parseFirebaseDetails(detailsData);
    Serial.println("✅ Shipping label data updated from Firebase");
  } else {
    Serial.println("⚠️ No shipping label data found in Firebase - using defaults");
  }
  
  // Send sensor data directly to Firebase
  Serial.println("📊 Preparing to send sensor data to Firebase...");
  if (sendSensorDataToFirebase()) {
    Serial.println("✅ Sensor data sent to Firebase successfully!");
    
    // Update display with QR code
    showOfflineQRCode();
    
    Serial.println("Cycle complete → deep sleep (Firebase mode).");
  } else {
    Serial.println("❌ Firebase send failed. Operating in offline mode.");
    showOfflineQRCode();
    Serial.println("Cycle complete → deep sleep (offline mode).");
  }
  
  // Ensure buzzer is off before deep sleep (unless still active from lock breach)
  digitalWrite(BUZZER_PIN, LOW);
  
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
// Buzzer and solenoid control handled automatically by location detection

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
  
  // Note: Lid open state is handled automatically by handleLockBreachEarly()

  if (currentData.gpsFixValid || currentData.coarseFix) {
    currentData.currentLocation = String(currentData.latitude, 6) + ", " + String(currentData.longitude, 6);
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
  delay(30000);                      // Initial delay before GPS acquisition
  sendGPSCommand("AT+CGPS=1,1");   // Start GPS in standalone mode
  delay(2000);                      // Allow the receiver to power-up

  // Skip AT+CGPSINFOCFG command - commented out
  // sendAT("AT+CGPSINFOCFG=1,31", 2000);
  Serial.println("\nWaiting 10 seconds for GPS to get signal...");
  
  // Simple delay without interrupt checking to prevent stuck cycles
  for (int i = 0; i < 100; i++) {
    delay(100);
  }
  
  // sendAT("AT+CGPSINFOCFG=0,31", 2000);
  // Power-mode and NMEA configuration diagnostics
  sendAT("AT+CGPSPMD?", 2000);
  sendAT("AT+CGPSNMEA?", 2000);

  // Request current location data
  flushSIM7600Buffer();

  sim7600.println("AT+CGNSSINFO"); // Query both for robustness
  
  // Simple delay without interrupt checking to prevent stuck cycles
  for (int i = 0; i < 20; i++) {
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
  Serial.println("📡 Starting CLBS positioning sequence...");
  flushSIM7600Buffer();
  
  String resp = "";  // Declare response variable
  
  // Step 4: Start network connection
  flushSIM7600Buffer();
  sim7600.println("AT+CNETSTART");
  resp = waitForGPSResponse(3000);
  if (resp.indexOf("ERROR") != -1) {
    Serial.println("⚠️ CNETSTART failed, trying restart...");
    flushSIM7600Buffer();
    sim7600.println("AT+CNETSTOP");
    delay(1000);
    flushSIM7600Buffer();
    sim7600.println("AT+CNETSTART");
    resp = waitForGPSResponse(3000);
    if (resp.indexOf("ERROR") != -1) {
      Serial.println("✗ Failed to start network connection");
      return false;
    }
  }
  Serial.println("✓ Network connection started");
  
  // Step 5: Get CLBS location
  flushSIM7600Buffer();
  sim7600.println("AT+CLBS=1");
  resp = waitForGPSResponse(20000);  // Increased timeout for CLBS
  
  int idx = resp.indexOf("+CLBS:");
  if (idx == -1) {
    Serial.println("✗ No CLBS response");
    return false;
  }
  
  // Parse format: +CLBS: <locationcode>,<latitude>,<longitude>,<acc>
  int firstComma = resp.indexOf(',', idx);
  if (firstComma == -1) {
    Serial.println("✗ Invalid CLBS format");
    return false;
  }
  
  // Get location code (error code)
  String locCode = resp.substring(idx + 7, firstComma);
  int err = locCode.toInt();
  if (err != 0) {
    Serial.printf("✗ CLBS error code: %d\n", err);
    return false;
  }
  
  // Get latitude (comes first after error code)
  int secondComma = resp.indexOf(',', firstComma + 1);
  if (secondComma == -1) {
    Serial.println("✗ Missing latitude in CLBS response");
    return false;
  }
  String latStr = resp.substring(firstComma + 1, secondComma);
  
  // Get longitude (comes second)
  int thirdComma = resp.indexOf(',', secondComma + 1);
  if (thirdComma == -1) {
    Serial.println("✗ Missing longitude in CLBS response");
    return false;
  }
  String lonStr = resp.substring(secondComma + 1, thirdComma);
  
  // Get accuracy if available
  int fourthComma = resp.indexOf(',', thirdComma + 1);
  String accStr = "";
  if (fourthComma != -1) {
    accStr = resp.substring(thirdComma + 1, fourthComma);
  }
  
  // Debug: Print what we parsed
  Serial.println("📍 CLBS Parsing Debug:");
  Serial.printf("   latStr (1st value) = '%s'\n", latStr.c_str());
  Serial.printf("   lonStr (2nd value) = '%s'\n", lonStr.c_str());
  
  // CLBS response format from SIM7600: errorcode,latitude,longitude,accuracy
  // latStr contains latitude (first coordinate after error code)
  // lonStr contains longitude (second coordinate)
  double clbs_latitude = latStr.toDouble();
  double clbs_longitude = lonStr.toDouble();
  
  Serial.printf("   clbs_latitude = %.8f\n", clbs_latitude);
  Serial.printf("   clbs_longitude = %.8f\n", clbs_longitude);
  
  if (clbs_latitude == 0.0 || clbs_longitude == 0.0) {
    Serial.println("✗ Invalid coordinates (0,0)");
    return false;
  }
  
  // Assign to struct fields in correct order
  currentData.latitude = clbs_latitude;    // Latitude (e.g., 14.60)
  currentData.longitude = clbs_longitude;  // Longitude (e.g., 120.98)
  
  Serial.printf("   After assignment:\n");
  Serial.printf("   currentData.latitude = %.8f (should be ~14.60)\n", currentData.latitude);
  Serial.printf("   currentData.longitude = %.8f (should be ~120.98)\n", currentData.longitude);
  currentData.altitude = 0;
  currentData.gpsFixValid = false;   // not a GNSS fix
  currentData.coarseFix = true;
  
  if (accStr.length() > 0) {
    Serial.printf("✓ CLBS coarse fix: lat=%.5f, lon=%.5f (accuracy: %sm)\n", clbs_latitude, clbs_longitude, accStr.c_str());
  } else {
    Serial.printf("✓ CLBS coarse fix: lat=%.5f, lon=%.5f\n", clbs_latitude, clbs_longitude);
  }
  
  return true;
}

// =====================================================================
// HARDWARE INITIALIZATION
// =====================================================================
bool initializeAllHardware() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Buzzer off by default
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

  // Initialize SIM7600 with verified working configuration
  // UART2: RX=GPIO18, TX=GPIO19 @ 115200 baud
  sim7600.begin(115200, SERIAL_8N1, SIM7600_RX_PIN, SIM7600_TX_PIN);
  sim7600.setRxBufferSize(2048);  // Increase buffer for large HTTP responses
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
    // Simple wait without interrupt checking to prevent stuck cycles
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
    
    // Draw vertical bars pattern - optimized for wider display
    int barWidth = 3;  // Wider base bars for better visibility
    int currentX = xPos + 5;  // Start closer to edge
    
    // Create a more realistic barcode pattern that fills the width
    for (int i = 0; i < 120 && currentX < (xPos + width - 5); i++) {
        // Varied pattern for realistic barcode appearance
        if (i % 2 == 0 || i % 3 == 0 || i % 5 == 0) {
            // Vary bar widths: thin (3px), medium (6px), thick (9px)
            int thisBarWidth;
            if (i % 7 == 0) {
                thisBarWidth = barWidth * 3;  // Thick bar
            } else if (i % 4 == 0) {
                thisBarWidth = barWidth * 2;  // Medium bar
            } else {
                thisBarWidth = barWidth;      // Thin bar
            }
            
            Paint_DrawRectangle(currentX, yPos + 2, currentX + thisBarWidth, yPos + height - 2, 
                               EPD_7IN3F_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
            currentX += thisBarWidth + 2;  // Small gap between bars
        } else {
            currentX += barWidth + 1;  // Space for white areas
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
  Paint_DrawLine(30, y, EPD_7IN3F_WIDTH - 30, y, EPD_7IN3F_BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
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
  
  // ========== ROUTING INFO ==========
  // Routing Code and Postal Code (without Maxicode)
  String routing = currentData.routingCode.length() > 0 ? currentData.routingCode : "Not Provided";
  String postal = currentData.postalCode.length() > 0 ? currentData.postalCode : "Not Provided";
  
  Paint_DrawString_EN(leftMargin, y + 10, routing.c_str(), &Font20, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  Paint_DrawString_EN(leftMargin, y + 40, postal.c_str(), &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  y += 70;  // Move y position after routing info

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
  
  // Draw QR code (larger size)
  const int scale = 4;  // Increased from 3 to 4 for larger QR code
  const int qrSize = qrcode.size;
  const int qrOffsetX = rightColumnX + 40;  // Adjusted for better centering
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
  y += 20;  // Add spacing before tracking section
  Paint_DrawLine(30, y, EPD_7IN3F_WIDTH - 30, y, EPD_7IN3F_BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  y += 20;
  
  String tracking = currentData.trackingNumber.length() > 0 ? currentData.trackingNumber : "Not Provided";
  int textWidth = tracking.length() * 14;
  int centerX = (EPD_7IN3F_WIDTH - textWidth) / 2;
  Paint_DrawString_EN(centerX, y, tracking.c_str(), &Font24, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  y += 40;
  
  // ========== BARCODE ==========
  drawBarcode(50, y, 700, 60);  // Extended to 700px width with 50px margins, taller at 60px
  y += 65;
  
  // Barcode numbers aligned with extended barcode
  Paint_DrawString_EN(60, y, "0522", &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  Paint_DrawString_EN(710, y, "2077", &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  
  // ========== BOTTOM INFO ==========
  y += 30;
  // Show reference code at bottom
  snprintf(buf, sizeof(buf), "REF: %s", currentData.referenceCode.c_str());
  Paint_DrawString_EN(leftMargin, y, buf, &Font12, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  
  // Parse safe zone coordinates if available
  double safeLat = 0.0, safeLon = 0.0;
  bool hasSafeZone = parseCoordPair(currentData.deviceSetLocation, safeLat, safeLon);
  
  // Show current GPS location - simple format
  snprintf(buf, sizeof(buf), "GPS: %.3f, %.3f", currentData.latitude, currentData.longitude);
  Paint_DrawString_EN(rightColumnX, y, buf, &Font12, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  
  // Show safe zone location if available
  if (hasSafeZone && safeLat != 0.0 && safeLon != 0.0) {
    y += 20;
    snprintf(buf, sizeof(buf), "SAFE: %.3f, %.3f", safeLat, safeLon);
    Paint_DrawString_EN(rightColumnX, y, buf, &Font12, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
  }

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
  
  // Test communication
  String response = sendATCommandResponse("AT", 2000);
  if (response.indexOf("OK") == -1) {
    Serial.println("❌ SIM7600 not responding");
    return false;
  }
  
  // Set full functionality
  response = sendATCommandResponse("AT+CFUN=1", 5000);
  if (response.indexOf("OK") == -1) {
    Serial.println("⚠️ Failed to set full functionality");
  }
  
  // Check SIM status
  response = sendATCommandResponse("AT+CPIN?", 2000);
  if (response.indexOf("READY") == -1) {
    Serial.println("❌ SIM card not ready");
    Serial.println("Response: " + response);
    return false;
  }
  Serial.println("✅ SIM card ready");
  
  // Check network registration
  int attempts = 0;
  bool registered = false;
  while (attempts < 10 && !registered) {
    response = sendATCommandResponse("AT+CREG?", 2000);
    if (response.indexOf(",1") != -1 || response.indexOf(",5") != -1) {
      registered = true;
      Serial.println("✅ Network registered");
    } else {
      Serial.println("⏳ Waiting for network registration... (attempt " + String(attempts + 1) + "/10)");
      delay(3000);
      attempts++;
    }
  }
  
  if (!registered) {
    Serial.println("❌ Failed to register on network");
    return false;
  }
  
  // Attach to GPRS service
  response = sendATCommandResponse("AT+CGATT=1", 10000);
  if (response.indexOf("OK") == -1) {
    Serial.println("⚠️ Failed to attach to GPRS");
  }
  
  // Configure SSL/TLS for HTTPS (optional, continue even if fails)
  sendATCommand("AT+CSSLCFG=\"sslversion\",0,3", 2000);
  sendATCommand("AT+CSSLCFG=\"authmode\",0,0", 2000);
  sendATCommand("AT+CSSLCFG=\"ignorelocaltime\",0,1", 2000);
  
  // Configure and activate PDP context
  String apnCmd = "AT+CGDCONT=1,\"IP\",\"" + String(APN) + "\"";
  response = sendATCommandResponse(apnCmd.c_str(), 3000);
  if (response.indexOf("OK") == -1) {
    Serial.println("⚠️ Failed to set APN");
  }
  
  response = sendATCommandResponse("AT+CGACT=1,1", 15000);
  if (response.indexOf("OK") == -1) {
    Serial.println("⚠️ Failed to activate PDP context");
  }
  
  // Check signal strength
  response = sendATCommandResponse("AT+CSQ", 2000);
  if (response.indexOf("+CSQ:") != -1) {
    int signalStart = response.indexOf("+CSQ:") + 5;
    int signalEnd = response.indexOf(",", signalStart);
    if (signalEnd > signalStart) {
      String signalStr = response.substring(signalStart, signalEnd);
      int signal = signalStr.toInt();
      Serial.println("📶 Signal strength: " + String(signal) + "/31");
      if (signal < 5) {
        Serial.println("⚠️ Weak signal, may affect connectivity");
      }
    }
  }
  
  Serial.println("✅ Cellular data initialization complete!");
  return true;
}

bool sendSensorDataToFirebase() {
  Serial.println("\n📤 SENDING SENSOR DATA TO FIREBASE");
  Serial.println("Device ID: " + actualDeviceID);
  Serial.println("Current sensor values:");
  Serial.println("  Temperature: " + String(currentData.temperature, 1) + "°C");
  Serial.println("  Humidity: " + String(currentData.humidity, 1) + "%");
  Serial.println("  Location: " + String(currentData.latitude, 6) + ", " + String(currentData.longitude, 6));
  Serial.println("  Battery: " + String(currentData.batteryVoltage, 2) + "V");
  
  // Create JSON payload for Firebase
  String jsonData = "{";
  jsonData += "\"temp\":" + String(currentData.temperature, 1) + ",";
  jsonData += "\"humidity\":" + String(currentData.humidity, 1) + ",";
  jsonData += "\"currentLocation\":\"" + String(currentData.latitude, 6) + ", " + String(currentData.longitude, 6) + "\",";
  jsonData += "\"altitude\":" + String(currentData.altitude, 1) + ",";
  jsonData += "\"tilt\":" + String(currentData.tiltDetected ? "true" : "false") + ",";
  jsonData += "\"fall\":" + String(currentData.fallDetected ? "true" : "false") + ",";
  jsonData += "\"limitSwitch\":" + String(currentData.limitSwitchPressed ? "true" : "false") + ",";
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
  jsonData += "\"referenceCode\":\"" + currentData.referenceCode + "\"";
  jsonData += "}";
  
  // Send to Firebase
  String path = "/tracking_box/" + actualDeviceID + "/sensorData";
  Serial.println("📍 Firebase path: " + path);
  Serial.println("📝 JSON payload: " + jsonData);
  
  bool success = sendFirebaseHTTP(path, jsonData, "PUT");
  
  if (success) {
    Serial.println("✅ Sensor data sent to Firebase successfully");
    Serial.println("🔗 Check Firebase at: " + String(FIREBASE_URL) + path);
    rtcLastFirebaseUpdate = millis();
    
    // No need to check control commands - handled automatically by location detection
  } else {
    Serial.println("❌ Failed to send sensor data to Firebase");
  }
  
  return success;
}

// Send data to Firebase via HTTP
bool sendFirebaseHTTP(String path, String jsonData, String method) {
  Serial.println("\n=== SENDING TO FIREBASE ===");
  Serial.println("Path: " + path);
  Serial.println("Method: " + method);
  Serial.println("JSON Length: " + String(jsonData.length()));
  Serial.println("JSON Data: " + jsonData);
  
  // Terminate any existing HTTP session
  sendATCommand("AT+HTTPTERM", 1000);
  delay(500);
  
  // Initialize HTTP service
  String initResponse = sendATCommandResponse("AT+HTTPINIT", 3000);
  if (initResponse.indexOf("OK") == -1) {
    Serial.println("❌ Failed to initialize HTTP service");
    return false;
  }
  
  // Set URL
  String url = String(FIREBASE_URL) + path + ".json";
  String urlCmd = "AT+HTTPPARA=\"URL\",\"" + url + "\"";
  Serial.println("Setting URL: " + url);
  String urlResponse = sendATCommandResponse(urlCmd.c_str(), 3000);
  if (urlResponse.indexOf("OK") == -1) {
    Serial.println("❌ Failed to set URL");
    sendATCommand("AT+HTTPTERM", 1000);
    return false;
  }
  
  // Set content type
  String contentResponse = sendATCommandResponse("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 2000);
  if (contentResponse.indexOf("OK") == -1) {
    Serial.println("❌ Failed to set content type");
    sendATCommand("AT+HTTPTERM", 1000);
    return false;
  }
  
  // Prepare to send data
  String dataCmd = "AT+HTTPDATA=" + String(jsonData.length()) + ",10000";
  Serial.println("Preparing to send data: " + dataCmd);
  flushSIM7600Buffer();
  sim7600.println(dataCmd);
  
  // Wait for DOWNLOAD prompt
  String downloadPrompt = "";
  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {
    if (sim7600.available()) {
      downloadPrompt += sim7600.readString();
      if (downloadPrompt.indexOf("DOWNLOAD") != -1) {
        Serial.println("Got DOWNLOAD prompt");
        break;
      }
    }
    delay(10);
  }
  
  if (downloadPrompt.indexOf("DOWNLOAD") == -1) {
    Serial.println("❌ No DOWNLOAD prompt received");
    Serial.println("Received instead: " + downloadPrompt);
    sendATCommand("AT+HTTPTERM", 1000);
    return false;
  }
  
  // Send JSON data
  Serial.println("Sending JSON data...");
  sim7600.print(jsonData);
  delay(500);
  
  // Wait for OK after data send
  String dataResponse = "";
  startTime = millis();
  while (millis() - startTime < 2000) {
    if (sim7600.available()) {
      dataResponse += sim7600.readString();
    }
    delay(10);
  }
  Serial.println("Data send response: " + dataResponse);
  
  // Execute HTTP action (0=GET, 1=PUT, 2=POST)
  int action = (method == "GET") ? 0 : (method == "PUT") ? 1 : 2;
  String actionCmd = "AT+HTTPACTION=" + String(action);
  String actionResponse = sendATCommandResponse(actionCmd.c_str(), 15000);
  
  // Look for +HTTPACTION response
  if (actionResponse.indexOf("+HTTPACTION:") != -1) {
    Serial.println("HTTP Action response received");
    
    // Extract status code
    int statusStart = actionResponse.indexOf(",") + 1;
    int statusEnd = actionResponse.indexOf(",", statusStart);
    if (statusStart > 0 && statusEnd > statusStart) {
      String statusCode = actionResponse.substring(statusStart, statusEnd);
      Serial.println("HTTP Status Code: " + statusCode);
      
      if (statusCode == "200" || statusCode == "204") {
        Serial.println("✅ Firebase request successful");
        sendATCommand("AT+HTTPTERM", 1000);
        return true;
      }
    }
  }
  
  // Try to read any error response
  String errorResponse = sendATCommandResponse("AT+HTTPREAD=0,500", 3000);
  if (errorResponse.length() > 0) {
    Serial.println("Error response: " + errorResponse);
  }
  
  Serial.println("❌ Firebase request failed");
  sendATCommand("AT+HTTPTERM", 1000);
  return false;
}

// Read data from Firebase
String readFirebaseHTTP(String path) {
  Serial.println("\n=== READING FROM FIREBASE ===");
  Serial.println("Path: " + path);
  
  // Terminate any existing HTTP session
  sendATCommand("AT+HTTPTERM", 1000);
  delay(500);
  
  // Initialize HTTP service
  String initResponse = sendATCommandResponse("AT+HTTPINIT", 3000);
  if (initResponse.indexOf("OK") == -1) {
    Serial.println("❌ Failed to initialize HTTP service");
    return "";
  }
  
  // Set URL
  String url = String(FIREBASE_URL) + path + ".json";
  String urlCmd = "AT+HTTPPARA=\"URL\",\"" + url + "\"";
  Serial.println("Setting URL: " + url);
  String urlResponse = sendATCommandResponse(urlCmd.c_str(), 3000);
  if (urlResponse.indexOf("OK") == -1) {
    Serial.println("❌ Failed to set URL");
    sendATCommand("AT+HTTPTERM", 1000);
    return "";
  }
  
  // Execute GET request
  String actionResponse = sendATCommandResponse("AT+HTTPACTION=0", 10000);
  
  // Wait for +HTTPACTION response
  if (actionResponse.indexOf("+HTTPACTION:") == -1) {
    // Wait a bit more for the response
    delay(2000);
    actionResponse = "";
    unsigned long startTime = millis();
    while (millis() - startTime < 3000) {
      if (sim7600.available()) {
        actionResponse += sim7600.readString();
        if (actionResponse.indexOf("+HTTPACTION:") != -1) {
          break;
        }
      }
      delay(10);
    }
  }
  
  // Read response data
  String response = sendATCommandResponse("AT+HTTPREAD=0,1000", 3000);
  
  // First try to extract JSON object (between { and })
  int jsonStart = response.indexOf('{');
  int jsonEnd = response.lastIndexOf('}');
  
  if (jsonStart != -1 && jsonEnd != -1) {
    // Found JSON object
    response = response.substring(jsonStart, jsonEnd + 1);
  } else {
    // No JSON object found, try to extract plain string value
    // Look for the actual data after +HTTPREAD: DATA,XX format
    int dataStart = response.indexOf("+HTTPREAD: DATA,");
    if (dataStart != -1) {
      // Find the line after DATA line
      int newlineAfterData = response.indexOf('\n', dataStart);
      if (newlineAfterData != -1) {
        // Get the next line which contains the actual value
        int valueStart = newlineAfterData + 1;
        int valueEnd = response.indexOf('\n', valueStart);
        if (valueEnd == -1) {
          valueEnd = response.indexOf("+HTTPREAD: 0", valueStart);
          if (valueEnd == -1) {
            valueEnd = response.length();
          }
        }
        
        String value = response.substring(valueStart, valueEnd);
        value.trim();
        
        // Remove surrounding quotes if present
        if (value.startsWith("\"") && value.endsWith("\"")) {
          value = value.substring(1, value.length() - 1);
        }
        
        response = value;
      } else {
        response = "";
      }
    } else {
      response = "";
    }
  }
  
  sendATCommand("AT+HTTPTERM", 1000);
  return response;
}

// Send AT command with response
void sendATCommand(const char* cmd, int timeout) {
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
}

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
// FIREBASE DETAILS FUNCTIONS
// =====================================================================
// Control functions removed - buzzer/solenoid handled automatically by location detection

void parseFirebaseDetails(String jsonData) {
  // Parse device details from Firebase
  // Format: {"name":"...","setLocation":"...","description":"...","senderName":"...","recipientName":"...",...}
  
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
  
  // Parse description
  int descStart = jsonData.indexOf("\"description\":\"") + 15;
  if (descStart > 14) {
    int descEnd = jsonData.indexOf("\"", descStart);
    if (descEnd != -1) {
      currentData.deviceDescription = jsonData.substring(descStart, descEnd);
      Serial.println("✅ Updated description: " + currentData.deviceDescription);
    }
  }
  
  // Parse shipping label fields
  
  // Sender Name
  int senderNameStart = jsonData.indexOf("\"senderName\":\"") + 14;
  if (senderNameStart > 13) {
    int senderNameEnd = jsonData.indexOf("\"", senderNameStart);
    if (senderNameEnd != -1) {
      currentData.senderName = jsonData.substring(senderNameStart, senderNameEnd);
      Serial.println("✅ Updated sender name: " + currentData.senderName);
    }
  }
  
  // Sender Address
  int senderAddrStart = jsonData.indexOf("\"senderAddress\":\"") + 17;
  if (senderAddrStart > 16) {
    int senderAddrEnd = jsonData.indexOf("\"", senderAddrStart);
    if (senderAddrEnd != -1) {
      currentData.senderAddress = jsonData.substring(senderAddrStart, senderAddrEnd);
      Serial.println("✅ Updated sender address: " + currentData.senderAddress);
    }
  }
  
  // Recipient Name
  int recipNameStart = jsonData.indexOf("\"recipientName\":\"") + 17;
  if (recipNameStart > 16) {
    int recipNameEnd = jsonData.indexOf("\"", recipNameStart);
    if (recipNameEnd != -1) {
      currentData.recipientName = jsonData.substring(recipNameStart, recipNameEnd);
      Serial.println("✅ Updated recipient name: " + currentData.recipientName);
    }
  }
  
  // Recipient Address
  int recipAddrStart = jsonData.indexOf("\"recipientAddress\":\"") + 20;
  if (recipAddrStart > 19) {
    int recipAddrEnd = jsonData.indexOf("\"", recipAddrStart);
    if (recipAddrEnd != -1) {
      currentData.recipientAddress = jsonData.substring(recipAddrStart, recipAddrEnd);
      Serial.println("✅ Updated recipient address: " + currentData.recipientAddress);
    }
  }
  
  // Package Weight
  int weightStart = jsonData.indexOf("\"packWeight\":\"") + 14;
  if (weightStart > 13) {
    int weightEnd = jsonData.indexOf("\"", weightStart);
    if (weightEnd != -1) {
      currentData.packWeight = jsonData.substring(weightStart, weightEnd);
      Serial.println("✅ Updated package weight: " + currentData.packWeight);
    }
  }
  
  // Routing Code
  int routingStart = jsonData.indexOf("\"routingCode\":\"") + 15;
  if (routingStart > 14) {
    int routingEnd = jsonData.indexOf("\"", routingStart);
    if (routingEnd != -1) {
      currentData.routingCode = jsonData.substring(routingStart, routingEnd);
      Serial.println("✅ Updated routing code: " + currentData.routingCode);
    }
  }
  
  // Postal Code
  int postalStart = jsonData.indexOf("\"postalCode\":\"") + 14;
  if (postalStart > 13) {
    int postalEnd = jsonData.indexOf("\"", postalStart);
    if (postalEnd != -1) {
      currentData.postalCode = jsonData.substring(postalStart, postalEnd);
      Serial.println("✅ Updated postal code: " + currentData.postalCode);
    }
  }
  
  // Tracking Number
  int trackingStart = jsonData.indexOf("\"trackingNumber\":\"") + 18;
  if (trackingStart > 17) {
    int trackingEnd = jsonData.indexOf("\"", trackingStart);
    if (trackingEnd != -1) {
      currentData.trackingNumber = jsonData.substring(trackingStart, trackingEnd);
      Serial.println("✅ Updated tracking number: " + currentData.trackingNumber);
    }
  }
  
  // Service Type
  int serviceStart = jsonData.indexOf("\"serviceType\":\"") + 15;
  if (serviceStart > 14) {
    int serviceEnd = jsonData.indexOf("\"", serviceStart);
    if (serviceEnd != -1) {
      currentData.serviceType = jsonData.substring(serviceStart, serviceEnd);
      Serial.println("✅ Updated service type: " + currentData.serviceType);
    }
  }
}

// =====================================================================
// UTILITY FUNCTIONS
// =====================================================================

// Send motion detection alert to Firebase
void sendMotionAlert() {
  Serial.println("\n🏃 SENDING MOTION ALERT TO FIREBASE...");
  
  // Create alert JSON payload with all required fields for MotionAlert interface
  String alertData = "{";
  alertData += "\"deviceId\":\"" + actualDeviceID + "\",";
  alertData += "\"location\":\"" + String(currentData.latitude, 6) + ", " + String(currentData.longitude, 6) + "\",";
  alertData += "\"message\":\"Motion detected\",";
  alertData += "\"timestamp\":" + String(millis()) + ",";
  alertData += "\"type\":\"motion\"";
  alertData += "}";
  
  // Create unique alert ID using timestamp
  String alertPath = "/tracking_box/" + actualDeviceID + "/alerts/motion/" + String(millis());
  
  // Send alert to Firebase
  bool success = sendFirebaseHTTP(alertPath, alertData, "PUT");
  
  if (success) {
    Serial.println("✅ Motion alert sent to Firebase successfully!");
    Serial.println("   Alert will trigger notification on web dashboard");
  } else {
    Serial.println("❌ Failed to send motion alert to Firebase");
  }
}

// Send specific motion event alerts (shock, tilt, etc.)
void sendMotionEventAlert(String eventType, String message) {
  Serial.println("\n⚠️ SENDING " + eventType + " ALERT TO FIREBASE...");
  
  // Create alert JSON payload with all required fields for MotionAlert interface
  String alertData = "{";
  alertData += "\"deviceId\":\"" + actualDeviceID + "\",";
  alertData += "\"location\":\"" + String(currentData.latitude, 6) + ", " + String(currentData.longitude, 6) + "\",";
  alertData += "\"message\":\"" + message + "\",";
  alertData += "\"timestamp\":" + String(millis()) + ",";
  alertData += "\"type\":\"" + eventType + "\"";
  alertData += "}";
  
  // Create unique alert ID using timestamp
  String alertPath = "/tracking_box/" + actualDeviceID + "/alerts/motion/" + String(millis());
  
  // Send alert to Firebase
  bool success = sendFirebaseHTTP(alertPath, alertData, "PUT");
  
  if (success) {
    Serial.println("✅ " + eventType + " alert sent to Firebase successfully!");
  } else {
    Serial.println("❌ Failed to send " + eventType + " alert to Firebase");
  }
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

// =====================================================================
// DEVICE ID VALIDATION FUNCTIONS
// =====================================================================

// Check if a device ID already exists in Firebase
bool checkDeviceIDExists(String deviceID) {
  Serial.println("Checking if ID exists in Firebase: " + deviceID);
  
  // Check if any data exists under /tracking_box/{deviceID}
  String path = "/tracking_box/" + deviceID;
  String response = readFirebaseHTTP(path);
  
  Serial.println("Firebase response for " + deviceID + ": " + response);
  
  // Firebase returns "null" or empty for non-existent paths
  if (response.length() == 0) {
    Serial.println("→ Empty response, ID does not exist");
    return false;
  }
  
  if (response.indexOf("null") != -1 && response.indexOf("{") == -1) {
    Serial.println("→ Response is 'null', ID does not exist");
    return false;  // ID doesn't exist
  }
  
  // If response contains any JSON data, ID exists
  if (response.indexOf("{") != -1) {
    Serial.println("→ Found JSON data, ID exists");
    return true;  // ID exists with data
  }
  
  Serial.println("→ Unclear response, assuming ID does not exist");
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

// Get device MAC address
String getDeviceMacAddress() {
  uint64_t mac = ESP.getEfuseMac();
  String macStr = "";
  
  // Convert MAC to string format XX:XX:XX:XX:XX:XX
  for (int i = 0; i < 6; i++) {
    uint8_t byte = (mac >> (i * 8)) & 0xFF;
    if (macStr.length() > 0) macStr += ":";
    if (byte < 0x10) macStr += "0";
    macStr += String(byte, HEX);
  }
  
  macStr.toUpperCase();
  return macStr;
}

// Generate device ID from MAC address
String generateDeviceIDFromMAC() {
  // Use last 3 bytes of MAC to create a unique suffix
  uint64_t mac = ESP.getEfuseMac();
  uint32_t uniqueNum = (mac & 0xFFFFFF);  // Last 3 bytes
  
  // Convert to a 3-digit number (001-999)
  int deviceNum = (uniqueNum % 999) + 1;
  
  // Format as box_XXX with leading zeros
  String deviceID = "box_";
  if (deviceNum < 10) deviceID += "00";
  else if (deviceNum < 100) deviceID += "0";
  deviceID += String(deviceNum);
  
  return deviceID;
}

// =====================================================================
// OPTIMIZED LOCK BREACH HANDLING FUNCTIONS
// =====================================================================

// Handle lock breach early in the boot cycle for faster response
void handleLockBreachEarly() {
  Serial.println("\n🔒 HANDLING LOCK BREACH - Optimized Early Detection");
  Serial.printf("   Process start time: %lu ms\n", millis());
  
  // Fetch ONLY setLocation from Firebase (optimized data usage)
  String setLocationPath = "/tracking_box/" + actualDeviceID + "/details/setLocation";
  String setLocationData = readFirebaseHTTP(setLocationPath);
  
  // readFirebaseHTTP now handles quote removal, so we get clean data
  Serial.println("📍 Safe zone location retrieved: '" + setLocationData + "'");
  
  // Parse safe zone coordinates
  double safeLat = 0.0, safeLon = 0.0;
  bool hasValidSafeZone = parseCoordPair(setLocationData, safeLat, safeLon);
  
  if (hasValidSafeZone) {
    Serial.printf("✅ Safe zone parsed successfully: lat=%.8f, lon=%.8f\n", safeLat, safeLon);
  } else {
    Serial.println("❌ Failed to parse safe zone coordinates");
  }
  
  // Get current GPS location (quick fix attempt)
  Serial.println("🛰️ Getting current GPS location...");
  readGPSLocation();
  
  // If GPS fails, try cell location as fallback
  if (!currentData.gpsFixValid) {
    Serial.println("📡 GPS unavailable, trying cell tower location...");
    readCellLocation();
  }
  
  Serial.printf("📍 Current location after read: lat=%.8f, lon=%.8f\n", 
                currentData.latitude, currentData.longitude);
  Serial.printf("   Location fix valid: %s (GPS: %s, Cell: %s)\n", 
                currentData.gpsFixValid ? "YES" : "NO",
                currentData.gpsFixValid && !currentData.usingCGPS ? "YES" : "NO",
                currentData.usingCGPS ? "YES" : "NO");
  
  bool buzzerActivated = false;
  bool solenoidActivated = false;
  
  if (hasValidSafeZone && (currentData.gpsFixValid || currentData.coarseFix)) {
    // Calculate distance from safe zone using Haversine formula (works with GPS or CLBS)
    double distance = calculateDistance(safeLat, safeLon, 
                                       currentData.latitude, currentData.longitude);
    
    Serial.println("\n📊 DISTANCE CALCULATION DEBUG:");
    Serial.printf("   Safe zone coords: %.8f, %.8f\n", safeLat, safeLon);
    Serial.printf("   Current coords:   %.8f, %.8f\n", currentData.latitude, currentData.longitude);
    Serial.printf("   Lat difference:   %.8f degrees\n", currentData.latitude - safeLat);
    Serial.printf("   Lon difference:   %.8f degrees\n", currentData.longitude - safeLon);
    
    // Manual quick approximation for verification (at equator: 1 degree ≈ 111km)
    double approxLatDist = abs(currentData.latitude - safeLat) * 111000.0; // meters
    double approxLonDist = abs(currentData.longitude - safeLon) * 111000.0 * cos(safeLat * PI / 180.0);
    double approxDist = sqrt(approxLatDist * approxLatDist + approxLonDist * approxLonDist);
    
    Serial.printf("   Haversine distance: %.2f meters\n", distance);
    Serial.printf("   Approximate distance: %.2f meters (quick check)\n", approxDist);
    Serial.printf("   Threshold: 100 meters\n");
    Serial.printf("   Decision: %s (distance %.2f %s 100m)\n", 
                  distance < 100.0 ? "SAFE ZONE - SOLENOID" : "CRITICAL - BUZZER",
                  distance,
                  distance < 100.0 ? "<" : ">=");
    
    if (distance < 100.0) {
      // Within safe zone - delivery scenario
      Serial.println("✅ Device within safe zone - Package delivery detected");
      
      // 1. Send delivery notification first (quick, non-blocking)
      sendDeliveryNotification();
      
      // 2. Activate solenoid for 20 seconds (blocking operation)
      Serial.println("\n⏱️ Starting solenoid activation sequence...");
      activateSolenoidForDelivery(20000);
      solenoidActivated = true;
      
    } else {
      // Outside safe zone - security breach
      Serial.println("⚠️ DEVICE OUTSIDE SAFE ZONE - CRITICAL BREACH!");
      
      // 1. Send critical alert to Firebase first (quick, non-blocking)
      sendLockBreachAlert();
      
      // 2. Activate buzzer for 15 seconds (blocking operation)
      Serial.println("\n⏱️ Starting buzzer activation sequence...");
      handleBuzzerActivation(15000);
      buzzerActivated = true;
    }
  } else if (!hasValidSafeZone) {
    Serial.println("⚠️ No valid safe zone coordinates - treating as critical breach");
    // 1. Send alert first
    sendLockBreachAlert();
    // 2. Activate buzzer
    Serial.println("\n⏱️ Starting buzzer activation sequence...");
    handleBuzzerActivation(15000);
    buzzerActivated = true;
  } else if (!currentData.gpsFixValid && !currentData.coarseFix) {
    Serial.println("⚠️ No location fix available (GPS or CLBS) - defaulting to critical breach alert");
    // 1. Send alert first
    sendLockBreachAlert();
    // 2. Activate buzzer
    Serial.println("\n⏱️ Starting buzzer activation sequence...");
    handleBuzzerActivation(15000);
    buzzerActivated = true;
  }
  
  // Log completion of actuator operations
  Serial.println("\n✅ ALL ACTUATOR OPERATIONS COMPLETED");
  if (buzzerActivated) {
    Serial.println("   - Buzzer sequence completed");
  }
  if (solenoidActivated) {
    Serial.println("   - Solenoid sequence completed");
  }
  
  // Return to normal cycle - display will be updated in the main cycle with proper data
  Serial.println("\n✅ Lock breach handling complete - returning to normal cycle");
  Serial.printf("   Lock breach process time: %lu ms\n", millis());
}

// Send delivery notification to Firebase
void sendDeliveryNotification() {
  Serial.println("\n📦 SENDING DELIVERY NOTIFICATION...");
  
  // Create alert JSON payload with all required fields for MotionAlert interface
  String alertData = "{";
  alertData += "\"deviceId\":\"" + actualDeviceID + "\",";
  alertData += "\"location\":\"" + String(currentData.latitude, 6) + ", " + String(currentData.longitude, 6) + "\",";
  alertData += "\"message\":\"📦 Package delivered - Security lock activated\",";
  alertData += "\"timestamp\":" + String(millis()) + ",";
  alertData += "\"type\":\"delivery\"";
  alertData += "}";
  
  // Send to safe alerts path (will show as toast notification)
  String alertPath = "/tracking_box/" + actualDeviceID + "/alerts/safe/" + String(millis());
  
  // Send alert to Firebase
  bool success = sendFirebaseHTTP(alertPath, alertData, "PUT");
  
  if (success) {
    Serial.println("✅ Delivery notification sent to Firebase!");
    Serial.println("   Package delivery will be notified on web dashboard");
  } else {
    Serial.println("❌ Failed to send delivery notification");
  }
}

// Activate solenoid for delivery with specified duration
void activateSolenoidForDelivery(unsigned long duration) {
  Serial.println("\n🔓 ACTIVATING SOLENOID FOR DELIVERY");
  Serial.printf("   Duration: %lu seconds\n", duration / 1000);
  Serial.printf("   Start time: %lu ms\n", millis());
  
  // Activate solenoid
  digitalWrite(SOLENOID_PIN, HIGH);
  rtcSolenoidActive = true;
  rtcSolenoidStartTime = millis();
  
  // Keep solenoid active for specified duration
  unsigned long startTime = millis();
  unsigned long elapsedTime = 0;
  unsigned long lastDebugTime = 0;
  
  while (elapsedTime < duration) {
    elapsedTime = millis() - startTime;
    
    // Print detailed progress every second
    if (elapsedTime - lastDebugTime >= 1000) {
      Serial.printf("🔓 Solenoid active: %lu/%lu seconds (elapsed: %lu ms)\n", 
                    elapsedTime / 1000, duration / 1000, elapsedTime);
      lastDebugTime = elapsedTime;
    }
    
    // Check for interrupts
    if (shouldInterruptOperation()) {
      Serial.printf("⚠️ Solenoid operation interrupted at %lu ms!\n", elapsedTime);
      break;
    }
    
    delay(50); // Small delay to prevent tight loop
  }
  
  // Deactivate solenoid
  digitalWrite(SOLENOID_PIN, LOW);
  rtcSolenoidActive = false;
  rtcSolenoidStartTime = 0;
  
  Serial.printf("🔒 Solenoid deactivated after %lu ms\n", elapsedTime);
  Serial.printf("   End time: %lu ms\n", millis());
  Serial.printf("   Total duration: %.1f seconds\n", elapsedTime / 1000.0);
}

// Handle buzzer activation with proper timing control
void handleBuzzerActivation(unsigned long duration) {
  Serial.println("\n🔊 ACTIVATING BUZZER");
  Serial.printf("   Duration: %lu seconds\n", duration / 1000);
  Serial.printf("   Start time: %lu ms\n", millis());
  
  // Activate buzzer
  digitalWrite(BUZZER_PIN, HIGH);
  
  unsigned long startTime = millis();
  unsigned long elapsedTime = 0;
  unsigned long lastDebugTime = 0;
  
  while (elapsedTime < duration) {
    elapsedTime = millis() - startTime;
    
    // Print debug info every second
    if (elapsedTime - lastDebugTime >= 1000) {
      Serial.printf("🔊 Buzzer active: %lu/%lu seconds\n", 
                    elapsedTime / 1000, duration / 1000);
      lastDebugTime = elapsedTime;
    }
    
    // Check for interrupts
    if (shouldInterruptOperation()) {
      Serial.println("⚠️ Buzzer operation interrupted!");
      break;
    }
    
    delay(50); // Small delay to prevent tight loop
  }
  
  // Deactivate buzzer
  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.printf("🔇 Buzzer deactivated after %lu ms\n", elapsedTime);
  Serial.printf("   End time: %lu ms\n", millis());
}

// Send critical lock breach alert to Firebase
void sendLockBreachAlert() {
  Serial.println("\n🚨 SENDING CRITICAL LOCK BREACH ALERT TO FIREBASE...");
  
  // Create alert JSON payload with all required fields for MotionAlert interface
  String alertData = "{";
  alertData += "\"deviceId\":\"" + actualDeviceID + "\",";
  alertData += "\"location\":\"" + String(currentData.latitude, 6) + ", " + String(currentData.longitude, 6) + "\",";
  alertData += "\"message\":\"⚠️ CRITICAL: Lock breach detected - device outside safe zone\",";
  alertData += "\"timestamp\":" + String(millis()) + ",";
  alertData += "\"type\":\"lock_breach\"";
  alertData += "}";
  
  // Send to critical alerts path
  String alertPath = "/tracking_box/" + actualDeviceID + "/alerts/critical/" + String(millis());
  
  // Send alert to Firebase
  bool success = sendFirebaseHTTP(alertPath, alertData, "PUT");
  
  if (success) {
    Serial.println("✅ Critical lock breach alert sent to Firebase!");
    Serial.println("   Alert will trigger notification on web dashboard");
  } else {
    Serial.println("❌ Failed to send critical lock breach alert to Firebase");
  }
}