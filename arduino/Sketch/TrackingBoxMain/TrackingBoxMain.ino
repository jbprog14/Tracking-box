/*
 * =====================================================================
 * TRACKING BOX DEVICE - FINALIZED FIRMWARE
 * =====================================================================
 * 
 * This sketch implements a streamlined, single-cycle operation for the
 * tracking device. On every wake-up, it performs the following:
 * 1. Initialize all hardware.
 * 2. Gather a full set of sensor readings.
 * 3. Connect to WiFi.
 * 4. Send sensor data to the Firebase database.
 * 5. Fetch the latest device details from Firebase.
 * 6. Update the E-Ink display with all data. This action queues the
 *    device to immediately enter deep sleep.
 * 
 * The device wakes from deep sleep based on three triggers:
 * - A 15-minute timer.
 * - The box lid being opened (limit switch).
 * - A significant shock or movement detected by the LSM6DSL accelerometer.
 * 
 * =====================================================================
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_LSM6DSL.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "DEV_Config.h"
#include "EPD.h"
#include "GUI_Paint.h"
#include <math.h>  // for haversine

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
// NETWORK & FIREBASE CONFIGURATION
// =====================================================================
const char* WIFI_SSID = "archer_2.4G";
const char* WIFI_PASSWORD = "05132000";
const char* FIREBASE_HOST = "https://tracking-box-e17a1-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* FIREBASE_AUTH = "AIzaSyBQje281bPAt7MiJdK94ru1irAU8i3luzY";
const String DEVICE_ID = "box_001";

// =====================================================================
// GLOBAL OBJECTS & VARIABLES
// =====================================================================
#if ENABLE_SHT30_SENSOR
Adafruit_SHT31 sht30 = Adafruit_SHT31();
#endif
Adafruit_LSM6DSL lsm6ds = Adafruit_LSM6DSL();
HardwareSerial sim7600(1);
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

bool parseCoordPair(const String &str, double &lat, double &lon) {
  int comma = str.indexOf(',');
  if (comma == -1) return false;
  lat = str.substring(0, comma).toDouble();
  lon = str.substring(comma + 1).toDouble();
  return !(isnan(lat) || isnan(lon));
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
  String deviceSetLocation = "Unknown";
  String deviceDescription = "No Description";
  String wakeUpReason = "Power On";
  bool buzzerIsActive = false;
  bool buzzerDismissed = false;
  bool solenoidActive = false; // NEW: current requested state from Firebase
};

TrackerData currentData;

// Flag to request a restart after solenoid operation completes
bool restartAfterSolenoid = false;

// Forward declaration
void determineWakeUpReason();
void updateDisplay();

// =====================================================================
// MAIN SETUP (single cycle) – call new E-ink init just before display
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===== WAKING UP =====");

  determineWakeUpReason();

  if (connectToWiFi()) {
    Serial.println("✅ WiFi Connected.");
    initializeAllHardware();
    Serial.println("✅ Hardware Initialized.");
    collectSensorReading();
    Serial.println("✅ Sensor Readings Collected.");
    fetchDeviceDetailsFromFirebase();
    fetchBuzzerStateFromFirebase();
    fetchSolenoidStateFromFirebase(); // NEW: read solenoid flag
    evaluateLockBreach();
    // If solenoid flag is set, activate the lock once
    if (currentData.solenoidActive) {
      activateSolenoidAndClearFlag();
      restartAfterSolenoid = true; // request reboot after this cycle
    }
    sendSensorDataToFirebase();
    Serial.println("✅ Sensor Data Sent to Firebase.");

    // ------------------------------------------------------------------
    // Update E-Ink display with the latest data pulled from Firebase and
    // gathered from onboard sensors.
    // ------------------------------------------------------------------
    updateDisplay();

    // If buzzer is active, enter monitoring loop until dismissed
    handleBuzzerMonitoring();
    if (!currentData.buzzerIsActive) {
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
    }
    // -------------------------------------------------------------
    // If solenoid cycle finished, restart instead of deep sleep
    // -------------------------------------------------------------
    if (restartAfterSolenoid) {
      Serial.println("Restarting MCU after solenoid activation …");
      delay(500);
      ESP.restart();
    }
  } else {
    Serial.println("❌ WiFi connection failed.");
  }

  Serial.println("Cycle complete → deep sleep.");
  prepareForDeepSleep();
  esp_deep_sleep_start();
}

void loop() {
  // The loop is intentionally left empty.
  // The device performs a single cycle in setup() and then deep sleeps.
}

void sendSensorDataToFirebase() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("✗ Cannot send data, WiFi not connected.");
    return;
  }

  HTTPClient http;
  // Use PUT to replace the sensorData node directly
  String url = String(FIREBASE_HOST) + "/tracking_box/" + DEVICE_ID + "/sensorData.json?auth=" + FIREBASE_AUTH;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  // Create JSON document for the PUT request
  DynamicJsonDocument doc(1024);
  
  doc["timestamp"] = millis();
  doc["temp"] = currentData.temperature;
  doc["humidity"] = currentData.humidity;
  doc["batteryVoltage"] = currentData.batteryVoltage;
  doc["gpsFixValid"] = currentData.gpsFixValid;
  doc["usingCGPS"] = currentData.usingCGPS; // NEW flag pushed to Firebase
  doc["limitSwitchPressed"] = currentData.limitSwitchPressed;
  doc["tiltDetected"] = currentData.tiltDetected;
  doc["fallDetected"] = currentData.fallDetected;
  doc["wakeUpReason"] = currentData.wakeUpReason;
  doc["buzzerIsActive"] = currentData.buzzerIsActive;
  
  JsonObject accelerometer = doc.createNestedObject("accelerometer");
  accelerometer["x"] = currentData.accelX;
  accelerometer["y"] = currentData.accelY;
  accelerometer["z"] = currentData.accelZ;
  
  if (currentData.gpsFixValid) {
    JsonObject location = doc.createNestedObject("location");
    location["lat"] = currentData.latitude;
    location["lng"] = currentData.longitude;
    location["alt"] = currentData.altitude;
  }
  
  doc["currentLocation"] = currentData.currentLocation;

  // Serialize JSON to string for sending
  String jsonPayload;
  serializeJson(doc, jsonPayload);
  Serial.println("Sending to Firebase: " + jsonPayload);

  // Send the PUT request
  int httpResponseCode = http.PUT(jsonPayload);

  if (httpResponseCode > 0) {
    Serial.printf("✓ Firebase PUT successful, response code: %d\n", httpResponseCode);
  } else {
    Serial.printf("✗ Firebase PUT failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
  }

  http.end();
}

void fetchDeviceDetailsFromFirebase() {
  HTTPClient http;
  // Fetch only the 'details' node to be efficient
  String url = String(FIREBASE_HOST) + "/tracking_box/" + DEVICE_ID + "/details.json?auth=" + FIREBASE_AUTH;
  http.begin(url);
  
  int httpResponseCode = http.GET();
  if (httpResponseCode == HTTP_CODE_OK) {
    String response = http.getString();
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, response);
    
    if (!error) {
      currentData.deviceName = doc["name"] | "Unknown";
      currentData.deviceSetLocation = doc["setLocation"] | "Unknown";
      currentData.deviceDescription = doc["description"] | "No Description";
      Serial.println("✓ Fetched device details from Firebase.");
    } else {
      Serial.println("✗ Failed to parse device details JSON.");
    }
  } else {
    Serial.printf("✗ Firebase details fetch failed, code: %d\n", httpResponseCode);
  }
  http.end();
}

void fetchBuzzerStateFromFirebase() {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  // Fetch only the 'buzzerDismissed' field from the 'sensorData' node.
  String url = String(FIREBASE_HOST) + "/tracking_box/" + DEVICE_ID + "/sensorData/buzzerDismissed.json?auth=" + FIREBASE_AUTH;
  http.begin(url);
  
  int httpResponseCode = http.GET();
  if (httpResponseCode == HTTP_CODE_OK) {
    String response = http.getString();
    // Firebase returns a simple 'true' or 'false' for a boolean leaf, not a JSON object.
    if (response == "true") {
      currentData.buzzerDismissed = true;
      Serial.println("✓ Buzzer state is DISMISSED.");
    } else {
      currentData.buzzerDismissed = false;
    }
  } else {
    // If the fetch fails or the key doesn't exist, assume not dismissed.
    currentData.buzzerDismissed = false;
  }
  http.end();
}

// ---------------------------------------------------------------------------
// SOLENOID CONTROL
// ---------------------------------------------------------------------------
void fetchSolenoidStateFromFirebase() {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  String url = String(FIREBASE_HOST) + "/tracking_box/" + DEVICE_ID + "/solenoid.json?auth=" + FIREBASE_AUTH;
  http.begin(url);
  int code = http.GET();
  if (code == HTTP_CODE_OK) {
    String payload = http.getString();
    currentData.solenoidActive = (payload == "true");
  } else {
    currentData.solenoidActive = false;
  }
  http.end();
}

void activateSolenoidAndClearFlag() {
  Serial.println("🔓 Activating solenoid lock …");
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, HIGH); // energise solenoid
  delay(10000);                      // hold for 10 s (adjust as needed)
  digitalWrite(SOLENOID_PIN, LOW);  // release

  // Clear the flag in Firebase so we don’t fire again next cycle
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(FIREBASE_HOST) + "/tracking_box/" + DEVICE_ID + "/solenoid.json?auth=" + FIREBASE_AUTH;
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    http.PUT("false");
    http.end();
  }
  currentData.solenoidActive = false;
  Serial.println("🔒 Solenoid cycle complete.");
}

void evaluateLockBreach() {
  // A lock breach occurs if the lid is open AND the device's physical location
  // does not match its designated location from Firebase.
  // Note: Limit switch is pressed (true) when the lid is CLOSED.
  bool isLidOpen = !currentData.limitSwitchPressed;

  bool isLocationMismatch = true; // assume mismatch until proven otherwise

  // If we have a valid GPS fix and a parsable setLocation, compute distance
  if (currentData.gpsFixValid) {
    double setLat, setLon;
    if (parseCoordPair(currentData.deviceSetLocation, setLat, setLon)) {
      double distMeters = haversineMeters(currentData.latitude, currentData.longitude, setLat, setLon);
      isLocationMismatch = distMeters > 100.0; // 100-m radius
      Serial.printf("Distance to safe zone: %.1f m\n", distMeters);
    }
  }

  bool lockBreach = isLidOpen && isLocationMismatch;

  // If a new breach is detected, reset the dismissal flag in Firebase.
  if (lockBreach && currentData.buzzerDismissed) {
    Serial.println("🔄 New lock breach detected. Resetting dismissal state.");
    updateBuzzerStateInFirebase(false, false);
    currentData.buzzerDismissed = false;
  }
  
  // Activate the buzzer only if there is a breach and it has NOT been dismissed.
  if (lockBreach && !currentData.buzzerDismissed) {
    Serial.println("🚨 LOCK BREACH DETECTED!");
    currentData.wakeUpReason   = "LOCK BREACH";
    currentData.buzzerIsActive = true;
    digitalWrite(BUZZER_PIN, HIGH);  // Sound buzzer continuously
  } else {
    // If no breach or already dismissed, ensure buzzer is off.
    currentData.buzzerIsActive = false;
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void handleBuzzerMonitoring() {
  if (!currentData.buzzerIsActive) return;

  Serial.println("🚨 LOCK BREACH active – monitoring Firebase for dismissal…");
  unsigned long lastPoll = 0;
  
  // Stay in this loop as long as the buzzer is active, polling for dismissal
  while (currentData.buzzerIsActive) {
    if (millis() - lastPoll > 5000) { // Poll every 5 seconds
      lastPoll = millis();
      if (checkDismissCommand()) {
        Serial.println("🔕 Dismiss command received – stopping buzzer.");
        digitalWrite(BUZZER_PIN, LOW);
        currentData.buzzerIsActive = false;
        currentData.buzzerDismissed = true; // Flag that it was dismissed
        
        // Send final state update to Firebase before sleeping
        updateBuzzerStateInFirebase(false, true);
        break; // Exit monitoring loop and proceed to deep sleep
      }
    }
    delay(100); // Small delay to prevent busy-waiting
  }
}

// =====================================================================
// SENSOR READING FUNCTIONS
// =====================================================================
void collectSensorReading() {
  readTemperatureHumidity();
  readAccelerometerData();
  readGPSLocation();
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

  if (currentData.gpsFixValid) {
    currentData.currentLocation = String(currentData.latitude, 4) + ", " + String(currentData.longitude, 4);
  } else {
    currentData.currentLocation = "No GPS Fix";
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
  
  // ------------------------------------------------------------------
  // Enable GPS using the CGPS command set (more reliable on SIM7600).
  // ------------------------------------------------------------------
  sendGPSCommand("AT+CGPS=1,1");   // Start GPS in standalone mode
  delay(2000);                      // Allow the receiver to power-up

  // Request current location data
  flushSIM7600Buffer();
  sim7600.println("AT+CGPSINFO");
  
  String response = waitForGPSResponse(5000);

  if (response.indexOf("+CGPSINFO:") != -1) {
    if (isValidGPSFix(response)) {
      // CGPSINFO format: +CGPSINFO: <lat>,<N/S>,<lon>,<E/W>,<date>,<utc>,<alt>,<speed>,<course>
      String latitude      = extractGPSField(response, 1);
      String lat_direction = extractGPSField(response, 2);
      String longitude     = extractGPSField(response, 3);
      String lon_direction = extractGPSField(response, 4);
      String altitude      = extractGPSField(response, 7); // altitude field (may be blank)

      // Convert to decimal degrees
      currentData.latitude  = convertToDecimalDegrees(latitude,  lat_direction);
      currentData.longitude = convertToDecimalDegrees(longitude, lon_direction);
      currentData.altitude  = altitude.toFloat();
      currentData.gpsFixValid = true;
      Serial.println("✓ GPS fix acquired");
    } else {
      Serial.println("⚠ No GPS fix available");
      currentData.gpsFixValid = false;
    }
  } else {
    Serial.println("✗ GPS communication error");
    currentData.gpsFixValid = false;
  }
}

void readBatteryVoltage() {
  // Read battery voltage through ADC with voltage divider
  int adcReading = analogRead(BATTERY_ADC_PIN);
  currentData.batteryVoltage = (adcReading * 3.3 * 2.0) / 4095.0;
}

// =====================================================================
// HARDWARE INITIALIZATION
// =====================================================================
bool initializeAllHardware() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
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

  // Configure SIM7600 to supply power to the active GNSS antenna
  Serial.println("Configuring active GNSS antenna power...");
  sim7600.println("AT+CVAUXV=3050"); // Set auxiliary voltage to 3.05V
  delay(200);
  sim7600.println("AT+CVAUXS=1");   // Enable auxiliary voltage output
  delay(200);
  flushSIM7600Buffer(); // Clear any "OK" responses
  Serial.println("✓ Active antenna power configured.");

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
// WIFI & FIREBASE COMMUNICATION
// =====================================================================
bool connectToWiFi() {
  Serial.println("Connecting to WiFi network...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int connectionAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && connectionAttempts < 30) {
    delay(500);
    Serial.print(".");
    connectionAttempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("✓ WiFi connected successfully");
    Serial.println("IP Address: " + WiFi.localIP().toString());
    return true;
  } else {
    Serial.println("");
    Serial.println("✗ WiFi connection failed");
    return false;
  }
}

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

  // Wake up on timer
  esp_sleep_enable_timer_wakeup(SLEEP_TIME_US);
  Serial.println("  - Timer wakeup enabled for 15 minutes.");

  // Wake up on motion (LSM6DSL INT1 is on GPIO34)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 1); // RTC_GPIO 0, wake on HIGH
  Serial.println("  - Motion wakeup (EXT0) on GPIO 34 enabled.");

  // Wake up on limit switch (lid open - pin is LOW when open)
  uint64_t limitSwitchMask = 1ULL << LIMIT_SWITCH_PIN;
  esp_sleep_enable_ext1_wakeup(limitSwitchMask, ESP_EXT1_WAKEUP_ANY_HIGH); // should be ALL_LOW if switch pulls to gnd when open
  Serial.println("  - Limit switch wakeup (EXT1) on GPIO 33 enabled.");
  
  // Isolate GPIO12 pin from external circuits during deep sleep to prevent flash issues.
  rtc_gpio_isolate(GPIO_NUM_12);
}


// =====================================================================
// E-PAPER DISPLAY – Waveshare 7.3" 800×400 (7-color) helper
// =====================================================================
void updateDisplay() {
  Serial.println("Updating E-Ink display …");

  // Basic initialisation – safe to call each cycle
  DEV_Module_Init();
  EPD_7IN3F_Init();
  EPD_7IN3F_Clear(EPD_7IN3F_WHITE);

  // Allocate a quarter-frame buffer (800×200) like the Waveshare demo
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
  // Compose display content
  // ------------------------------
  uint16_t y = 5;  // Start Y-coord
  const uint16_t lineGap = 28;

   // Details
  Paint_DrawString_EN(10, y, String("DETAILS:").c_str(), &Font24, EPD_7IN3F_WHITE, EPD_7IN3F_RED);
  y += lineGap + 6;

  // Device name (large font)
  char buf[64];
  snprintf(buf, sizeof(buf), "PACKAGE OWNER: %s", currentData.deviceName);
  Paint_DrawString_EN(10, y, buf, &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK );
  y += lineGap;

  // Current Location
  snprintf(buf, sizeof(buf), "CURRENT LOCATION: %s", currentData.currentLocation.c_str());
  Paint_DrawString_EN(10, y, buf, &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK );
  y += lineGap;

  // Item Description
  snprintf(buf, sizeof(buf), "ITEM DESCRIPTION: %s", currentData.deviceDescription);
  Paint_DrawString_EN(10, y, buf, &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK );
  y += lineGap + 6;

  // Sensor Readings
  Paint_DrawString_EN(10, y, String("SENSOR READINGS:").c_str(), &Font24, EPD_7IN3F_WHITE, EPD_7IN3F_RED );
  y += lineGap + 10;

  // Temperature | Accel
  snprintf(buf, sizeof(buf), "TEMPERATURE: %.1fC | ACCELEROMETER: X=%.2fg  Y=%.2fg  Z=%.2fg", currentData.temperature, currentData.accelX, currentData.accelY, currentData.accelZ);
  Paint_DrawString_EN(10, y, buf, &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK );
  y += lineGap;

  // Humidity | Battery
  snprintf(buf, sizeof(buf), "HUMIDITY: %.1f%% | BATTERY (V): %.2fV", currentData.humidity, currentData.batteryVoltage);
   Paint_DrawString_EN(10, y, buf, &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK );
   y += lineGap;

  // wake-up reason
  snprintf(buf, sizeof(buf), "LAST WAKE-UP REASON: %s", currentData.wakeUpReason.c_str());
  Paint_DrawString_EN(10, y, buf, &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK );
  y += lineGap;

  // ------------------------------
  // Push buffer to display and sleep
  // ------------------------------
  EPD_7IN3F_DisplayPart(imgBuf, 0, 0, EPD_7IN3F_WIDTH, EPD_7IN3F_HEIGHT / 2);
  EPD_7IN3F_Sleep();
  free(imgBuf);
  imgBuf = nullptr;

  Serial.println("✓ Display updated");
}

// =====================================================================
// END OF TRACKING BOX MAIN FIRMWARE
// ===================================================================== 

bool checkDismissCommand() {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  String url = String(FIREBASE_HOST) + "/tracking_box/" + DEVICE_ID + "/dismissAlert.json?auth=" + FIREBASE_AUTH;
  http.begin(url);
  int code = http.GET();
  if (code == HTTP_CODE_OK) {
    String payload = http.getString();
    DynamicJsonDocument doc(128);
    if (deserializeJson(doc, payload) == DeserializationError::Ok) {
      bool dismissed = doc["dismissed"] | false;
      http.end();
      return dismissed;
    }
  }
  http.end();
  return false;
}

void updateBuzzerStateInFirebase(bool active, bool dismissed) {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  String url = String(FIREBASE_HOST) + "/tracking_box/" + DEVICE_ID + "/sensorData.json?auth=" + FIREBASE_AUTH;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  DynamicJsonDocument doc(128);
  doc["buzzerIsActive"] = active;
  doc["buzzerDismissed"] = dismissed;
  String body; serializeJson(doc, body);
  http.PATCH(body);
  http.end();
} 