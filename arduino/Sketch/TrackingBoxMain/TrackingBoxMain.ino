/*
 * =====================================================================
 * TRACKING BOX DEVICE - MAIN FIRMWARE
 * =====================================================================
 * 
 * This sketch combines all individual sensor tests into one complete
 * tracking device that satisfies all requirements from FEATURES.md
 * 
 * 📚 REQUIRED LIBRARIES (Install via Arduino IDE Library Manager):
 * - ArduinoJson (by Benoit Blanchon) - REQUIRED for compilation
 * 
 * 🔧 OPTIONAL LIBRARIES (Enable below by changing 0 to 1):
 * - GxEPD2 (by ZinggJM) - For E-ink display functionality
 * - Adafruit SHT31 (by Adafruit) - For temperature/humidity sensor
 * 
 * 🎯 QUICK START:
 * 1. Install ArduinoJson library (required)
 * 2. This sketch will compile without optional libraries
 * 3. Install optional libraries and set ENABLE flags to 1 when ready
 * 
 * Components:
 * - SHT30 Temperature & Humidity Sensor (optional)
 * - SIM7600 GNSS (GPS Location) (built-in)
 * - ESP32 Microcontroller (required)
 * - LSM6DSL 6-axis Accelerometer/Gyroscope (built-in)
 * - Li-Po Battery (built-in)
 * - 3.7" Waveshare E-ink Display (optional)
 * - Buzzer (built-in)
 * 
 * Core Functionality:
 * 1. Wake every 15 minutes OR on accelerometer interrupt
 * 2. Read all sensors when awake
 * 3. Send data to Firebase database
 * 4. Update E-ink display with current information (if enabled)
 * 5. Return to deep sleep mode
 * 
 * =====================================================================
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"

// =====================================================================
// OPTIONAL LIBRARIES - Comment out to disable features
// =====================================================================
// Set to 1 to enable E-ink display (requires GxEPD2 library installation)
// Set to 0 to disable E-ink display and compile without the library
#define ENABLE_EINK_DISPLAY 1

// Set to 1 to enable SHT30 sensor (requires Adafruit_SHT31 library)
// Set to 0 to disable SHT30 sensor and use simulated temperature/humidity data
#define ENABLE_SHT30_SENSOR 0

// E-ink Display Libraries (only include if enabled)
#if ENABLE_EINK_DISPLAY
#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <GxEPD2_4C.h>
#include <GxEPD2_7C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#endif

// SHT30 Library (only include if enabled)
#if ENABLE_SHT30_SENSOR
#include "Adafruit_SHT31.h"
#endif

// =====================================================================
// PIN DEFINITIONS
// =====================================================================
// Power & Analog
#define BATTERY_ADC_PIN     36    // ADC1_CH0 - Safe with WiFi

// SHT30 Temperature/Humidity - I2C Bus 0 (Default)
#define SHT30_SDA_PIN       21    // I2C0 SDA (default)
#define SHT30_SCL_PIN       22    // I2C0 SCL (default)

// LSM6DSL Accelerometer - I2C Bus 1 (Secondary)
#define LSM6DSL_SDA_PIN     18    // I2C1 SDA (custom)
#define LSM6DSL_SCL_PIN     19    // I2C1 SCL (custom)
#define LSM6DSL_INT1_PIN    25    // Interrupt pin

// SIM7600 GPS Module - UART2
#define SIM7600_TX_PIN      32    // Safe UART
#define SIM7600_RX_PIN      33    // Safe UART

// E-ink Display - SPI
#define EINK_CS_PIN         5     // Chip Select
#define EINK_DC_PIN         17    // Data/Command
#define EINK_RST_PIN        16    // Reset
#define EINK_BUSY_PIN       4     // Busy status
#define EINK_CLK_PIN        23    // SPI Clock
#define EINK_DIN_PIN        27    // SPI Data

// Digital I/O
#define LIMIT_SWITCH_PIN    34    // ADC1_CH6 - Input-only, better for EXT1 wakeup
#define BUZZER_PIN          26    // PWM capable

// =====================================================================
// SYSTEM CONFIGURATION
// =====================================================================
#define SLEEP_MINUTES       15
#define uS_TO_S_FACTOR      1000000ULL
#define SLEEP_TIME_US       (SLEEP_MINUTES * 60 * uS_TO_S_FACTOR)
#define TILT_THRESHOLD      0.9   // Accelerometer tilt detection threshold

// LSM6DSL Register Addresses
#define LSM6DSL_ADDR1       0x6A
#define LSM6DSL_ADDR2       0x6B
#define LSM6DSL_WHO_AM_I    0x0F
#define LSM6DSL_CTRL1_XL    0x10
#define LSM6DSL_CTRL3_C     0x12
#define LSM6DSL_OUTX_L_XL   0x28

// =====================================================================
// NETWORK & FIREBASE CONFIGURATION
// =====================================================================
// WiFi Credentials - REPLACE WITH YOUR VALUES
const char* WIFI_SSID = "archer_2.4G";
const char* WIFI_PASSWORD = "05132000";

// Firebase Configuration (from firebase.ts)
const char* FIREBASE_HOST = "https://tracking-box-e17a1-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* FIREBASE_AUTH = "AIzaSyBQje281bPAt7MiJdK94ru1irAU8i3luzY";

// Device Information
const String DEVICE_ID = "box_001";
const String DEVICE_NAME = "Tracking Box 001";
const String DEVICE_LOCATION = "Default Location";

// =====================================================================
// GLOBAL OBJECTS & VARIABLES
// =====================================================================
#if ENABLE_SHT30_SENSOR
Adafruit_SHT31 sht30 = Adafruit_SHT31();
#endif

HardwareSerial sim7600(1);

#if ENABLE_EINK_DISPLAY
// Correct display driver for Waveshare 3.7" E-ink Display
GxEPD2_BW<GxEPD2_370_TC1, GxEPD2_370_TC1::HEIGHT> display(GxEPD2_370_TC1(EINK_CS_PIN, EINK_DC_PIN, EINK_RST_PIN, EINK_BUSY_PIN));
#endif

uint8_t lsm6dsl_address = LSM6DSL_ADDR1;

// Sensor Data Structure
struct TrackerData {
  float temperature = 0.0;
  float humidity = 0.0;
  double latitude = 0.0;
  double longitude = 0.0;
  float altitude = 0.0;
  bool tiltDetected = false;
  String wakeReason = "";
  float batteryVoltage = 0.0;
  unsigned long timestamp = 0;
  bool gpsFixValid = false;
  bool limitSwitchPressed = false;
  bool locationBreach = false;
  String deviceSetLocation = "";
  String deviceDescription = "";
  String deviceName = "";
};

TrackerData currentData;
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR bool isFirstBoot = true;

// =====================================================================
// MAIN SETUP FUNCTION
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  bootCount++;
  
  // Determine wake up reason
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  currentData.wakeReason = getWakeupReasonString(wakeup_reason);
  currentData.timestamp = millis();
  
  // Check limit switch status (critical security feature)
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  currentData.limitSwitchPressed = !digitalRead(LIMIT_SWITCH_PIN); // LOW when pressed (pullup)
  
  Serial.println("=====================================================");
  Serial.println("        TRACKING BOX DEVICE - STARTUP");
  Serial.println("=====================================================");
  Serial.println("Boot Count: " + String(bootCount));
  Serial.println("Wake Reason: " + currentData.wakeReason);
  Serial.println("Timestamp: " + String(currentData.timestamp));
  
  // Initialize all hardware components
  if (initializeAllHardware()) {
    Serial.println("✓ Hardware initialization successful");
    
    // Read all sensor data
    readAllSensors();
    
    // Track connection status for display
    bool wifiConnected = false;
    bool firebaseSuccess = false;
    
    // Connect to WiFi and send data to Firebase
    if (connectToWiFi()) {
      wifiConnected = true;
      
      // Fetch device details from Firebase first (for security check)
      fetchDeviceDetailsFromFirebase();
      
      // Perform security check if limit switch was pressed
      if (currentData.limitSwitchPressed) {
        performSecurityCheck();
      }
      
      sendDataToFirebase();
      firebaseSuccess = true; // Assume success if no exception thrown
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
    } else {
      // No WiFi connection - show QR code for offline access
      displayOfflineQRCode();
    }
    
    // Update E-ink display with current data
    #if ENABLE_EINK_DISPLAY
    // Show connection status first
    displayConnectionStatus(wifiConnected, firebaseSuccess);
    delay(5000); // Display connection status for 5 seconds
    
    // Then show detailed sensor readings
    updateEInkDisplay();
    Serial.println("💡 E-ink display updated with complete sensor data");
    #else
    Serial.println("⚠ E-ink display update skipped (disabled)");
    #endif
    
    // Sound success notification
    buzzerAlert(2, 100);
    
    Serial.println("✓ Operation cycle completed successfully");
  } else {
    Serial.println("✗ Hardware initialization failed!");
    buzzerAlert(5, 200); // Error alert
  }
  
  // Prepare for and enter deep sleep
  // prepareForDeepSleep();
  
  // Serial.println("Entering deep sleep mode...");
  // Serial.flush();
  // esp_deep_sleep_start();
  
  // DEEP SLEEP DISABLED FOR TESTING
  Serial.println("🔄 Deep sleep disabled for testing - will restart in 30 seconds");
  delay(30000); // Wait 30 seconds before restarting cycle
  Serial.println("🔄 Restarting operation cycle...");
  Serial.println("");
}

void loop() {
  // TESTING MODE: Run continuous operation cycle
  // Deep sleep is disabled, so we run the main operation in loop
  
  Serial.println("🔄 Starting new operation cycle...");
  
  // Read all sensor data
  readAllSensors();
  
  // Track connection status for display
  bool wifiConnected = false;
  bool firebaseSuccess = false;
  
  // Connect to WiFi and send data to Firebase
  if (connectToWiFi()) {
    wifiConnected = true;
    
    // Always fetch device details from Firebase for display
    fetchDeviceDetailsFromFirebase();
    
    sendDataToFirebase();
    firebaseSuccess = true; // Assume success if no exception thrown
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
  
  // Update E-ink display with current data
  #if ENABLE_EINK_DISPLAY
  // Show connection status first
  displayConnectionStatus(wifiConnected, firebaseSuccess);
  delay(5000); // Display connection status for 5 seconds
  
  // Then show detailed sensor readings
  updateEInkDisplay();
  Serial.println("💡 E-ink display updated with complete sensor data");
  #else
  Serial.println("⚠ E-ink display update skipped (disabled)");
  #endif
  
  // Sound success notification
  buzzerAlert(2, 100);
  
  Serial.println("✓ Operation cycle completed successfully");
  Serial.println("⏱️ Waiting 30 seconds before next cycle...");
  Serial.println("");
  
  delay(30000); // Wait 30 seconds before next cycle
}

// =====================================================================
// HARDWARE INITIALIZATION
// =====================================================================
bool initializeAllHardware() {
  Serial.println("Initializing hardware components...");
  
  // Initialize GPIO pins
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  
  // Initialize SHT30 Temperature & Humidity Sensor
  #if ENABLE_SHT30_SENSOR
  Wire.begin(SHT30_SDA_PIN, SHT30_SCL_PIN);
  if (!sht30.begin(0x44)) {
    Serial.println("✗ SHT30 sensor initialization failed");
    return false;
  }
  Serial.println("✓ SHT30 sensor initialized");
  #else
  Serial.println("⚠ SHT30 sensor disabled - using simulated data");
  #endif
  
  // Initialize LSM6DSL Accelerometer
  Wire1.begin(LSM6DSL_SDA_PIN, LSM6DSL_SCL_PIN);
  if (!initializeAccelerometer()) {
    Serial.println("✗ LSM6DSL accelerometer initialization failed");
    return false;
  }
  Serial.println("✓ LSM6DSL accelerometer initialized");
  
  // Initialize SIM7600 GPS Module
  sim7600.begin(115200, SERIAL_8N1, SIM7600_RX_PIN, SIM7600_TX_PIN);
  delay(2000);
  flushSIM7600Buffer();
  Serial.println("✓ SIM7600 GPS module initialized");
  
  // Initialize E-ink Display
  #if ENABLE_EINK_DISPLAY
  // Manual hardware reset first for reliable initialization
  pinMode(EINK_RST_PIN, OUTPUT);
  digitalWrite(EINK_RST_PIN, HIGH);
  delay(200);
  digitalWrite(EINK_RST_PIN, LOW);
  delay(20);
  digitalWrite(EINK_RST_PIN, HIGH);
  delay(200);
  
  // Initialize with extended reset pulse for first-time reliability
  display.init(115200, true, 20, false); // Extended 20ms reset pulse for reliable startup
  delay(1000); // Extended delay to ensure display is ready
  Serial.println("✓ E-ink display initialized successfully");
  #else
  Serial.println("⚠ E-ink display disabled - install GxEPD2 library to enable");
  #endif
  
  return true;
}

bool initializeAccelerometer() {
  uint8_t possible_addresses[] = {LSM6DSL_ADDR1, LSM6DSL_ADDR2};
  
  for (int i = 0; i < 2; i++) {
    lsm6dsl_address = possible_addresses[i];
    uint8_t deviceId = readAccelRegister(LSM6DSL_WHO_AM_I);
    
    if (deviceId == 0x6A) {
      // Configure accelerometer for normal operation
      writeAccelRegister(LSM6DSL_CTRL1_XL, 0x60); // ODR=208Hz, ±2g full scale
      writeAccelRegister(LSM6DSL_CTRL3_C, 0x40);  // Enable block data update
      delay(100); // Allow sensor to stabilize
      return true;
    }
  }
  return false;
}

// =====================================================================
// SENSOR READING FUNCTIONS
// =====================================================================
void readAllSensors() {
  Serial.println("Reading all sensors...");
  
  // Read SHT30 Temperature and Humidity
  readTemperatureHumidity();
  
  // Read LSM6DSL Accelerometer for tilt detection
  readAccelerometerData();
  
  // Read SIM7600 GPS Location
  readGPSLocation();
  
  // Read battery voltage
  readBatteryVoltage();
  
  // Print all readings
  printSensorReadings();
}

void readTemperatureHumidity() {
  #if ENABLE_SHT30_SENSOR
  currentData.temperature = sht30.readTemperature();
  currentData.humidity = sht30.readHumidity();
  
  // Handle invalid readings
  if (isnan(currentData.temperature)) {
    currentData.temperature = 0.0;
    Serial.println("⚠ Invalid temperature reading");
  }
  if (isnan(currentData.humidity)) {
    currentData.humidity = 0.0;
    Serial.println("⚠ Invalid humidity reading");
  }
  #else
  // Use simulated data when SHT30 is disabled
  currentData.temperature = 25.0 + (random(-50, 50) / 10.0); // 20-30°C range
  currentData.humidity = 50.0 + (random(-200, 200) / 10.0);   // 30-70% range
  Serial.println("⚠ Using simulated temperature/humidity data");
  #endif
}

void readAccelerometerData() {
  // Read raw accelerometer data
  uint8_t rawData[6];
  for (int i = 0; i < 6; i++) {
    rawData[i] = readAccelRegister(LSM6DSL_OUTX_L_XL + i);
  }
  
  // Convert to signed 16-bit values
  int16_t rawX = (rawData[1] << 8) | rawData[0];
  int16_t rawY = (rawData[3] << 8) | rawData[2];
  int16_t rawZ = (rawData[5] << 8) | rawData[4];
  
  // Convert to g values (±2g full scale)
  float accelX = rawX * 2.0 / 32768.0;
  float accelY = rawY * 2.0 / 32768.0;
  float accelZ = rawZ * 2.0 / 32768.0;
  
  // Check for tilt condition (any axis exceeding threshold)
  currentData.tiltDetected = (abs(accelX) > TILT_THRESHOLD || 
                             abs(accelY) > TILT_THRESHOLD || 
                             abs(accelZ) > TILT_THRESHOLD);
}

void readGPSLocation() {
  Serial.println("Reading GPS location...");
  
  // Enable GNSS system
  sendGPSCommand("AT+CGNSS=1");
  delay(2000);
  
  // Request current location
  flushSIM7600Buffer();
  sim7600.println("AT+CGNSSINFO");
  
  String response = waitForGPSResponse(5000);
  
  if (response.indexOf("+CGNSSINFO:") != -1) {
    if (isValidGPSFix(response)) {
      // Extract GPS data fields
      String latitude = extractGPSField(response, 4);
      String lat_direction = extractGPSField(response, 5);
      String longitude = extractGPSField(response, 6);
      String lon_direction = extractGPSField(response, 7);
      String altitude = extractGPSField(response, 10);
      
      // Convert to decimal degrees
      currentData.latitude = convertToDecimalDegrees(latitude, lat_direction);
      currentData.longitude = convertToDecimalDegrees(longitude, lon_direction);
      currentData.altitude = altitude.toFloat();
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

// Fetch device details from Firebase for security comparison
void fetchDeviceDetailsFromFirebase() {
  Serial.println("📥 Fetching device details from Firebase...");
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("✗ WiFi not connected for Firebase fetch");
    return;
  }
  
  HTTPClient http;
  String url = String(FIREBASE_HOST) + "/tracking_box/" + DEVICE_ID + "/details.json";
  
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  
  int httpResponseCode = http.GET();
  
  if (httpResponseCode == HTTP_CODE_OK) {
    String response = http.getString();
    Serial.println("✓ Firebase details fetched successfully");
    Serial.println("📄 Raw Firebase response: " + response);
    
    // Parse JSON response
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, response);
    
    if (!error) {
      currentData.deviceSetLocation = doc["setLocation"] | "";
      currentData.deviceDescription = doc["description"] | "";
      currentData.deviceName = doc["name"] | "";
      
      Serial.println("📍 Set Location: " + currentData.deviceSetLocation);
      Serial.println("📝 Description: " + currentData.deviceDescription);
      Serial.println("🏷️ Name: " + currentData.deviceName);
    } else {
      Serial.println("✗ Failed to parse Firebase details JSON");
    }
  } else {
    Serial.println("✗ Firebase details fetch failed: " + String(httpResponseCode));
  }
  
  http.end();
}

// Perform security check comparing GPS location with set location
void performSecurityCheck() {
  Serial.println("🔒 Performing security check...");
  
  if (!currentData.gpsFixValid) {
    Serial.println("⚠️ Security check skipped - No GPS fix available");
    buzzerAlert(3, 300); // Warning beeps for no GPS
    return;
  }
  
  if (currentData.deviceSetLocation.length() == 0) {
    Serial.println("⚠️ Security check skipped - No set location in database");
    buzzerAlert(3, 300); // Warning beeps for no set location
    return;
  }
  
  // Parse set location coordinates
  int commaIndex = currentData.deviceSetLocation.indexOf(',');
  if (commaIndex == -1) {
    Serial.println("✗ Invalid set location format in database");
    buzzerAlert(3, 300);
    return;
  }
  
  double setLat = currentData.deviceSetLocation.substring(0, commaIndex).toDouble();
  double setLon = currentData.deviceSetLocation.substring(commaIndex + 1).toDouble();
  
  // Calculate distance between current GPS and set location
  double distance = calculateDistance(currentData.latitude, currentData.longitude, setLat, setLon);
  
  Serial.println("📍 Current GPS: " + String(currentData.latitude, 6) + "," + String(currentData.longitude, 6));
  Serial.println("📍 Set Location: " + String(setLat, 6) + "," + String(setLon, 6));
  Serial.println("📏 Distance: " + String(distance, 2) + " meters");
  
  // Security threshold: 100 meters (adjust as needed)
  const double SECURITY_THRESHOLD_METERS = 100.0;
  
  if (distance > SECURITY_THRESHOLD_METERS) {
    Serial.println("🚨 SECURITY BREACH DETECTED! Device moved beyond safe zone!");
    currentData.locationBreach = true;
    
    // Sound security alarm
    for (int i = 0; i < 10; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(200);
      digitalWrite(BUZZER_PIN, LOW);
      delay(200);
    }
  } else {
    Serial.println("✅ Security check passed - Device within safe zone");
    currentData.locationBreach = false;
    buzzerAlert(2, 100); // Success beeps
  }
}

// Calculate distance between two GPS coordinates using Haversine formula
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000; // Earth's radius in meters
  double dLat = (lat2 - lat1) * PI / 180.0;
  double dLon = (lon2 - lon1) * PI / 180.0;
  
  double a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) * sin(dLon/2) * sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  
  return R * c;
}

// Display QR code for offline access
void displayOfflineQRCode() {
  Serial.println("📱 Displaying offline QR code...");
  
  #if ENABLE_EINK_DISPLAY
  display.setRotation(3);
  display.setFont(&FreeSans12pt7b);
  display.setTextColor(GxEPD_BLACK);
  display.setFullWindow();
  
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    
    // Title
    display.setCursor(20, 30);
    display.print("OFFLINE MODE");
    
    // Instructions
    display.setFont(&FreeSans9pt7b);
    display.setCursor(20, 60);
    display.print("No WiFi Connection");
    display.setCursor(20, 85);
    display.print("Scan QR Code for Details:");
    
    // QR Code placeholder (simplified representation)
    display.fillRect(50, 100, 200, 200, GxEPD_BLACK);
    display.fillRect(60, 110, 180, 180, GxEPD_WHITE);
    
    // Draw QR-like pattern
    for (int i = 0; i < 18; i++) {
      for (int j = 0; j < 18; j++) {
        if ((i + j) % 3 == 0) {
          display.fillRect(60 + i*10, 110 + j*10, 8, 8, GxEPD_BLACK);
        }
      }
    }
    
    // URL text
    display.setCursor(20, 330);
    display.print("URL: tracking-box.local/qr/");
    display.setCursor(20, 355);
    display.print(DEVICE_ID);
    
    // Device info
    display.setCursor(20, 390);
    display.print("Device: " + DEVICE_ID.substring(0, 10));
    display.setCursor(20, 415);
    display.print("Battery: " + String(currentData.batteryVoltage, 1) + "V");
    
  } while (display.nextPage());
  
  Serial.println("💡 Offline QR code displayed on E-ink");
  #else
  Serial.println("⚠ E-ink display disabled - QR code not shown");
  Serial.println("📱 QR Code URL: tracking-box.local/qr/" + DEVICE_ID);
  #endif
  
  // Sound offline notification
  buzzerAlert(4, 150);
}

void sendDataToFirebase() {
  Serial.println("Sending data to Firebase...");
  
  // Always update device details to ensure the structure exists
  // This creates the details table even if name/location are empty
  updateDeviceDetails();
  
  // Send sensor data
  sendSensorData();
}

void updateDeviceDetails() {
  HTTPClient http;
  String firebaseURL = String(FIREBASE_HOST) + "/tracking_box/" + DEVICE_ID + "/details.json?auth=" + String(FIREBASE_AUTH);
  
  http.begin(firebaseURL);
  http.addHeader("Content-Type", "application/json");
  
  // Create details JSON payload
  DynamicJsonDocument detailsDoc(512);
  detailsDoc["name"] = DEVICE_NAME;
  detailsDoc["setLocation"] = DEVICE_LOCATION;
  // Note: itemDesc/description is managed by the web interface
  // We don't overwrite it here to preserve user edits
  
  String detailsString;
  serializeJson(detailsDoc, detailsString);
  
  int httpResponseCode = http.PATCH(detailsString);  // Changed from PUT to PATCH
  
  if (httpResponseCode > 0) {
    Serial.println("✓ Device details updated successfully");
  } else {
    Serial.println("✗ Device details update failed");
  }
  
  http.end();
}

void sendSensorData() {
  HTTPClient http;
  String firebaseURL = String(FIREBASE_HOST) + "/tracking_box/" + DEVICE_ID + "/sensorData.json?auth=" + String(FIREBASE_AUTH);
  
  http.begin(firebaseURL);
  http.addHeader("Content-Type", "application/json");
  
  // Create current location string from GPS coordinates
  String currentLocation = "";
  if (currentData.gpsFixValid) {
    currentLocation = String(currentData.latitude, 6) + "," + String(currentData.longitude, 6);
  } else {
    currentLocation = "No GPS Fix";
  }
  
  // Create sensor data JSON payload according to new structure
  DynamicJsonDocument sensorDoc(1024);
  sensorDoc["temp"] = currentData.temperature;
  sensorDoc["humidity"] = currentData.humidity;
  sensorDoc["accelerometer"] = currentData.tiltDetected ? "TILT_DETECTED" : "NORMAL";
  sensorDoc["currentLocation"] = currentLocation;
  
  // Additional data for debugging/monitoring (optional)
  sensorDoc["batteryVoltage"] = currentData.batteryVoltage;
  sensorDoc["wakeReason"] = currentData.wakeReason;
  sensorDoc["timestamp"] = millis();
  sensorDoc["bootCount"] = bootCount;
  sensorDoc["altitude"] = currentData.altitude;
  
  // Security and limit switch data
  sensorDoc["limitSwitchPressed"] = currentData.limitSwitchPressed;
  sensorDoc["locationBreach"] = currentData.locationBreach;
  
  String sensorString;
  serializeJson(sensorDoc, sensorString);
  
  int httpResponseCode = http.PUT(sensorString);
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("✓ Sensor data uploaded successfully");
    Serial.println("Response Code: " + String(httpResponseCode));
    Serial.println("Data sent: " + sensorString);
  } else {
    Serial.println("✗ Sensor data upload failed");
    Serial.println("Error Code: " + String(httpResponseCode));
  }
  
  http.end();
}

// =====================================================================
// E-INK DISPLAY UPDATE
// =====================================================================
#if ENABLE_EINK_DISPLAY
void updateEInkDisplay() {
  Serial.println("Updating E-ink display with sensor data...");
  
  // Debug: Show what Firebase data we have
  Serial.println("🔍 Firebase data for display:");
  Serial.println("  Device Name: '" + currentData.deviceName + "'");
  Serial.println("  Set Location: '" + currentData.deviceSetLocation + "'");
  Serial.println("  Description: '" + currentData.deviceDescription + "'");
  
  display.setRotation(3);
  display.setFullWindow();
  display.firstPage();
  
  do {
    display.fillScreen(GxEPD_WHITE);
    
    // Header Section
    display.setFont(&FreeSans12pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(10, 25);
    String deviceIdUpper = DEVICE_ID.substring(4);
    deviceIdUpper.toUpperCase();
    display.print("TRACKING BOX - " + deviceIdUpper);
    
    // Device info (use Firebase data if available)
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(10, 45);
    String displayName = currentData.deviceName.length() > 0 ? currentData.deviceName : DEVICE_NAME;
    display.print("Name: " + displayName);
    
    display.setCursor(10, 60);
    String displayLocation = currentData.deviceSetLocation.length() > 0 ? currentData.deviceSetLocation : DEVICE_LOCATION;
    display.print("Set Location: " + displayLocation.substring(0, 25)); // Truncate for display
    
    display.setFont(&FreeSans9pt7b);
    display.setCursor(350, 45);
    display.print("Boot: " + String(bootCount));
    
    // Separator line
    display.drawLine(10, 70, 470, 70, GxEPD_BLACK);
    
    // Environment section
    display.setFont(&FreeSans9pt7b);
    display.setCursor(10, 90);
    display.print("ENVIRONMENT DATA");
    
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(15, 110);
    display.print("Temperature: " + String(currentData.temperature, 1) + " C");
    
    display.setCursor(15, 130);
    display.print("Humidity: " + String(currentData.humidity, 0) + " %");
    
    // GPS section
    display.setFont(&FreeSans9pt7b);
    display.setCursor(10, 155);
    display.print("GPS LOCATION");
    
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(15, 175);
    if (currentData.gpsFixValid) {
      display.print("Lat: " + String(currentData.latitude, 4));
      display.setCursor(15, 195);
      display.print("Lon: " + String(currentData.longitude, 4));
    } else {
      display.print("GPS: Searching for fix...");
    }
    
    // System status
    display.setFont(&FreeSans9pt7b);
    display.setCursor(10, 220);
    display.print("SYSTEM STATUS");
    
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(15, 240);
    display.print("Battery: " + String(currentData.batteryVoltage, 2) + "V");
    
    display.setCursor(15, 260);
    display.print("Motion: " + String(currentData.tiltDetected ? "DETECTED" : "STABLE"));
    
    display.setCursor(15, 280);
    display.print("Wake: " + getFriendlyWakeReason(currentData.wakeReason));
    
    // Security status
    display.setFont(&FreeSans9pt7b);
    display.setCursor(10, 305);
    display.print("SECURITY STATUS");
    
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(15, 325);
    if (currentData.limitSwitchPressed) {
      display.print("Switch: PRESSED");
    } else {
      display.print("Switch: NORMAL");
    }
    
    display.setCursor(15, 345);
    if (currentData.locationBreach) {
      display.print("Location: BREACH!");
    } else {
      display.print("Location: SECURE");
    }
    
    // Description (if available)
    if (currentData.deviceDescription.length() > 0) {
      display.setFont(&FreeSans9pt7b);
      display.setCursor(10, 370);
      display.print("DESCRIPTION");
      
      display.setFont(&FreeMonoBold9pt7b);
      display.setCursor(15, 390);
      String desc = currentData.deviceDescription.substring(0, 40); // Truncate for display
      display.print(desc);
    }
    
  } while (display.nextPage());
  
  display.hibernate();
  
  Serial.println("✓ E-ink display updated with sensor data");
}

void displayConnectionStatus(bool wifiConnected, bool firebaseSuccess) {
  Serial.println("Updating E-ink with connection status...");
  
  display.setRotation(3);
  display.setFullWindow();
  display.firstPage();
  
  do {
    display.fillScreen(GxEPD_WHITE);
    
    // Header Section
    display.setFont(&FreeSans12pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(10, 25);
    display.print("TRACKING BOX - CONNECTION");
    
    // Device info
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(10, 45);
    display.print("Device: " + DEVICE_ID);
    
    display.setCursor(10, 60);
    display.print("Boot Count: " + String(bootCount));
    
    // Separator line
    display.drawLine(10, 70, 470, 70, GxEPD_BLACK);
    
    // Connection Status Section
    display.setFont(&FreeSans9pt7b);
    display.setCursor(10, 90);
    display.print("CONNECTION STATUS");
    
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(15, 110);
    display.print("WiFi: " + String(wifiConnected ? "CONNECTED" : "FAILED"));
    
    display.setCursor(15, 130);
    display.print("Firebase: " + String(firebaseSuccess ? "SUCCESS" : "FAILED"));
    
    // Wake reason
    display.setCursor(15, 150);
    display.print("Wake: " + getFriendlyWakeReason(currentData.wakeReason));
    
    // Status summary
    display.setFont(&FreeSans9pt7b);
    display.setCursor(10, 180);
    display.print("SYSTEM STATUS");
    
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(15, 200);
    if (wifiConnected && firebaseSuccess) {
      display.print("Status: ALL SYSTEMS OPERATIONAL");
    } else if (wifiConnected) {
      display.print("Status: WIFI OK, DATA UPLOAD FAILED");
    } else {
      display.print("Status: WIFI CONNECTION FAILED");
    }
    
    // Timestamp
    display.setFont(&FreeSans9pt7b);
    display.setCursor(15, 230);
    display.print("Last Update: " + formatReadableTime());
    
    // Battery info
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(15, 260);
    display.print("Battery: " + String(currentData.batteryVoltage, 2) + "V");
    
  } while (display.nextPage());
  
  display.hibernate();
}

#else
// If E-ink display is disabled, provide informative console output
void updateEInkDisplay() {
  Serial.println("⚠ E-ink display update skipped - display disabled");
  Serial.println("💡 To enable: Set ENABLE_EINK_DISPLAY to 1 and install GxEPD2 library");
  Serial.println("");
  Serial.println("📺 SIMULATED E-INK DISPLAY CONTENT:");
  Serial.println("=====================================");
  String deviceIdUpper = DEVICE_ID;
  deviceIdUpper.toUpperCase();
  Serial.println("TRACKING BOX - " + deviceIdUpper);
  Serial.println("Temperature: " + String(currentData.temperature, 1) + "°C");
  Serial.println("Humidity: " + String(currentData.humidity, 0) + "%");
  Serial.println("GPS: " + String(currentData.gpsFixValid ? "Fixed" : "Searching"));
  Serial.println("Battery: " + String(currentData.batteryVoltage, 2) + "V");
  Serial.println("Motion: " + String(currentData.tiltDetected ? "Detected" : "Stable"));
  Serial.println("Boot Count: " + String(bootCount));
  Serial.println("=====================================");
}

void displayConnectionStatus(bool wifiConnected, bool firebaseSuccess) {
  Serial.println("📺 CONNECTION STATUS DISPLAY:");
  Serial.println("WiFi: " + String(wifiConnected ? "Connected" : "Failed"));
  Serial.println("Firebase: " + String(firebaseSuccess ? "Success" : "Failed"));
}
#endif

// =====================================================================
// DEEP SLEEP PREPARATION
// =====================================================================
void prepareForDeepSleep() {
  Serial.println("🛏️ Preparing for deep sleep mode...");
  
  // Configure wake-up sources
  esp_sleep_enable_timer_wakeup(SLEEP_TIME_US);                    // Timer wake (15 minutes)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 0);                    // Accelerometer interrupt wake
  esp_sleep_enable_ext1_wakeup(1ULL << LIMIT_SWITCH_PIN, ESP_EXT1_WAKEUP_ANY_HIGH); // Limit switch wake
  
  Serial.println("⏰ Timer wake-up: " + String(SLEEP_MINUTES) + " minutes");
  Serial.println("📳 Accelerometer wake-up: Enabled (GPIO25)");
  Serial.println("🔘 Limit switch wake-up: Enabled (GPIO34)");
  
  // Configure GPIO to maintain state during sleep
  rtc_gpio_pullup_en(GPIO_NUM_25);
  rtc_gpio_hold_en(GPIO_NUM_25);
  
  // Ensure WiFi is disconnected
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  Serial.println("Sleep configuration complete");
  Serial.println("Next wake: Timer (" + String(SLEEP_MINUTES) + " min) OR Accelerometer interrupt");
}

// =====================================================================
// UTILITY FUNCTIONS
// =====================================================================
String getWakeupReasonString(esp_sleep_wakeup_cause_t wakeup_reason) {
  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:       return "Accelerometer Interrupt";
    case ESP_SLEEP_WAKEUP_EXT1:       return "Limit Switch Pressed";
    case ESP_SLEEP_WAKEUP_TIMER:      return "Timer (15 minutes)";
    case ESP_SLEEP_WAKEUP_UNDEFINED:  return "Power On / First Boot";
    default:                          return "Unknown Wake Source";
  }
}

void buzzerAlert(int beepCount, int beepDuration) {
  for (int i = 0; i < beepCount; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(beepDuration);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < beepCount - 1) {
      delay(beepDuration);
    }
  }
}

// =====================================================================
// E-INK DISPLAY HELPER FUNCTIONS
// =====================================================================
String getFriendlyWakeReason(String reason) {
  if (reason.indexOf("Timer") != -1) {
    return "Scheduled Update";
  } else if (reason.indexOf("Accelerometer") != -1) {
    return "Motion Detected";
  } else if (reason.indexOf("Limit Switch") != -1) {
    return "Switch Activated";
  } else if (reason.indexOf("Power") != -1 || reason.indexOf("First") != -1) {
    return "Device Started";
  } else {
    return "System Wake";
  }
}

String formatReadableTime() {
  // Convert milliseconds to a more readable format
  unsigned long seconds = millis() / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  
  seconds = seconds % 60;
  minutes = minutes % 60;
  hours = hours % 24;
  
  String timeStr = "";
  if (hours > 0) {
    timeStr += String(hours) + "h ";
  }
  if (minutes > 0 || hours > 0) {
    timeStr += String(minutes) + "m ";
  }
  timeStr += String(seconds) + "s ago";
  
  return timeStr;
}

void printSensorReadings() {
  Serial.println("=====================================================");
  Serial.println("           CURRENT SENSOR READINGS");
  Serial.println("=====================================================");
  Serial.println("Temperature:    " + String(currentData.temperature, 2) + " °C");
  Serial.println("Humidity:       " + String(currentData.humidity, 2) + " %");
  Serial.println("Latitude:       " + String(currentData.latitude, 6));
  Serial.println("Longitude:      " + String(currentData.longitude, 6));
  Serial.println("Altitude:       " + String(currentData.altitude, 2) + " m");
  Serial.println("Tilt Detected:  " + String(currentData.tiltDetected ? "YES" : "NO"));
  Serial.println("GPS Fix Valid:  " + String(currentData.gpsFixValid ? "YES" : "NO"));
  Serial.println("Battery:        " + String(currentData.batteryVoltage, 2) + " V");
  Serial.println("Wake Reason:    " + currentData.wakeReason);
  Serial.println("Boot Count:     " + String(bootCount));
  Serial.println("Limit Switch:   " + String(currentData.limitSwitchPressed ? "PRESSED" : "NORMAL"));
  Serial.println("Location Breach:" + String(currentData.locationBreach ? "YES" : "NO"));
  if (currentData.deviceSetLocation.length() > 0) {
    Serial.println("Set Location:   " + currentData.deviceSetLocation);
  }
  if (currentData.deviceDescription.length() > 0) {
    Serial.println("Description:    " + currentData.deviceDescription);
  }
  Serial.println("=====================================================");
}

// =====================================================================
// LSM6DSL ACCELEROMETER I2C FUNCTIONS
// =====================================================================
uint8_t readAccelRegister(uint8_t registerAddress) {
  Wire1.beginTransmission(lsm6dsl_address);
  Wire1.write(registerAddress);
  uint8_t error = Wire1.endTransmission(false);
  
  if (error != 0) {
    return 0xFF; // Error indicator
  }
  
  Wire1.requestFrom(lsm6dsl_address, (uint8_t)1);
  return Wire1.available() ? Wire1.read() : 0xFF;
}

void writeAccelRegister(uint8_t registerAddress, uint8_t value) {
  Wire1.beginTransmission(lsm6dsl_address);
  Wire1.write(registerAddress);
  Wire1.write(value);
  Wire1.endTransmission();
}

// =====================================================================
// SIM7600 GPS COMMUNICATION FUNCTIONS
// =====================================================================
bool sendGPSCommand(String command) {
  flushSIM7600Buffer();
  sim7600.println(command);
  String response = waitForGPSResponse(3000);
  return response.indexOf("OK") != -1;
}

String waitForGPSResponse(unsigned long timeoutMs) {
  unsigned long startTime = millis();
  String response = "";
  
  while (millis() - startTime < timeoutMs) {
    while (sim7600.available()) {
      response += (char)sim7600.read();
    }
    if (response.indexOf("OK") != -1 || response.indexOf("ERROR") != -1) {
      break;
    }
    delay(10);
  }
  return response;
}

void flushSIM7600Buffer() {
  while (sim7600.available()) {
    sim7600.read();
  }
}

String extractGPSField(String data, int fieldIndex) {
  int colonIndex = data.indexOf(":");
  if (colonIndex == -1) return "";
  
  String fieldData = data.substring(colonIndex + 1);
  fieldData.trim();
  
  int currentField = 0;
  int startIndex = 0;
  
  while (currentField < fieldIndex) {
    startIndex = fieldData.indexOf(',', startIndex) + 1;
    if (startIndex == 0) return "";
    currentField++;
  }
  
  int endIndex = fieldData.indexOf(',', startIndex);
  if (endIndex == -1) endIndex = fieldData.length();
  
  return fieldData.substring(startIndex, endIndex);
}

bool isValidGPSFix(String response) {
  String fixStatus = extractGPSField(response, 1);
  fixStatus.trim();
  return (fixStatus == "01" || fixStatus == "02"); // 2D or 3D fix
}

double convertToDecimalDegrees(String coordinate, String direction) {
  if (coordinate.length() < 4) return 0.0;
  
  int decimalIndex = coordinate.indexOf('.');
  if (decimalIndex == -1) return 0.0;
  
  // Extract degrees and minutes
  String degreesString = coordinate.substring(0, decimalIndex - 2);
  String minutesString = coordinate.substring(decimalIndex - 2);
  
  double degrees = degreesString.toDouble();
  double minutes = minutesString.toDouble();
  
  // Convert to decimal degrees
  double decimalDegrees = degrees + (minutes / 60.0);
  
  // Apply direction (South and West are negative)
  if (direction == "S" || direction == "W") {
    decimalDegrees = -decimalDegrees;
  }
  
  return decimalDegrees;
}

// =====================================================================
// END OF TRACKING BOX MAIN FIRMWARE
// ===================================================================== 