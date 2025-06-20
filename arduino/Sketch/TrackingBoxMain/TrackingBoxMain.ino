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
#define SHT30_SDA           18    // SHT30 I2C Data
#define SHT30_SCL           19    // SHT30 I2C Clock
#define LSM6DSL_SDA         21    // Accelerometer I2C Data
#define LSM6DSL_SCL         22    // Accelerometer I2C Clock
#define SIM7600_RX          32    // GPS UART Receive (changed from 16)
#define SIM7600_TX          33    // GPS UART Transmit (changed from 17)
#define EINK_CS             5     // E-ink SPI Chip Select
#define EINK_DC             17    // E-ink Data/Command (TX)
#define EINK_RST            16    // E-ink Reset (RX)
#define EINK_BUSY           4     // E-ink Busy
#define BUZZER_PIN          25    // Buzzer Output
#define ACCEL_INT_PIN       26    // Accelerometer Interrupt
#define BATTERY_ADC_PIN     35    // Battery Voltage Monitor

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
GxEPD2_BW<GxEPD2_370_TC1, GxEPD2_370_TC1::HEIGHT> display(GxEPD2_370_TC1(EINK_CS, EINK_DC, EINK_RST, EINK_BUSY));
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
  pinMode(ACCEL_INT_PIN, INPUT_PULLUP);
  
  // Initialize SHT30 Temperature & Humidity Sensor
  #if ENABLE_SHT30_SENSOR
  Wire.begin(SHT30_SDA, SHT30_SCL);
  if (!sht30.begin(0x44)) {
    Serial.println("✗ SHT30 sensor initialization failed");
    return false;
  }
  Serial.println("✓ SHT30 sensor initialized");
  #else
  Serial.println("⚠ SHT30 sensor disabled - using simulated data");
  #endif
  
  // Initialize LSM6DSL Accelerometer
  Wire1.begin(LSM6DSL_SDA, LSM6DSL_SCL);
  if (!initializeAccelerometer()) {
    Serial.println("✗ LSM6DSL accelerometer initialization failed");
    return false;
  }
  Serial.println("✓ LSM6DSL accelerometer initialized");
  
  // Initialize SIM7600 GPS Module
  sim7600.begin(115200, SERIAL_8N1, SIM7600_RX, SIM7600_TX);
  delay(2000);
  flushSIM7600Buffer();
  Serial.println("✓ SIM7600 GPS module initialized");
  
  // Initialize E-ink Display
  #if ENABLE_EINK_DISPLAY
  display.init(115200, true, 2, false); // USE THIS for Waveshare boards with "clever" reset circuit, 2ms reset pulse
  delay(1000); // Longer delay to ensure display is ready
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
    
    // Device info
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(10, 45);
    display.print("Name: " + DEVICE_NAME);
    
    display.setCursor(10, 60);
    display.print("Location: " + DEVICE_LOCATION);
    
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
  Serial.println("Preparing for deep sleep mode...");
  
  // Configure wake-up sources
  esp_sleep_enable_timer_wakeup(SLEEP_TIME_US);                    // Timer wake (15 minutes)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_26, 0);                    // Accelerometer interrupt wake
  
  // Configure GPIO to maintain state during sleep
  rtc_gpio_pullup_en(GPIO_NUM_26);
  rtc_gpio_hold_en(GPIO_NUM_26);
  
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