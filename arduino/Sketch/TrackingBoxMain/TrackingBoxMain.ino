/*
 * =====================================================================
 * TRACKING BOX DEVICE - MAIN FIRMWARE
 * =====================================================================
 * 
 * This sketch combines all individual sensor tests into one complete
 * tracking device that satisfies all requirements from FEATURES.md
 * 
 * Components:
 * - SHT30 Temperature & Humidity Sensor
 * - SIM7600 GNSS (GPS Location)
 * - ESP32 Microcontroller
 * - LSM6DSL 6-axis Accelerometer/Gyroscope
 * - Li-Po Battery
 * - 3.7" Waveshare E-ink Display
 * - Buzzer
 * 
 * Core Functionality:
 * 1. Wake every 15 minutes OR on accelerometer interrupt
 * 2. Read all sensors when awake
 * 3. Send data to Firebase database
 * 4. Update E-ink display with current information
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

// E-ink Display Libraries
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

// SHT30 Library
#include "Adafruit_SHT31.h"

// =====================================================================
// PIN DEFINITIONS
// =====================================================================
#define SHT30_SDA           18    // SHT30 I2C Data
#define SHT30_SCL           19    // SHT30 I2C Clock
#define LSM6DSL_SDA         21    // Accelerometer I2C Data
#define LSM6DSL_SCL         22    // Accelerometer I2C Clock
#define SIM7600_RX          16    // GPS UART Receive
#define SIM7600_TX          17    // GPS UART Transmit
#define EINK_CS             5     // E-ink SPI Chip Select
#define EINK_DC             0     // E-ink Data/Command
#define EINK_RST            2     // E-ink Reset
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
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Firebase Configuration (from firebase.ts)
const char* FIREBASE_HOST = "https://tracking-box-e17a1-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* FIREBASE_AUTH = "AIzaSyBQje281bPAt7MiJdK94ru1irAU8i3luzY";

// Device Information
const String DEVICE_ID = "TRACKER_001";
const String OWNER_NAME = "Device Owner";
const String OWNER_ADDRESS = "Owner Address";

// =====================================================================
// GLOBAL OBJECTS & VARIABLES
// =====================================================================
Adafruit_SHT31 sht30 = Adafruit_SHT31();
HardwareSerial sim7600(1);
GxEPD2_BW<GxEPD2_290_T5D, GxEPD2_290_T5D::HEIGHT> display(GxEPD2_290_T5D(EINK_CS, EINK_DC, EINK_RST, EINK_BUSY));

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
    
    // Connect to WiFi and send data to Firebase
    if (connectToWiFi()) {
      sendDataToFirebase();
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
    }
    
    // Update E-ink display with current data
    updateEInkDisplay();
    
    // Sound success notification
    buzzerAlert(2, 100);
    
    Serial.println("✓ Operation cycle completed successfully");
  } else {
    Serial.println("✗ Hardware initialization failed!");
    buzzerAlert(5, 200); // Error alert
  }
  
  // Prepare for and enter deep sleep
  prepareForDeepSleep();
  
  Serial.println("Entering deep sleep mode...");
  Serial.flush();
  esp_deep_sleep_start();
}

void loop() {
  // This function should never be reached due to deep sleep
  // If reached, restart the device
  Serial.println("ERROR: Loop reached - restarting device");
  ESP.restart();
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
  Wire.begin(SHT30_SDA, SHT30_SCL);
  if (!sht30.begin(0x44)) {
    Serial.println("✗ SHT30 sensor initialization failed");
    return false;
  }
  Serial.println("✓ SHT30 sensor initialized");
  
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
  display.init(115200, true, 2, false);
  Serial.println("✓ E-ink display initialized");
  
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
  
  HTTPClient http;
  String firebaseURL = String(FIREBASE_HOST) + "/trackers/" + DEVICE_ID + "/readings.json?auth=" + String(FIREBASE_AUTH);
  
  http.begin(firebaseURL);
  http.addHeader("Content-Type", "application/json");
  
  // Create JSON payload with all sensor data
  DynamicJsonDocument jsonDoc(1024);
  jsonDoc["deviceId"] = DEVICE_ID;
  jsonDoc["ownerName"] = OWNER_NAME;
  jsonDoc["ownerAddress"] = OWNER_ADDRESS;
  jsonDoc["temperature"] = currentData.temperature;
  jsonDoc["humidity"] = currentData.humidity;
  jsonDoc["latitude"] = currentData.latitude;
  jsonDoc["longitude"] = currentData.longitude;
  jsonDoc["altitude"] = currentData.altitude;
  jsonDoc["tiltDetected"] = currentData.tiltDetected;
  jsonDoc["wakeReason"] = currentData.wakeReason;
  jsonDoc["batteryVoltage"] = currentData.batteryVoltage;
  jsonDoc["timestamp"] = String(currentData.timestamp);
  jsonDoc["bootCount"] = bootCount;
  jsonDoc["gpsFixValid"] = currentData.gpsFixValid;
  
  String jsonString;
  serializeJson(jsonDoc, jsonString);
  
  int httpResponseCode = http.POST(jsonString);
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("✓ Firebase upload successful");
    Serial.println("Response Code: " + String(httpResponseCode));
    Serial.println("Response: " + response.substring(0, 100) + "...");
  } else {
    Serial.println("✗ Firebase upload failed");
    Serial.println("Error Code: " + String(httpResponseCode));
  }
  
  http.end();
}

// =====================================================================
// E-INK DISPLAY UPDATE
// =====================================================================
void updateEInkDisplay() {
  Serial.println("Updating E-ink display...");
  
  display.setRotation(3);
  display.setFullWindow();
  display.firstPage();
  
  do {
    display.fillScreen(GxEPD_WHITE);
    
    // Title Header
    display.setFont(&FreeSans12pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(10, 25);
    display.print("TRACKING BOX");
    
    // Device Information
    display.setFont(&FreeSans9pt7b);
    display.setCursor(10, 50);
    display.print("Device: " + DEVICE_ID);
    
    // Environmental Readings
    display.setCursor(10, 75);
    display.print("Temperature: " + String(currentData.temperature, 1) + "C");
    
    display.setCursor(10, 95);
    display.print("Humidity: " + String(currentData.humidity, 1) + "%");
    
    // GPS Information
    display.setCursor(10, 120);
    if (currentData.gpsFixValid) {
      display.print("GPS: " + String(currentData.latitude, 4) + "," + String(currentData.longitude, 4));
    } else {
      display.print("GPS: No Fix Available");
    }
    
    // System Status
    display.setCursor(10, 145);
    display.print("Wake: " + currentData.wakeReason);
    
    display.setCursor(10, 165);
    display.print("Tilt: " + String(currentData.tiltDetected ? "DETECTED" : "Normal"));
    
    display.setCursor(10, 185);
    display.print("Battery: " + String(currentData.batteryVoltage, 2) + "V");
    
    display.setCursor(10, 205);
    display.print("Boot Count: " + String(bootCount));
    
    // Last update timestamp
    display.setCursor(10, 225);
    display.print("Updated: " + String(currentData.timestamp / 1000) + "s");
    
  } while (display.nextPage());
  
  display.hibernate();
}

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