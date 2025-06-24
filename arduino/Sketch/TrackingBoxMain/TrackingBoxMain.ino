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
#include "DEV_Config.h"
#include "EPD.h"
#include "GUI_Paint.h"

#include "fonts.h"

#define ENABLE_SHT30_SENSOR 1

// E-ink Display Libraries (only include if enabled)


// SHT30 Library (only include if enabled)
#if ENABLE_SHT30_SENSOR
#include "Adafruit_SHT31.h"
#endif



// Frame buffer pointer for GUI_Paint
static UBYTE *gImageBuffer = nullptr;

// =====================================================================
// PIN DEFINITIONS
// =====================================================================
// Power & Analog
#define BATTERY_ADC_PIN     36    // ADC1_CH0 - Safe with WiFi

// SHT30 Temperature/Humidity - I2C Bus 0 (Default)
#define SHT30_SDA_PIN       21    // I2C0 SDA (default)
#define SHT30_SCL_PIN       22    // I2C0 SCL (default)

// LSM6DSL Accelerometer - shares I2C bus with SHT30
#define LSM6DSL_SDA_PIN     21    // I2C0 SDA (shared)
#define LSM6DSL_SCL_PIN     22    // I2C0 SCL (shared)
#define LSM6DSL_INT1_PIN    34    // Interrupt pin (INT1)

// SIM7600 GPS Module - UART2 (matches waveshare tracker board schematic)
#define SIM7600_TX_PIN      17    // TX -> Module RX
#define SIM7600_RX_PIN      16    // RX <- Module TX

// Digital I/O
#define LIMIT_SWITCH_PIN    33    // Limit switch input (with internal pull-up)
#define BUZZER_PIN          32    // Buzzer output (PWM capable)

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
  DEV_Module_Init();
  EPD_3IN7_4Gray_Init();
  
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
    // After communication, update main dashboard on E-ink
    updateEInkDisplay();
    
    // Sound success notification
    buzzerAlert(2, 100);
    
    Serial.println("✓ Operation cycle completed successfully");
  } else {
    Serial.println("✗ Hardware initialization failed!");
    buzzerAlert(5, 200); // Error alert
  }
  
  prepareForDeepSleep();
  Serial.println("Entering deep sleep mode...");
  Serial.flush();
  esp_deep_sleep_start();
}

void loop() {
  // We should never reach the main loop because the device enters
  // deep-sleep at the end of setup(). If we do, immediately deep-sleep.
  prepareForDeepSleep();
  esp_deep_sleep_start();
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
  
  // Initialize LSM6DSL Accelerometer (shares same I2C bus as SHT30)
  // No need for a second I2C controller; use the default Wire instance
  // (The bus was already initialised above).
  // Re-initialising is harmless but guarantees clock speed remains default.
  Wire.begin(LSM6DSL_SDA_PIN, LSM6DSL_SCL_PIN);
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
  
  // Initialize Waveshare E-ink Display (3.7" ED037TC1)
  Serial.println("Initializing Waveshare 3.7\" e-ink...");
  DEV_Module_Init();
  EPD_3IN7_4Gray_Init();

  // Allocate full-frame buffer once
  if (gImageBuffer == nullptr) {
    UWORD imgSize = ((EPD_3IN7_WIDTH % 4 == 0) ? (EPD_3IN7_WIDTH / 4) : (EPD_3IN7_WIDTH / 4 + 1)) * EPD_3IN7_HEIGHT;
    gImageBuffer = (UBYTE *)malloc(imgSize);
    if (!gImageBuffer) {
      Serial.println("✗ E-ink framebuffer malloc failed");
      return false;
    }
    Paint_NewImage(gImageBuffer, EPD_3IN7_WIDTH, EPD_3IN7_HEIGHT, 270, WHITE);
    Paint_SetScale(4); // 4-gray mode
  }

  // Clear screen once at boot
  EPD_3IN7_4Gray_Clear();
  Serial.println("✓ E-ink display initialized successfully");
  
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

void displayOfflineQRCode() {
  Serial.println("📱 Displaying offline QR code...");

  _preparePaint();
  Paint_DrawString_EN(10, 10, (char*)"OFFLINE MODE", &Font16, WHITE, BLACK);
  Paint_DrawString_EN(10, 40, (char*)"No WiFi Connection", &Font12, WHITE, BLACK);
  Paint_DrawString_EN(10, 56, (char*)"Scan QR -> details", &Font12, WHITE, BLACK);

  // Draw simple QR placeholder box
  Paint_DrawRectangle(150, 80, 350, 280, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
  for(int i=0;i<10;i++){
    for(int j=0;j<10;j++){
      if((i+j)%3==0){
        Paint_DrawRectangle(160+i*18, 90+j*18, 160+i*18+12, 90+j*18+12, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
      }
    }
  }

  char buf[64];
  sprintf(buf, "tracking-box.local/qr/%s", DEVICE_ID.c_str());
  Paint_DrawString_EN(10, 300, buf, &Font12, WHITE, BLACK);

  _showAndSleep();

  Serial.println("💡 Offline QR placeholder drawn");

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



// Waveshare-based rendering functions -------------------------------------------------
void _preparePaint() {
  if (!gImageBuffer) return;
  Paint_SelectImage(gImageBuffer);
  Paint_Clear(WHITE);
}

void _showAndSleep() {
  if (!gImageBuffer) return;
  EPD_3IN7_4Gray_Display(gImageBuffer);
  EPD_3IN7_Sleep();
}

void updateEInkDisplay() {
  _preparePaint();
  // Header
  Paint_DrawString_EN(10, 4, (char*)"TRACKING BOX", &Font16, WHITE, BLACK);

  char buf[64];
  // Temperature & Humidity
  sprintf(buf, "Temp %.1fC", currentData.temperature);
  Paint_DrawString_EN(10, 30, buf, &Font12, WHITE, BLACK);
  sprintf(buf, "Hum  %.0f%%", currentData.humidity);
  Paint_DrawString_EN(10, 46, buf, &Font12, WHITE, BLACK);

  // Battery
  sprintf(buf, "Batt %.2fV", currentData.batteryVoltage);
  Paint_DrawString_EN(10, 62, buf, &Font12, WHITE, BLACK);

  // GPS (short)
  if (currentData.gpsFixValid) {
    sprintf(buf, "Lat %.4f", currentData.latitude);
    Paint_DrawString_EN(200, 30, buf, &Font12, WHITE, BLACK);
    sprintf(buf, "Lon %.4f", currentData.longitude);
    Paint_DrawString_EN(200, 46, buf, &Font12, WHITE, BLACK);
  } else {
    Paint_DrawString_EN(200, 38, (char*)"GPS searching", &Font12, WHITE, BLACK);
  }

  // Motion / security
  Paint_DrawString_EN(10, 90, (char*)(currentData.tiltDetected ? "MOTION DETECTED" : "Stable"), &Font12, WHITE, BLACK);
  if (currentData.locationBreach) {
    Paint_DrawString_EN(10, 106, (char*)"SECURITY BREACH", &Font12, WHITE, BLACK);
  }

  _showAndSleep();
}

void displayConnectionStatus(bool wifiConnected, bool firebaseSuccess) {
  _preparePaint();
  Paint_DrawString_EN(10, 20, (char*)"CONNECTION STATUS", &Font16, WHITE, BLACK);
  Paint_DrawString_EN(10, 50, (char*)(wifiConnected ? "WiFi OK" : "WiFi FAIL"), &Font12, WHITE, BLACK);
  Paint_DrawString_EN(10, 66, (char*)(firebaseSuccess ? "Firebase OK" : "Firebase FAIL"), &Font12, WHITE, BLACK);
  _showAndSleep();
}

// =====================================================================
// DEEP SLEEP PREPARATION
// =====================================================================
void prepareForDeepSleep() {
  Serial.println("🛏️ Preparing for deep sleep mode...");
  
  // Configure wake-up sources
  esp_sleep_enable_timer_wakeup(SLEEP_TIME_US);                    // Timer wake (15 minutes)
  esp_sleep_enable_ext0_wakeup((gpio_num_t)LSM6DSL_INT1_PIN, 0);                    // Accelerometer interrupt wake
  esp_sleep_enable_ext1_wakeup(1ULL << LIMIT_SWITCH_PIN, ESP_EXT1_WAKEUP_ALL_LOW);  // Limit switch wake (active-low)
  
  Serial.println("⏰ Timer wake-up: " + String(SLEEP_MINUTES) + " minutes");
  Serial.println("📳 Accelerometer wake-up: Enabled (GPIO" + String(LSM6DSL_INT1_PIN) + ")");
  Serial.println("🔘 Limit switch wake-up: Enabled (GPIO" + String(LIMIT_SWITCH_PIN) + ")");
  
  // Configure GPIO to maintain state during sleep
  rtc_gpio_pullup_en((gpio_num_t)LSM6DSL_INT1_PIN);
  rtc_gpio_hold_en((gpio_num_t)LSM6DSL_INT1_PIN);
  
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
  Wire.beginTransmission(lsm6dsl_address);
  Wire.write(registerAddress);
  uint8_t error = Wire.endTransmission(false);
  
  if (error != 0) {
    return 0xFF; // Error indicator
  }
  
  Wire.requestFrom(lsm6dsl_address, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void writeAccelRegister(uint8_t registerAddress, uint8_t value) {
  Wire.beginTransmission(lsm6dsl_address);
  Wire.write(registerAddress);
  Wire.write(value);
  Wire.endTransmission();
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