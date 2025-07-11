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
#include "esp_sleep.h"
#include "driver/rtc_io.h"
// E-ink support removed
// #include "DEV_Config.h"
// #include "EPD.h"
// #include "GUI_Paint.h"
// #include "fonts.h"
// #include "qrcode.h"   // Tiny library for generating QR bitmaps
// #include <esp_heap_caps.h>

// Display paging ---------------------------------------------------------
// #define EPD_PAGE_ROWS  40   // E-ink paging disabled

#define ENABLE_SHT30_SENSOR 1

#if ENABLE_SHT30_SENSOR
#include "Adafruit_SHT31.h"
#endif

// Frame buffer pointer for GUI_Paint
// static UBYTE *gImageBuffer = nullptr;   // removed

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

// =====================================================================
// LSM6DSL CONSTANTS
// =====================================================================
// LSM6DSL Registers
#define LSM6DSL_WHO_AM_I         0x0F
#define LSM6DSL_CTRL1_XL         0x10
#define LSM6DSL_CTRL3_C          0x12
#define LSM6DSL_OUTX_L_XL        0x28
#define LSM6DSL_CTRL8_XL         0x17 // High-pass filter configuration
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
#define FALL_THRESHOLD_MAGNITUDE    0.2  // g – if total accel magnitude < this → free-fall/shock

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
HardwareSerial sim7600(1);
uint8_t lsm6dsl_address = 0x6A;

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
};

TrackerData currentData;

// Forward declaration
void determineWakeUpReason();

// =====================================================================
// MAIN SETUP FUNCTION (Single-cycle operation)
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n================ WAKING UP ================");
  
  // A. Determine why the device woke up
  determineWakeUpReason();

  // 1. Initialize all hardware
  if (!initializeAllHardware()) {
    Serial.println("❌ Hardware initialization FAILED. Entering deep sleep.");
    prepareForDeepSleep();
    esp_deep_sleep_start();
    return;
  }
  Serial.println("✅ Hardware Initialized.");

  // 2. Gather all sensor readings
  collectSensorReading();
  Serial.println("✅ Sensor Readings Collected.");

  // 3. Connect to WiFi to send and receive data
    if (connectToWiFi()) {
    // 4. Send all sensor data to Firebase
    sendSensorDataToFirebase();
    Serial.println("✅ Sensor Data Sent to Firebase.");

    // 5. Fetch the latest complete data back from Firebase for display
    fetchDataForDisplay();
    Serial.println("✅ Latest Data Fetched from Firebase.");

    // 6. E-ink display removed

    // Disconnect WiFi to save power
      if (!currentData.buzzerIsActive) {
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      }
    } else {
    Serial.println("❌ WiFi connection failed.");
    }

  // 7. Go to deep sleep
  Serial.println("Cycle complete. Entering deep sleep.");
  Serial.println("========================================\n");

  // If buzzer is sounding due to LOCK BREACH, monitor Firebase for dismissal
  if (currentData.buzzerIsActive) {
    Serial.println("🚨 LOCK BREACH active – monitoring Firebase for dismissal …");
    unsigned long lastPoll = 0;
    while (currentData.buzzerIsActive) {
      if (millis() - lastPoll > 5000) { // poll every 5 s
        lastPoll = millis();
        if (checkDismissCommand()) {
          Serial.println("🔕 Dismiss command received – stopping buzzer and entering deep-sleep.");
          digitalWrite(BUZZER_PIN, LOW);
          currentData.buzzerIsActive = false;
          currentData.buzzerDismissed = true; // we will add field
          updateBuzzerStateInFirebase(false, true);
          break; // exit monitoring loop, continue to deep sleep
        }
      }
      delay(100);
    }
  }
  
  prepareForDeepSleep();
  esp_deep_sleep_start();
}

void loop() {
  // The loop is intentionally left empty.
  // The device performs a single cycle in setup() and then deep sleeps.
}

// =====================================================================
// E-INK DISPLAY UPDATE
// =====================================================================
// ---------------------------------------------------------------------------
#if 0   // E-INK CODE BLOCK DISABLED
void updateEInkDisplay() {
  char buf[256];

  // Pre-compute all text lines and their absolute Y positions
  struct TextLine { String txt; int y; const sFONT* font; } lines[11];
  int idx = 0;
  int yPos = 20;
  lines[idx++] = { currentData.deviceName, yPos, &Font24 }; yPos += 50;

  lines[idx++] = { String("Set Location: ") + currentData.deviceSetLocation, yPos, &Font16 }; yPos += 40;
  lines[idx++] = { String("Item Description: ") + currentData.deviceDescription, yPos, &Font16 }; yPos += 40;
  lines[idx++] = { String("Package Location: ") + currentData.currentLocation, yPos, &Font16 }; yPos += 40;
  lines[idx++] = { String("Wake Reason: ") + currentData.wakeUpReason, yPos, &Font16 }; yPos += 60;

  lines[idx++] = { String("Sensor Readings"), yPos, &Font24 }; yPos += 50;
  lines[idx++] = { String("Temperature: ") + String(currentData.temperature,1) + " C", yPos, &Font16 }; yPos += 40;
  lines[idx++] = { String("Humidity: ") + String((int)currentData.humidity) + " %", yPos, &Font16 }; yPos += 40;
  lines[idx++] = { String("Battery: ") + String(currentData.batteryVoltage,2) + " V", yPos, &Font16 }; yPos += 40;
  sprintf(buf,"Accelerometer: %.1fg, %.1fg, %.1fg",currentData.accelX,currentData.accelY,currentData.accelZ);
  lines[idx++] = { String(buf), yPos, &Font16 }; yPos += 40;

  // Loop over the screen in pages
  for(uint16_t pageY = 0; pageY < EPD_7IN3F_HEIGHT; pageY += EPD_PAGE_ROWS) {
    Paint_NewImage(gImageBuffer, EPD_7IN3F_WIDTH, EPD_PAGE_ROWS, 0, EPD_7IN3F_WHITE);
    Paint_SelectImage(gImageBuffer);
    Paint_Clear(WHITE);

    // Draw any lines that fall within this page
    for(int i=0;i<idx;i++) {
      if(lines[i].y >= pageY && lines[i].y < pageY + EPD_PAGE_ROWS) {
        Paint_DrawString_EN(20, lines[i].y - pageY, (char*)lines[i].txt.c_str(), (sFONT*)lines[i].font, WHITE, BLACK);
      }
    }

    // Stream this page to the display RAM
    EPD_7IN3F_DisplayPart(gImageBuffer, 0, pageY, EPD_7IN3F_WIDTH, EPD_PAGE_ROWS);
  }

  // Latch full frame
  EPD_7IN3F_Sleep();
}
#endif // end E-ink code block

void sendSensorDataToFirebase() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("✗ Cannot send data, WiFi not connected.");
    return;
  }

  HTTPClient http;
  // Use PATCH to update only the sensorData node within the device's data
  String url = String(FIREBASE_HOST) + "/tracking_box/" + DEVICE_ID + ".json?auth=" + FIREBASE_AUTH;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  // Create JSON document for the PATCH request
  DynamicJsonDocument patchDoc(1024);
  
  // Create the nested sensorData object
  JsonObject sensorData = patchDoc.createNestedObject("sensorData");
  sensorData["timestamp"] = millis();
  sensorData["temp"] = currentData.temperature;
  sensorData["humidity"] = currentData.humidity;
  sensorData["batteryVoltage"] = currentData.batteryVoltage;
  sensorData["gpsFixValid"] = currentData.gpsFixValid;
  sensorData["limitSwitchPressed"] = currentData.limitSwitchPressed;
  sensorData["tiltDetected"] = currentData.tiltDetected;
  sensorData["fallDetected"] = currentData.fallDetected;
  sensorData["wakeUpReason"] = currentData.wakeUpReason;
  sensorData["buzzerIsActive"] = currentData.buzzerIsActive;
  
  JsonObject accelerometer = sensorData.createNestedObject("accelerometer");
  accelerometer["x"] = currentData.accelX;
  accelerometer["y"] = currentData.accelY;
  accelerometer["z"] = currentData.accelZ;
  
  if (currentData.gpsFixValid) {
    JsonObject location = sensorData.createNestedObject("location");
    location["lat"] = currentData.latitude;
    location["lng"] = currentData.longitude;
    location["alt"] = currentData.altitude;
  }
  
  sensorData["currentLocation"] = currentData.currentLocation;

  // Serialize JSON to string for sending
  String jsonPayload;
  serializeJson(patchDoc, jsonPayload);
  Serial.println("Sending to Firebase: " + jsonPayload);

  // Send the PATCH request
  int httpResponseCode = http.PATCH(jsonPayload);

  if (httpResponseCode > 0) {
    Serial.printf("✓ Firebase PATCH successful, response code: %d\n", httpResponseCode);
  } else {
    Serial.printf("✗ Firebase PATCH failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
  }

  http.end();
}

void sendBreachUpdateToFirebase() {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  String url = String(FIREBASE_HOST) + "/tracking_box/" + DEVICE_ID + "/sensorData.json?auth=" + FIREBASE_AUTH;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  DynamicJsonDocument doc(256);
  doc["buzzerIsActive"] = currentData.buzzerIsActive;
  doc["wakeUpReason"]  = currentData.wakeUpReason;
  doc["buzzerDismissed"] = false;
  String payload; serializeJson(doc, payload);
  http.PATCH(payload);
  http.end();
}

void fetchDataForDisplay() {
  HTTPClient http;
  String url = String(FIREBASE_HOST) + "/tracking_box/" + DEVICE_ID + ".json";
  http.begin(url);
  
  int httpResponseCode = http.GET();
  if (httpResponseCode == HTTP_CODE_OK) {
    String response = http.getString();
    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, response);
    
    if (!error) {
      // Fetch details
      currentData.deviceName = doc["details"]["name"] | "Unknown";
      currentData.deviceSetLocation = doc["details"]["setLocation"] | "Unknown";
      currentData.deviceDescription = doc["details"]["description"] | "No Description";

      // Fetch latest sensor readings that were just sent
      currentData.temperature = doc["sensorData"]["temp"] | 0.0;
      currentData.humidity = doc["sensorData"]["humidity"] | 0.0;
      currentData.batteryVoltage = doc["sensorData"]["batteryVoltage"] | 0.0;
      currentData.currentLocation = doc["sensorData"]["currentLocation"] | "Unknown";
      currentData.accelX = doc["sensorData"]["accelerometer"]["x"] | 0.0;
      currentData.accelY = doc["sensorData"]["accelerometer"]["y"] | 0.0;
      currentData.accelZ = doc["sensorData"]["accelerometer"]["z"] | 0.0;

      // ------------------------------------------------------------------
      // Tilt & free-fall detection – logic from accel-gyro.ino
      // ------------------------------------------------------------------
      currentData.tiltDetected = (currentData.accelZ < TILT_THRESHOLD_Z_AXIS);

      float accelMagnitude = sqrt(currentData.accelX * currentData.accelX +
                                  currentData.accelY * currentData.accelY +
                                  currentData.accelZ * currentData.accelZ);
      currentData.fallDetected = (accelMagnitude < FALL_THRESHOLD_MAGNITUDE);

      // -------------------------------------------------------------
      // Evaluate LOCK BREACH after we have both limit switch state and
      // the desired setLocation from Firebase.
      // Condition: lid open (limitSwitchPressed == false) AND
      //            currentLocation string differs from the setLocation.
      // When true, sound buzzer continuously and set wake reason.
      // -------------------------------------------------------------
      bool lockBreach = (!currentData.limitSwitchPressed) &&
                        (currentData.currentLocation != currentData.deviceSetLocation);

      if (lockBreach) {
        currentData.wakeUpReason   = "LOCK BREACH";
        currentData.buzzerIsActive = true;
        digitalWrite(BUZZER_PIN, HIGH);  // sound continuously
        sendBreachUpdateToFirebase();
      } else {
        currentData.buzzerIsActive = false;
        digitalWrite(BUZZER_PIN, LOW);
      }

    } else {
      Serial.println("✗ Failed to parse Firebase JSON for display.");
    }
  } else {
    Serial.println("✗ Firebase data fetch failed.");
  }
  http.end();
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
  // Tilt & free-fall detection – logic from accel-gyro.ino
  // ------------------------------------------------------------------
  currentData.tiltDetected = (currentData.accelZ < TILT_THRESHOLD_Z_AXIS);

  float accelMagnitude = sqrt(currentData.accelX * currentData.accelX +
                              currentData.accelY * currentData.accelY +
                              currentData.accelZ * currentData.accelZ);
  currentData.fallDetected = (accelMagnitude < FALL_THRESHOLD_MAGNITUDE);

  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  currentData.limitSwitchPressed = !digitalRead(LIMIT_SWITCH_PIN);

  if (currentData.gpsFixValid) {
    currentData.currentLocation = String(currentData.latitude, 4) + ", " + String(currentData.longitude, 4);
  } else {
    currentData.currentLocation = "No GPS Fix";
  }
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
  uint8_t rawData[6];
  for (int i = 0; i < 6; i++) {
    rawData[i] = readLSM6DSLRegister(LSM6DSL_OUTX_L_XL + i);
  }

  int16_t rawX = (rawData[1] << 8) | rawData[0];
  int16_t rawY = (rawData[3] << 8) | rawData[2];
  int16_t rawZ = (rawData[5] << 8) | rawData[4];

  // ±2 g → sensitivity 0.061 mg/LSB  => 2 g / 32768
  const float SCALE = 2.0 / 32768.0;
  currentData.accelX = rawX * SCALE;
  currentData.accelY = rawY * SCALE;
  currentData.accelZ = rawZ * SCALE;
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
// HARDWARE INITIALIZATION
// =====================================================================
bool initializeAllHardware() {
  /* E-ink hardware initialisation removed */
  
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  
  #if ENABLE_SHT30_SENSOR
  Wire.begin(SHT30_SDA_PIN, SHT30_SCL_PIN);
  if (!sht30.begin(0x44)) {
    Serial.println("✗ SHT30 sensor initialization failed");
    return false;
  }
  #endif
  
  Wire.begin(LSM6DSL_SDA_PIN, LSM6DSL_SCL_PIN);
  if (!initLSM6DSL()) {
    Serial.println("✗ LSM6DSL accelerometer initialization failed");
    return false;
  }
  
  sim7600.begin(115200, SERIAL_8N1, SIM7600_RX_PIN, SIM7600_TX_PIN);
  delay(2000);
  flushSIM7600Buffer();
  
  // E-ink clear removed
  return true;
}

bool initLSM6DSL() {
  uint8_t addresses[] = {LSM6DSL_ADDR1, LSM6DSL_ADDR2};

  for (int i = 0; i < 2; i++) {
    lsm6dsl_address = addresses[i];

    // Verify sensor presence
    uint8_t whoAmI = readLSM6DSLRegister(LSM6DSL_WHO_AM_I);
    if (whoAmI == 0x6A) {
      // CTRL1_XL: 208 Hz ODR, ±2 g FS  -> 0x60
      writeLSM6DSLRegister(LSM6DSL_CTRL1_XL, 0x60);

      // CTRL3_C: Block-data-update = 1
      writeLSM6DSLRegister(LSM6DSL_CTRL3_C, 0x40);

      /* ---------- restore wake-up engine ---------- */

      // 1) turn on the slope high-pass filter (keeps gravity out)
      writeLSM6DSLRegister(LSM6DSL_CTRL8_XL, 0x80);   // HP_SLOPE_XL_EN = 1

      // 2) choose a threshold – 0x08 ≈ 0.50 g when FS = ±2 g
      writeLSM6DSLRegister(LSM6DSL_WAKE_UP_THS, 0x08);

      // 3) set minimum duration – 0x20 = 3 samples ≈ 14 ms at 208 Hz
      writeLSM6DSLRegister(LSM6DSL_WAKE_UP_DUR, 0x20);

      // 4) route the wake-up interrupt to INT1
      writeLSM6DSLRegister(LSM6DSL_MD1_CFG, 0x20);    // INT1_WU = 1

      delay(100); // Stabilise
      Serial.println("✓ LSM6DSL initialised (simple mode)");
      return true;
    }
  }
  return false;
}

uint8_t readLSM6DSLRegister(uint8_t reg) {
  Wire.beginTransmission(lsm6dsl_address);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) return 0xFF;
  Wire.requestFrom(lsm6dsl_address, (uint8_t)1);
  if (!Wire.available()) return 0xFF;
  return Wire.read();
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
#if 0   // E-INK CODE BLOCK DISABLED
void displayOfflineQRCode() {
  Serial.println("📱 Displaying offline QR code...");

  _preparePaint();
  Paint_DrawString_EN(20, 20, (char*)"OFFLINE MODE", (sFONT*)&Font24, WHITE, BLACK);
  Paint_DrawString_EN(20, 70, (char*)"No WiFi Connection", (sFONT*)&Font16, WHITE, BLACK);
  Paint_DrawString_EN(20, 100, (char*)"Scan QR for device details", (sFONT*)&Font16, WHITE, BLACK);

  // ------------------------------------------------------------------
  // Draw a REAL QR Code so a technician can simply scan the screen even
  // when the device is offline.  The code contains a local URL that is
  // served once the user joins the hotspot.
  // ------------------------------------------------------------------

  char url[64];
  sprintf(url, "tracking-box.local/qr/%s", DEVICE_ID.c_str());

  const uint8_t QR_VERSION = 3;             // 29×29 modules
  const uint8_t MODULE_SCALE = 8;           // each module = 8×8 pixels for larger display
  uint8_t qrBuffer[qrcode_getBufferSize(QR_VERSION)];
  QRCode qrcode;
  qrcode_initText(&qrcode, qrBuffer, QR_VERSION, ECC_MEDIUM, url);

  const int qrLeft = 450;  // Position QR on the right side
  const int qrTop  = 120;

  for (uint8_t y = 0; y < qrcode.size; y++) {
    for (uint8_t x = 0; x < qrcode.size; x++) {
      if (qrcode_getModule(&qrcode, x, y)) {
        int x0 = qrLeft + x * MODULE_SCALE;
        int y0 = qrTop  + y * MODULE_SCALE;
        Paint_DrawRectangle(x0, y0, x0 + MODULE_SCALE - 1, y0 + MODULE_SCALE - 1, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
      }
    }
  }

  char buf[64];
  sprintf(buf, "URL: %s", url);
  Paint_DrawString_EN(20, 440, buf, (sFONT*)&Font12, WHITE, BLACK);

  updateEInkDisplay(); // Use paged drawing for offline QR

  Serial.println("💡 Offline QR placeholder drawn");

  // Sound offline notification
  buzzerAlert(4, 150);
}
#endif // end E-ink code block

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
// DEEP SLEEP PREPARATION
// =====================================================================
void prepareForDeepSleep() {
  Serial.println("🛏️ Preparing for deep sleep mode...");
  
  // Configure wake-up sources
  esp_sleep_enable_timer_wakeup(SLEEP_TIME_US); // Timer wake (15 minutes)
  // Wake when the limit-switch line goes HIGH (lid opened)
  esp_sleep_enable_ext1_wakeup(1ULL << LIMIT_SWITCH_PIN, ESP_EXT1_WAKEUP_ANY_HIGH);
  
  // Enable the accelerometer wake-up *unconditionally* so any motion
  // that crosses the configured threshold will wake the ESP32.
  // esp_sleep_enable_ext0_wakeup((gpio_num_t)LSM6DSL_INT1_PIN, 0);   // Accelerometer interrupt wake (active low)
  Serial.println("📳 Accelerometer wake-up: Enabled (always)");
  rtc_gpio_pullup_en((gpio_num_t)LSM6DSL_INT1_PIN);
  rtc_gpio_hold_en((gpio_num_t)LSM6DSL_INT1_PIN);
  
  Serial.println("⏰ Timer wake-up: " + String(SLEEP_MINUTES) + " minutes");
  Serial.println("🔘 Limit switch wake-up: Enabled (GPIO" + String(LIMIT_SWITCH_PIN) + ")");
  
  // Ensure WiFi is disconnected
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  Serial.println("Sleep configuration complete");
  Serial.println("Next wake: Timer (" + String(SLEEP_MINUTES) + " min) OR Interrupt");
}

// =====================================================================
// UTILITY FUNCTIONS
// =====================================================================
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
// Waveshare-based rendering functions -------------------------------------------------
// ---------------------------------------------------------------------------
#if 0   // E-INK CODE BLOCK DISABLED
void _preparePaint() {
  if (!gImageBuffer) return;
  Paint_SelectImage(gImageBuffer);
  Paint_Clear(WHITE);
}

void _showAndSleep() {} // no-op; kept for offline QR which now uses paged drawing directly
#endif // end E-ink code block

void determineWakeUpReason() {
  esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
  switch(wakeup_cause) {
    case ESP_SLEEP_WAKEUP_TIMER:
      currentData.wakeUpReason = "TIMER DUE (15mns.)";
      break;
    case ESP_SLEEP_WAKEUP_EXT0:
      currentData.wakeUpReason = "SHOCK DETECTED";
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
// END OF TRACKING BOX MAIN FIRMWARE
// ===================================================================== 

bool checkDismissCommand() {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  String url = String(FIREBASE_HOST) + "/tracking_box/" + DEVICE_ID + "/dismissAlert.json";
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