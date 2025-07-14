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
// E-ink support re-enabled
#if ENABLE_EINK
#include "DEV_Config.h"
#include "EPD.h"
#include "GUI_Paint.h"
#include "fonts.h"
#include "qrcode.h"   // Tiny library for generating QR bitmaps
#endif
#include <esp_heap_caps.h>

// E-ink configuration
#define ENABLE_EINK 0

// Display paging ---------------------------------------------------------
#if ENABLE_EINK
#define EPD_PAGE_ROWS  40   // E-ink paging enabled
#endif

#define ENABLE_SHT30_SENSOR 1

#if ENABLE_SHT30_SENSOR
#include "Adafruit_SHT31.h"
#endif

// Frame buffer pointer for GUI_Paint
#if ENABLE_EINK
static UBYTE *gImageBuffer = nullptr;
#endif

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
  
  determineWakeUpReason();

  // Main operational cycle: Connect > Init > Read > Sync > Sleep
  if (connectToWiFi()) {
    Serial.println("✅ WiFi Connected.");
    
    initializeAllHardware();
    Serial.println("✅ Hardware Initialized.");
    
    collectSensorReading();
    Serial.println("✅ Sensor Readings Collected.");
    
    // Fetch details needed for lock breach check
    fetchDeviceDetailsFromFirebase();
    fetchBuzzerStateFromFirebase(); // Fetch dismissal status

    // Evaluate lock breach condition based on sensors and Firebase data
    evaluateLockBreach();

    // Send all sensor and state data to Firebase
    sendSensorDataToFirebase();
    Serial.println("✅ Sensor Data Sent to Firebase.");

    // Update the E-Ink display with all collected data
#if ENABLE_EINK
    updateEInkDisplay();
    Serial.println("✅ E-ink Display Updated.");
#endif

    // If buzzer is active, enter monitoring loop until dismissed from the cloud
    handleBuzzerMonitoring();

    // Disconnect WiFi if not actively monitoring buzzer
    if (!currentData.buzzerIsActive) {
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
    }
  } else {
    Serial.println("❌ WiFi connection failed. Cannot sync with cloud.");
  }

  // Always go to sleep at the end of the cycle to conserve power
  Serial.println("Cycle complete. Entering deep sleep.");
  Serial.println("========================================\n");
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
#if ENABLE_EINK
void updateEInkDisplay() {
  if (!gImageBuffer) {
    Serial.println("E-ink buffer not allocated. Cannot update display.");
    return;
  }
  
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

  // Latch full frame and put display to sleep
  EPD_7IN3F_Sleep();
}
#endif // end E-ink code block

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

void evaluateLockBreach() {
  // A lock breach occurs if the lid is open AND the device's physical location
  // does not match its designated location from Firebase.
  // Note: Limit switch is pressed (true) when the lid is CLOSED.
  bool isLidOpen = !currentData.limitSwitchPressed;
  bool isLocationMismatch = (currentData.currentLocation != currentData.deviceSetLocation);
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
#if ENABLE_EINK
  // --- E-ink hardware initialisation re-enabled ---
  if (DEV_Module_Init() != 0) {
    Serial.println("✗ E-ink DEV_Module_Init FAILED");
    return false;
  }
  EPD_7IN3F_Init();
  EPD_7IN3F_Clear(EPD_7IN3F_WHITE);

  // Allocate memory for the display buffer
  gImageBuffer = (UBYTE *)heap_caps_malloc(EPD_7IN3F_WIDTH * EPD_PAGE_ROWS, MALLOC_CAP_DMA);
  if (!gImageBuffer) {
    Serial.println("✗ Failed to allocate memory for E-ink buffer!");
    return false;
  }
  // --- End E-ink init ---
#endif
  
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

  // Consolidated I2C initialization for all sensors
  Wire.begin(SHT30_SDA_PIN, SHT30_SCL_PIN);
  Wire.setClock(100000); // 100 kHz I2C like sht-gyro test sketch
  
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
  
  // E-ink clear removed
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
#if ENABLE_EINK
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
  
  // Re-enable the accelerometer wake-up on motion.
  esp_sleep_enable_ext0_wakeup((gpio_num_t)LSM6DSL_INT1_PIN, 1);   // Wake-up on motion interrupt (active high)
  Serial.println("📳 Accelerometer wake-up: Enabled (on motion)");
  
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
#if ENABLE_EINK
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