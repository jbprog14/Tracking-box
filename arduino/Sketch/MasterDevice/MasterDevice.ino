/*
 * =====================================================================
 * MASTER DEVICE - SMS TO FIREBASE RELAY WITH BIDIRECTIONAL CONTROL
 * =====================================================================
 * 
 * This Master device acts as a relay between Slave tracking devices
 * and Firebase when Slaves cannot connect to WiFi. It provides:
 * 
 * 1. SMS Reception: Receives sensor data from multiple Slave devices
 * 2. Firebase Upload: Reconstructs full JSON structure and uploads
 * 3. Control Monitoring: Watches Firebase for control state changes
 * 4. SMS Commands: Sends control commands back to Slave devices
 * 
 * The Master maintains bidirectional communication, allowing remote
 * control of buzzer alarms and solenoid locks even in SMS-only mode.
 * 
 * =====================================================================
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <map>

// =====================================================================
// PIN DEFINITIONS
// =====================================================================
#define SIM_RX 16
#define SIM_TX 17
#define SIM_BAUD 115200
#define LED_PIN 2  // Status LED

// =====================================================================
// NETWORK & FIREBASE CONFIGURATION
// =====================================================================
const char* WIFI_SSID = "archer_2.4G";
const char* WIFI_PASSWORD = "05132000";
const char* FIREBASE_HOST = "https://tracking-box-e17a1-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* FIREBASE_AUTH = "AIzaSyBQje281bPAt7MiJdK94ru1irAU8i3luzY";

// =====================================================================
// GLOBAL OBJECTS & VARIABLES
// =====================================================================
HardwareSerial sim7600(1);

// Structure to hold device data
struct DeviceData {
  // Core sensor data (Type A message)
  String deviceId;
  uint64_t timestamp;
  float temperature;
  float humidity;
  double latitude;
  double longitude;
  float altitude;
  bool tiltDetected;
  bool fallDetected;
  bool limitSwitchPressed;
  bool solenoidActive;
  bool buzzerIsActive;
  bool coarseFix;
  bool usingCGPS;
  String wakeUpReason;
  
  // Extended data (Type B message)
  float accelX;
  float accelY;
  float accelZ;
  float batteryVoltage;
  bool buzzerDismissed;
  
  // Metadata
  String phoneNumber;
  unsigned long lastUpdate;
  bool hasExtendedData;
};

// Store data for multiple devices
std::map<String, DeviceData> devices;

// Control states from Firebase
struct ControlState {
  bool buzzer;
  bool solenoid;
  bool buzzerDismissed;
  unsigned long lastCheck;
};

std::map<String, ControlState> deviceControls;

// =====================================================================
// FUNCTION DECLARATIONS
// =====================================================================
void connectToWiFi();
void configureModem();
void checkForSMS();
void parseSMS(String phoneNumber, String message);
void parseTypeA(DeviceData &device, String message);
void parseTypeB(DeviceData &device, String message);
void uploadToFirebase(const DeviceData &device);
void checkFirebaseControls();
void sendControlSMS(String phoneNumber, const ControlState &controls);
void sendAT(const char *cmd, int waitTime);
bool sendSMS(String phoneNumber, String message);
void blinkLED(int times);

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("\n===== MASTER DEVICE - SMS TO FIREBASE RELAY =====");
  
  // Connect to WiFi
  connectToWiFi();
  
  // Initialize SIM7600
  sim7600.begin(SIM_BAUD, SERIAL_8N1, SIM_RX, SIM_TX);
  delay(3000);
  
  configureModem();
  
  Serial.println("\n✅ Master device ready. Monitoring SMS and Firebase...");
  blinkLED(3);
}

// =====================================================================
// MAIN LOOP
// =====================================================================
void loop() {
  // Check for incoming SMS from slave devices
  checkForSMS();
  
  // Check Firebase for control state changes every 10 seconds
  static unsigned long lastControlCheck = 0;
  if (millis() - lastControlCheck > 10000) {
    lastControlCheck = millis();
    checkFirebaseControls();
  }
  
  // Heartbeat LED
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 5000) {
    lastBlink = millis();
    blinkLED(1);
  }
  
  delay(1000);
}

// =====================================================================
// WIFI CONNECTION
// =====================================================================
void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\n✅ WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// =====================================================================
// MODEM CONFIGURATION
// =====================================================================
void configureModem() {
  Serial.println("Configuring SIM7600...");
  
  sendAT("AT", 1000);
  sendAT("AT+CMGF=1", 1000);  // Text mode
  sendAT("AT+CPMS=\"SM\",\"SM\",\"SM\"", 1000);  // Use SIM storage
  sendAT("AT+CNMI=1,2,0,0,0", 1000);  // New SMS indication
  
  // Clear any existing messages
  sendAT("AT+CMGD=1,4", 2000);  // Delete all messages
  
  Serial.println("✅ SIM7600 configured");
}

// =====================================================================
// SMS RECEPTION AND PARSING
// =====================================================================
void checkForSMS() {
  sim7600.println("AT+CMGL=\"ALL\"");
  delay(1000);
  
  if (sim7600.available()) {
    String response = sim7600.readString();
    
    // Process all messages in the response
    int pos = 0;
    while ((pos = response.indexOf("+CMGL:", pos)) != -1) {
      // Extract message index
      int indexStart = pos + 7;
      int indexEnd = response.indexOf(",", indexStart);
      int smsIndex = response.substring(indexStart, indexEnd).toInt();
      
      // Extract phone number
      int phoneStart = response.indexOf("\"+", indexEnd) + 2;
      int phoneEnd = response.indexOf("\"", phoneStart);
      String phoneNumber = response.substring(phoneStart, phoneEnd);
      
      // Find message content (next line after header)
      int msgStart = response.indexOf("\n", phoneEnd) + 1;
      int msgEnd = response.indexOf("\n", msgStart);
      if (msgEnd == -1) msgEnd = response.length();
      
      String message = response.substring(msgStart, msgEnd);
      message.trim();
      
      if (message.length() > 0) {
        Serial.println("\n📨 SMS from " + phoneNumber + ": " + message);
        
        // Parse the SMS content
        parseSMS(phoneNumber, message);
        
        // Delete the processed SMS
        sim7600.print("AT+CMGD=");
        sim7600.println(smsIndex);
        delay(500);
      }
      
      pos = msgEnd;
    }
  }
}

void parseSMS(String phoneNumber, String message) {
  // Check message type
  if (message.startsWith("A,")) {
    // Type A: Core sensor data
    String deviceId = message.substring(2, message.indexOf(',', 2));
    
    DeviceData &device = devices[deviceId];
    device.phoneNumber = phoneNumber;
    device.lastUpdate = millis();
    
    parseTypeA(device, message);
    
    // Upload to Firebase immediately for Type A messages
    uploadToFirebase(device);
    
    Serial.println("✅ Type A data processed for " + deviceId);
    
  } else if (message.startsWith("B,")) {
    // Type B: Extended data
    String deviceId = message.substring(2, message.indexOf(',', 2));
    
    if (devices.find(deviceId) != devices.end()) {
      DeviceData &device = devices[deviceId];
      parseTypeB(device, message);
      
      // Upload updated data to Firebase
      uploadToFirebase(device);
      
      Serial.println("✅ Type B data processed for " + deviceId);
    }
  }
}

void parseTypeA(DeviceData &device, String message) {
  // Format: "A,deviceId,timestamp,temp,hum,lat,lng,alt,tilt,fall,lim,sol,buzz,coarse,cgps,wake"
  
  int pos = 2;  // Skip "A,"
  int nextPos;
  
  // Device ID
  nextPos = message.indexOf(',', pos);
  device.deviceId = message.substring(pos, nextPos);
  pos = nextPos + 1;
  
  // Timestamp
  nextPos = message.indexOf(',', pos);
  device.timestamp = message.substring(pos, nextPos).toInt() * 1000ULL;  // Convert to milliseconds
  pos = nextPos + 1;
  
  // Temperature
  nextPos = message.indexOf(',', pos);
  device.temperature = message.substring(pos, nextPos).toFloat();
  pos = nextPos + 1;
  
  // Humidity
  nextPos = message.indexOf(',', pos);
  device.humidity = message.substring(pos, nextPos).toFloat();
  pos = nextPos + 1;
  
  // Latitude
  nextPos = message.indexOf(',', pos);
  device.latitude = message.substring(pos, nextPos).toDouble();
  pos = nextPos + 1;
  
  // Longitude
  nextPos = message.indexOf(',', pos);
  device.longitude = message.substring(pos, nextPos).toDouble();
  pos = nextPos + 1;
  
  // Altitude
  nextPos = message.indexOf(',', pos);
  device.altitude = message.substring(pos, nextPos).toFloat();
  pos = nextPos + 1;
  
  // Boolean flags
  nextPos = message.indexOf(',', pos);
  device.tiltDetected = (message.substring(pos, nextPos) == "1");
  pos = nextPos + 1;
  
  nextPos = message.indexOf(',', pos);
  device.fallDetected = (message.substring(pos, nextPos) == "1");
  pos = nextPos + 1;
  
  nextPos = message.indexOf(',', pos);
  device.limitSwitchPressed = (message.substring(pos, nextPos) == "1");
  pos = nextPos + 1;
  
  nextPos = message.indexOf(',', pos);
  device.solenoidActive = (message.substring(pos, nextPos) == "1");
  pos = nextPos + 1;
  
  nextPos = message.indexOf(',', pos);
  device.buzzerIsActive = (message.substring(pos, nextPos) == "1");
  pos = nextPos + 1;
  
  nextPos = message.indexOf(',', pos);
  device.coarseFix = (message.substring(pos, nextPos) == "1");
  pos = nextPos + 1;
  
  nextPos = message.indexOf(',', pos);
  device.usingCGPS = (message.substring(pos, nextPos) == "1");
  pos = nextPos + 1;
  
  // Wake reason (remainder of string)
  String wakeAbbrev = message.substring(pos);
  if (wakeAbbrev == "TMR") device.wakeUpReason = "TIMER DUE (15mns.)";
  else if (wakeAbbrev == "MOT") device.wakeUpReason = "MOTION DETECTED";
  else if (wakeAbbrev == "BRH") device.wakeUpReason = "LOCK BREACH";
  else if (wakeAbbrev == "BOT") device.wakeUpReason = "FIRST BOOT";
  else device.wakeUpReason = wakeAbbrev;
}

void parseTypeB(DeviceData &device, String message) {
  // Format: "B,deviceId,accelX,accelY,accelZ,batteryV,buzzerDismissed"
  
  int pos = 2;  // Skip "B,"
  int nextPos;
  
  // Skip device ID
  nextPos = message.indexOf(',', pos);
  pos = nextPos + 1;
  
  // Accelerometer X
  nextPos = message.indexOf(',', pos);
  device.accelX = message.substring(pos, nextPos).toFloat();
  pos = nextPos + 1;
  
  // Accelerometer Y
  nextPos = message.indexOf(',', pos);
  device.accelY = message.substring(pos, nextPos).toFloat();
  pos = nextPos + 1;
  
  // Accelerometer Z
  nextPos = message.indexOf(',', pos);
  device.accelZ = message.substring(pos, nextPos).toFloat();
  pos = nextPos + 1;
  
  // Battery voltage
  nextPos = message.indexOf(',', pos);
  device.batteryVoltage = message.substring(pos, nextPos).toFloat();
  pos = nextPos + 1;
  
  // Buzzer dismissed
  device.buzzerDismissed = (message.substring(pos) == "1");
  device.hasExtendedData = true;
}

// =====================================================================
// FIREBASE UPLOAD
// =====================================================================
void uploadToFirebase(const DeviceData &device) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ Cannot upload - WiFi not connected");
    return;
  }
  
  HTTPClient http;
  
  // Build the full JSON structure matching Firebase schema
  DynamicJsonDocument doc(2048);
  
  // Update sensorData
  String sensorUrl = String(FIREBASE_HOST) + "/tracking_box/" + device.deviceId + "/sensorData.json?auth=" + FIREBASE_AUTH;
  http.begin(sensorUrl);
  http.addHeader("Content-Type", "application/json");
  
  doc["timestamp"] = device.timestamp;
  doc["temp"] = device.temperature;
  doc["humidity"] = device.humidity;
  doc["batteryVoltage"] = device.batteryVoltage;
  doc["gpsFixValid"] = (device.latitude != 0 && device.longitude != 0);
  doc["usingCGPS"] = device.usingCGPS;
  doc["coarseFix"] = device.coarseFix;
  doc["limitSwitchPressed"] = device.limitSwitchPressed;
  doc["tiltDetected"] = device.tiltDetected;
  doc["fallDetected"] = device.fallDetected;
  doc["wakeUpReason"] = device.wakeUpReason;
  doc["buzzerIsActive"] = device.buzzerIsActive;
  doc["buzzerDismissed"] = device.buzzerDismissed;
  
  // Add accelerometer data if available
  if (device.hasExtendedData) {
    JsonObject accel = doc.createNestedObject("accelerometer");
    accel["x"] = device.accelX;
    accel["y"] = device.accelY;
    accel["z"] = device.accelZ;
  }
  
  // Add location if valid
  if (device.latitude != 0 && device.longitude != 0) {
    JsonObject location = doc.createNestedObject("location");
    location["lat"] = device.latitude;
    location["lng"] = device.longitude;
    location["alt"] = device.altitude;
    
    // Update currentLocation string
    doc["currentLocation"] = String(device.latitude, 6) + ", " + String(device.longitude, 6);
  } else {
    doc["currentLocation"] = "GPS Initializing. Please Wait. . .";
  }
  
  String jsonPayload;
  serializeJson(doc, jsonPayload);
  
  int httpCode = http.PUT(jsonPayload);
  
  if (httpCode > 0) {
    Serial.println("✅ Firebase upload successful for " + device.deviceId);
  } else {
    Serial.println("❌ Firebase upload failed: " + http.errorToString(httpCode));
  }
  
  http.end();
}

// =====================================================================
// FIREBASE CONTROL MONITORING
// =====================================================================
void checkFirebaseControls() {
  // Check control states for all known devices
  for (auto &pair : devices) {
    String deviceId = pair.first;
    DeviceData &device = pair.second;
    
    // Skip if we haven't heard from this device recently (> 30 minutes)
    if (millis() - device.lastUpdate > 1800000) continue;
    
    HTTPClient http;
    
    // Check buzzer control
    String buzzerUrl = String(FIREBASE_HOST) + "/tracking_box/" + deviceId + "/controlFlags/buzzer.json?auth=" + FIREBASE_AUTH;
    http.begin(buzzerUrl);
    
    int httpCode = http.GET();
    bool newBuzzerState = false;
    
    if (httpCode == HTTP_CODE_OK) {
      String response = http.getString();
      newBuzzerState = (response == "true");
    }
    http.end();
    
    // Check solenoid control
    String solenoidUrl = String(FIREBASE_HOST) + "/tracking_box/" + deviceId + "/controlFlags/solenoid.json?auth=" + FIREBASE_AUTH;
    http.begin(solenoidUrl);
    
    httpCode = http.GET();
    bool newSolenoidState = false;
    
    if (httpCode == HTTP_CODE_OK) {
      String response = http.getString();
      newSolenoidState = (response == "true");
    }
    http.end();
    
    // Check buzzer dismissed state
    String dismissUrl = String(FIREBASE_HOST) + "/tracking_box/" + deviceId + "/sensorData/buzzerDismissed.json?auth=" + FIREBASE_AUTH;
    http.begin(dismissUrl);
    
    httpCode = http.GET();
    bool newDismissState = false;
    
    if (httpCode == HTTP_CODE_OK) {
      String response = http.getString();
      newDismissState = (response == "true");
    }
    http.end();
    
    // Check if control states have changed
    ControlState &controls = deviceControls[deviceId];
    
    if (controls.buzzer != newBuzzerState || 
        controls.solenoid != newSolenoidState ||
        controls.buzzerDismissed != newDismissState ||
        controls.lastCheck == 0) {
      
      // Update stored states
      controls.buzzer = newBuzzerState;
      controls.solenoid = newSolenoidState;
      controls.buzzerDismissed = newDismissState;
      controls.lastCheck = millis();
      
      // Send control SMS to slave device
      Serial.println("📤 Sending control SMS to " + deviceId);
      sendControlSMS(device.phoneNumber, controls);
      
      // Clear solenoid flag after sending
      if (newSolenoidState) {
        delay(2000);
        http.begin(solenoidUrl);
        http.addHeader("Content-Type", "application/json");
        http.PUT("false");
        http.end();
      }
    }
  }
}

// =====================================================================
// SMS SENDING
// =====================================================================
void sendControlSMS(String phoneNumber, const ControlState &controls) {
  // Format: "CMD,buzzer,solenoid,dismiss"
  String message = "CMD,";
  message += controls.buzzer ? "1" : "0";
  message += ",";
  message += controls.solenoid ? "1" : "0";
  message += ",";
  message += controls.buzzerDismissed ? "1" : "0";
  
  if (sendSMS(phoneNumber, message)) {
    Serial.println("✅ Control SMS sent successfully");
    blinkLED(2);
  } else {
    Serial.println("❌ Failed to send control SMS");
  }
}

bool sendSMS(String phoneNumber, String message) {
  sim7600.println("AT+CMGF=1");
  delay(500);
  
  sim7600.print("AT+CMGS=\"");
  sim7600.print(phoneNumber);
  sim7600.println("\"");
  delay(1000);
  
  // Wait for prompt
  unsigned long timeout = millis() + 5000;
  bool promptFound = false;
  
  while (millis() < timeout && !promptFound) {
    if (sim7600.available()) {
      char c = sim7600.read();
      if (c == '>') {
        promptFound = true;
      }
    }
  }
  
  if (!promptFound) return false;
  
  // Send message
  sim7600.print(message);
  delay(500);
  sim7600.write(26);  // Ctrl+Z
  
  // Wait for confirmation
  delay(5000);
  
  String response = "";
  while (sim7600.available()) {
    response += sim7600.readString();
  }
  
  return (response.indexOf("OK") != -1 || response.indexOf("+CMGS:") != -1);
}

// =====================================================================
// UTILITY FUNCTIONS
// =====================================================================
void sendAT(const char *cmd, int waitTime) {
  Serial.print(">> ");
  Serial.println(cmd);
  
  sim7600.println(cmd);
  delay(waitTime);
  
  String response = "";
  while (sim7600.available()) {
    response += (char)sim7600.read();
  }
  
  if (response.length() > 0) {
    Serial.print("<< ");
    Serial.println(response);
  }
}

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    if (i < times - 1) delay(100);
  }
}