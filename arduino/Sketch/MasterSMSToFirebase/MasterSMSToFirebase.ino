/*
 * =====================================================================
 * MASTER SMS-TO-FIREBASE DEVICE
 * =====================================================================
 * 
 * This sketch acts as a Master device that:
 * 1. Lists unread SMS messages using AT+CMGL="REC UNREAD"
 * 2. Reads each message individually using AT+CMGR=<index>
 * 3. Parses the SMS content to extract sensor data
 * 4. Converts the data to Firebase JSON format
 * 5. Updates Firebase database with the received data
 * 6. Deletes successfully processed messages using AT+CMGD=<index>
 * 
 * SMS PROCESSING WORKFLOW:
 * - AT+CMGL="REC UNREAD" → Get list of unread message indices
 * - AT+CMGR=<index> → Read specific message content
 * - Parse sensor data from message content
 * - Upload to Firebase database
 * - AT+CMGD=<index> → Delete processed message
 * 
 * SMS Format Expected (comma-separated):
 * "DEVICE_ID,timestamp,temp,humidity,lat,lng,alt,tilt,fall,limitSwitch,accelX,accelY,accelZ,batteryVoltage,wakeUpReason"
 * 
 * =====================================================================
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <time.h>

// =====================================================================
// PIN DEFINITIONS
// =====================================================================
#define SIM_RX 16
#define SIM_TX 17
#define SIM_BAUD 115200

// =====================================================================
// NETWORK & FIREBASE CONFIGURATION
// =====================================================================
const char* WIFI_SSID = "archer_2.4G";
const char* WIFI_PASSWORD = "05132000";
const char* FIREBASE_HOST = "https://tracking-box-e17a1-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* FIREBASE_AUTH = "AIzaSyBQje281bPAt7MiJdK94ru1irAU8i3luzY";

// =====================================================================
// GLOBAL OBJECTS
// =====================================================================
HardwareSerial sim7600(1);

// Structure to hold parsed SMS data
struct SMSData {
  String deviceID = "";
  uint64_t timestamp = 0;
  float temperature = 0.0;
  float humidity = 0.0;
  double latitude = 0.0;
  double longitude = 0.0;
  float altitude = 0.0;
  bool tiltDetected = false;
  bool fallDetected = false;
  bool limitSwitchPressed = false;
  float accelX = 0.0;
  float accelY = 0.0;
  float accelZ = 0.0;
  float batteryVoltage = 0.0;
  String wakeUpReason = "";
  bool gpsFixValid = false;
};

// =====================================================================
// FUNCTION DECLARATIONS
// =====================================================================
void sendAT(const char *cmd, int waitTime);
void readModem();
void readNewestSMS();
bool readSpecificSMS(int messageIndex);
void deleteSMS(int messageIndex);
bool parseSMSData(String smsContent, SMSData &data);
void sendDataToFirebase(const SMSData &data);
bool connectToWiFi();
void flushSIM7600Buffer();
String extractSMSContent(String response);
String extractSMSContentFromCMGR(String response);
void testSMSFunctionality();

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("===== MASTER SMS-TO-FIREBASE DEVICE =====");
  Serial.println("Initializing SIM7600...");
  
  // Initialize SIM7600
  sim7600.begin(SIM_BAUD, SERIAL_8N1, SIM_RX, SIM_TX);
  delay(2000);
  
  // Configure SIM7600 for SMS with verification
  Serial.println("Testing SIM7600 connectivity...");
  sendAT("AT", 1000);
  
  Serial.println("Checking SIM card status...");
  sendAT("AT+CPIN?", 1000);                  // Check SIM card status
  
  Serial.println("Checking network registration...");
  sendAT("AT+CREG?", 1000);                  // Check network registration
  
  Serial.println("Configuring SMS settings...");
  sendAT("AT+CMGF=1", 1000);                 // Text mode
  sendAT("AT+CPMS=\"SM\",\"SM\",\"SM\"", 1000); // Use SIM storage
  sendAT("AT+CNMI=1,2,0,0,0", 1000);        // New SMS indication
  
  Serial.println("Checking SMS storage status...");
  sendAT("AT+CPMS?", 1000);                  // Check SMS storage status
  
  Serial.println("SIM7600 SMS configuration complete");
  
  // Test SMS functionality
  testSMSFunctionality();
  
  // Connect to WiFi
  if (connectToWiFi()) {
    Serial.println("✅ WiFi Connected - Ready to receive SMS");
    
    // Configure time for timestamps
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    struct tm timeinfo;
    for (uint8_t i = 0; i < 10; ++i) {
      if (getLocalTime(&timeinfo, 500)) break;
    }
    if (timeinfo.tm_year > 120) {
      Serial.printf("✓ Time synced: %04d-%02d-%02d %02d:%02d\n",
                    timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                    timeinfo.tm_hour, timeinfo.tm_min);
    }
  } else {
    Serial.println("❌ WiFi connection failed");
  }
  
  Serial.println("Starting SMS monitoring loop...");
  Serial.println("\n💡 DEBUG COMMANDS:");
  Serial.println("   Type 'sms' or 'check' to manually trigger SMS check");
  Serial.println("   Type 'help' for command list");
  Serial.println("   Device will automatically check for SMS every 3 seconds\n");
}

// =====================================================================
// MAIN LOOP
// =====================================================================
void loop() {
  // Check for manual SMS check command from Serial
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    if (command == "sms" || command == "check" || command == "test") {
      Serial.println("🔍 Manual SMS check triggered...");
      readNewestSMS();
    } else if (command == "help") {
      Serial.println("Available commands:");
      Serial.println("  sms/check/test - Manual SMS check");
      Serial.println("  help - Show this help");
    }
  }
  
  // Check WiFi connection first and reconnect if needed
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠ WiFi disconnected, attempting reconnection...");
    if (!connectToWiFi()) {
      Serial.println("❌ WiFi reconnection failed, continuing SMS monitoring...");
    }
  }
  
  // Check for new SMS messages
  readNewestSMS();
  
  // Wait before next check
  delay(3000);  // Check every 3 seconds (increased from 2s for stability)
  
  // Periodic status report every 30 seconds
  static unsigned long lastStatusReport = 0;
  if (millis() - lastStatusReport > 30000) {
    Serial.println("📡 Master device active - WiFi: " + 
                   String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected") + 
                   " | Uptime: " + String(millis() / 1000) + "s");
    lastStatusReport = millis();
  }
  
  // Debug status every 15 seconds during development
  static unsigned long lastDebugReport = 0;
  if (millis() - lastDebugReport > 15000) {
    Serial.println("🔄 SMS monitoring active - checking every 3 seconds");
    lastDebugReport = millis();
  }
}

// =====================================================================
// SMS PROCESSING FUNCTIONS
// =====================================================================
void readNewestSMS() {
  // First, list unread messages to get their indices
  flushSIM7600Buffer();
  Serial.println("🔍 Checking for unread SMS messages...");
  sim7600.println("AT+CMGL=\"REC UNREAD\"");  // List unread messages
  delay(1000);

  if (sim7600.available()) {
    String listResponse = sim7600.readString();
    Serial.println("📋 CMGL Response: " + listResponse);
    
    if (listResponse.indexOf("+CMGL:") != -1) {
      Serial.println("=== UNREAD SMS DETECTED ===");
      Serial.println("List Response: " + listResponse);
      
      // Extract all message indices from the list
      int startPos = 0;
      while (true) {
        int cmglPos = listResponse.indexOf("+CMGL: ", startPos);
        if (cmglPos == -1) break;
        
        // Extract message index
        int indexStart = cmglPos + 7; // After "+CMGL: "
        int indexEnd = listResponse.indexOf(",", indexStart);
        if (indexEnd == -1) break;
        
        int messageIndex = listResponse.substring(indexStart, indexEnd).toInt();
        Serial.printf("Found unread message at index: %d\n", messageIndex);
        
        // Read the specific message using AT+CMGR
        if (readSpecificSMS(messageIndex)) {
          // Message was successfully processed and will be deleted
          deleteSMS(messageIndex);
        }
        
        startPos = indexEnd;
      }
      
      Serial.println("=========================");
    } else {
      Serial.println("📭 No unread SMS messages found");
    }
  } else {
    Serial.println("⚠ No response from SIM7600 to CMGL command");
  }
}

bool readSpecificSMS(int messageIndex) {
  Serial.printf("Reading SMS at index %d using AT+CMGR...\n", messageIndex);
  
  // Clear buffer and send AT+CMGR command
  flushSIM7600Buffer();
  sim7600.print("AT+CMGR=");
  sim7600.println(messageIndex);
  delay(1500); // Allow time for response
  
  String response = "";
  unsigned long timeout = millis() + 5000; // 5 second timeout
  
  while (millis() < timeout && sim7600.available()) {
    response += sim7600.readString();
    delay(100); // Small delay to ensure complete response
  }
  
  if (response.length() == 0) {
    Serial.println("❌ No response to AT+CMGR command");
    return false;
  }
  
  Serial.println("CMGR Response: " + response);
  
  // Check if command was successful
  if (response.indexOf("OK") == -1) {
    Serial.println("❌ AT+CMGR command failed");
    return false;
  }
  
  // Extract SMS content from CMGR response
  String smsContent = extractSMSContentFromCMGR(response);
  
  if (smsContent.length() > 0) {
    Serial.println("=== NEW SMS RECEIVED ===");
    Serial.println("SMS Content: " + smsContent);
    
    // Parse SMS data
    SMSData parsedData;
    if (parseSMSData(smsContent, parsedData)) {
      Serial.println("✓ SMS data parsed successfully");
      Serial.println("Device ID: " + parsedData.deviceID);
      
      // Send to Firebase
      if (WiFi.status() == WL_CONNECTED) {
        sendDataToFirebase(parsedData);
        return true; // Success - message can be deleted
      } else {
        Serial.println("❌ Cannot send to Firebase - WiFi not connected");
        return false; // Don't delete message if Firebase upload failed
      }
    } else {
      Serial.println("❌ Failed to parse SMS data");
      return false;
    }
  } else {
    Serial.println("❌ No valid SMS content found in CMGR response");
    return false;
  }
}

void deleteSMS(int messageIndex) {
  Serial.printf("Deleting SMS at index: %d\n", messageIndex);
  
  flushSIM7600Buffer();
  sim7600.print("AT+CMGD=");
  sim7600.println(messageIndex);
  delay(1000);
  
  // Check deletion response
  String response = "";
  unsigned long timeout = millis() + 3000;
  while (millis() < timeout && sim7600.available()) {
    response += sim7600.readString();
  }
  
  if (response.indexOf("OK") != -1) {
    Serial.println("✓ SMS deleted successfully");
  } else {
    Serial.println("❌ Failed to delete SMS");
  }
}

String extractSMSContent(String response) {
  // Find the actual SMS content after the header
  int contentStart = -1;
  int headerEnd = response.indexOf("+CMGL:");
  
  if (headerEnd != -1) {
    // Look for the end of the first line (header)
    int firstNewline = response.indexOf('\n', headerEnd);
    if (firstNewline != -1) {
      contentStart = firstNewline + 1;
    }
  }
  
  if (contentStart != -1) {
    int contentEnd = response.indexOf('\n', contentStart);
    if (contentEnd == -1) {
      contentEnd = response.length();
    }
    
    String content = response.substring(contentStart, contentEnd);
    content.trim();
    return content;
  }
  
  return "";
}

String extractSMSContentFromCMGR(String response) {
  // CMGR response format:
  // +CMGR: "REC UNREAD","+639184652918","","23/11/28,15:30:22+32"
  // SMS_CONTENT_HERE
  // 
  // OK
  
  int cmgrPos = response.indexOf("+CMGR:");
  if (cmgrPos == -1) {
    Serial.println("❌ No +CMGR header found");
    return "";
  }
  
  // Find the end of the CMGR header line
  int headerLineEnd = response.indexOf('\n', cmgrPos);
  if (headerLineEnd == -1) {
    Serial.println("❌ No newline after +CMGR header");
    return "";
  }
  
  // SMS content starts after the header line
  int contentStart = headerLineEnd + 1;
  
  // Find the end of SMS content (look for "OK" on its own line or end of response)
  int okPos = response.indexOf("\nOK", contentStart);
  int contentEnd;
  
  if (okPos != -1) {
    contentEnd = okPos;
  } else {
    // If no "OK" found, use end of response
    contentEnd = response.length();
  }
  
  if (contentStart >= contentEnd) {
    Serial.println("❌ No content found between header and OK");
    return "";
  }
  
  String content = response.substring(contentStart, contentEnd);
  content.trim();
  
  // Remove any remaining carriage returns or line feeds
  content.replace("\r", "");
  content.replace("\n", "");
  
  Serial.println("Extracted content: '" + content + "'");
  return content;
}

bool parseSMSData(String smsContent, SMSData &data) {
  // Expected format: "DEVICE_ID,timestamp,temp,humidity,lat,lng,alt,tilt,fall,limitSwitch,accelX,accelY,accelZ,batteryVoltage,wakeUpReason"
  
  int fieldIndex = 0;
  int lastComma = -1;
  
  for (int i = 0; i <= smsContent.length(); i++) {
    if (i == smsContent.length() || smsContent[i] == ',') {
      String field = smsContent.substring(lastComma + 1, i);
      field.trim();
      
      switch (fieldIndex) {
        case 0:  data.deviceID = field; break;
        case 1:  data.timestamp = field.toInt(); break;
        case 2:  data.temperature = field.toFloat(); break;
        case 3:  data.humidity = field.toFloat(); break;
        case 4:  data.latitude = field.toDouble(); break;
        case 5:  data.longitude = field.toDouble(); break;
        case 6:  data.altitude = field.toFloat(); break;
        case 7:  data.tiltDetected = (field == "1" || field.equalsIgnoreCase("true")); break;
        case 8:  data.fallDetected = (field == "1" || field.equalsIgnoreCase("true")); break;
        case 9:  data.limitSwitchPressed = (field == "1" || field.equalsIgnoreCase("true")); break;
        case 10: data.accelX = field.toFloat(); break;
        case 11: data.accelY = field.toFloat(); break;
        case 12: data.accelZ = field.toFloat(); break;
        case 13: data.batteryVoltage = field.toFloat(); break;
        case 14: data.wakeUpReason = field; break;
      }
      
      fieldIndex++;
      lastComma = i;
    }
  }
  
  // Validate required fields
  if (data.deviceID.length() == 0) {
    Serial.println("❌ Missing device ID in SMS");
    return false;
  }
  
  // Check if we have GPS coordinates
  data.gpsFixValid = (data.latitude != 0.0 && data.longitude != 0.0);
  
  Serial.printf("Parsed: ID=%s, Temp=%.1f°C, Hum=%.1f%%, GPS=%s, Batt=%.2fV\n",
                data.deviceID.c_str(), data.temperature, data.humidity,
                data.gpsFixValid ? "Valid" : "Invalid", data.batteryVoltage);
  
  return true;
}

// =====================================================================
// FIREBASE COMMUNICATION
// =====================================================================
void sendDataToFirebase(const SMSData &data) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("✗ Cannot send data, WiFi not connected.");
    return;
  }

  HTTPClient http;
  // Use PUT to replace the sensorData node for the specific device
  String url = String(FIREBASE_HOST) + "/tracking_box/" + data.deviceID + "/sensorData.json?auth=" + FIREBASE_AUTH;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  // Create JSON document matching TrackingBoxMain format
  DynamicJsonDocument doc(1024);
  
  // Use provided timestamp or current time
  uint64_t epochMs = data.timestamp > 0 ? data.timestamp : (uint64_t)time(nullptr) * 1000ULL;
  
  doc["timestamp"] = epochMs;
  doc["temp"] = data.temperature;
  doc["humidity"] = data.humidity;
  doc["batteryVoltage"] = data.batteryVoltage;
  doc["gpsFixValid"] = data.gpsFixValid;
  doc["limitSwitchPressed"] = data.limitSwitchPressed;
  doc["tiltDetected"] = data.tiltDetected;
  doc["fallDetected"] = data.fallDetected;
  doc["wakeUpReason"] = data.wakeUpReason;
  doc["receivedViaSMS"] = true; // Flag to indicate data came via SMS
  doc["masterDevice"] = "SMS_Gateway"; // Identify the master device
  
  JsonObject accelerometer = doc.createNestedObject("accelerometer");
  accelerometer["x"] = data.accelX;
  accelerometer["y"] = data.accelY;
  accelerometer["z"] = data.accelZ;
  
  if (data.gpsFixValid) {
    JsonObject location = doc.createNestedObject("location");
    location["lat"] = data.latitude;
    location["lng"] = data.longitude;
    location["alt"] = data.altitude;
    
    doc["currentLocation"] = String(data.latitude, 4) + ", " + String(data.longitude, 4);
  } else {
    doc["currentLocation"] = "GPS Unavailable";
  }

  // Serialize JSON to string for sending
  String jsonPayload;
  serializeJson(doc, jsonPayload);
  Serial.println("Sending to Firebase: " + jsonPayload);

  // Send the PUT request
  int httpResponseCode = http.PUT(jsonPayload);

  if (httpResponseCode > 0) {
    Serial.printf("✓ Firebase PUT successful for %s, response code: %d\n", 
                  data.deviceID.c_str(), httpResponseCode);
  } else {
    Serial.printf("✗ Firebase PUT failed for %s, error: %s\n", 
                  data.deviceID.c_str(), http.errorToString(httpResponseCode).c_str());
  }

  http.end();
}

// =====================================================================
// WIFI CONNECTION
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

// =====================================================================
// SIM7600 HELPER FUNCTIONS
// =====================================================================
void sendAT(const char *cmd, int waitTime) {
  sim7600.println(cmd);
  delay(waitTime);
  readModem();
}

void readModem() {
  while (sim7600.available()) {
    Serial.write(sim7600.read());
  }
}

void flushSIM7600Buffer() {
  while (sim7600.available()) {
    sim7600.read();
  }
}

void testSMSFunctionality() {
  Serial.println("\n🧪 TESTING SMS FUNCTIONALITY");
  Serial.println("===============================");
  
  // Test 1: Check if we can list ALL messages (not just unread)
  Serial.println("Test 1: Listing ALL SMS messages...");
  flushSIM7600Buffer();
  sim7600.println("AT+CMGL=\"ALL\"");
  delay(2000);
  
  if (sim7600.available()) {
    String response = sim7600.readString();
    Serial.println("ALL SMS Response: " + response);
    if (response.indexOf("+CMGL:") != -1) {
      Serial.println("✓ SMS storage has messages");
    } else {
      Serial.println("ℹ SMS storage appears empty");
    }
  } else {
    Serial.println("❌ No response to AT+CMGL=\"ALL\"");
  }
  
  // Test 2: Check SMS service center
  Serial.println("\nTest 2: Checking SMS service center...");
  sendAT("AT+CSCA?", 2000);
  
  // Test 3: Manual check for unread messages
  Serial.println("\nTest 3: Manual check for unread messages...");
  flushSIM7600Buffer();
  sim7600.println("AT+CMGL=\"REC UNREAD\"");
  delay(2000);
  
  if (sim7600.available()) {
    String unreadResponse = sim7600.readString();
    Serial.println("Unread SMS Response: " + unreadResponse);
  } else {
    Serial.println("❌ No response to unread SMS check");
  }
  
  Serial.println("===============================");
  Serial.println("🧪 SMS FUNCTIONALITY TEST COMPLETE\n");
} 