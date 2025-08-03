/*
 * =====================================================================
 * MASTER SMS RECEIVER - DEBUG TOOL
 * =====================================================================
 * 
 * This sketch acts as a simple Master device that only receives and
 * displays SMS messages from any sender. It does not parse the
 * content or communicate with Firebase.
 *
 * This is useful for debugging the SMS sending functionality of the
 * TrackingBoxMain.ino sketch.
 *
 * =====================================================================
 *
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager

// =====================================================================
// PIN DEFINITIONS
// =====================================================================
#define SIM_RX 16
#define SIM_TX 17
#define SIM_BAUD 115200

// =====================================================================
// GLOBAL OBJECTS
// =====================================================================
HardwareSerial sim7600(1);
unsigned long lastSMSCleanup = 0;
const unsigned long SMS_CLEANUP_INTERVAL = 10 * 60 * 1000; // 10 minutes in milliseconds

// Device tracking for monitoring control states
struct ActiveDevice {
  String deviceId;
  String phoneNumber;
  bool buzzerActive;
  bool buzzerDismissed;
  bool solenoidActive;
  unsigned long lastChecked;
};

const int MAX_DEVICES = 10;
ActiveDevice activeDevices[MAX_DEVICES];
int activeDeviceCount = 0;
unsigned long lastFirebaseCheck = 0;
const unsigned long FIREBASE_CHECK_INTERVAL = 10000; // Check every 10 seconds


// ---------------------------------------------------------------------
// FIREBASE CONFIGURATION
// ---------------------------------------------------------------------
const char* FIREBASE_HOST = "https://tracking-box-e17a1-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* FIREBASE_AUTH = "AIzaSyBQje281bPAt7MiJdK94ru1irAU8i3luzY";

// WiFiManager instance
WiFiManager wifiManager;


// ---------------------------------------------------------------------
// DEBUG HELPER FOR FIREBASE ONLY
// ---------------------------------------------------------------------
const bool DEBUG_FB = true;
inline void DBG_FB(const String &msg) { if (DEBUG_FB) Serial.println(msg); }


// =====================================================================
// FUNCTION DECLARATIONS
// =====================================================================
String readModem();
void readAllSMS();
String sendAT(const char *cmd, int waitTime);
void showSignalQuality();
void readSMS(int index);
String extractSMSContent(const String &resp);
String csvToFirebaseJson(const String &csv);
String extractFieldFromCSV(const String &csv, int fieldIndex);
String extractPhoneNumberFromCMGR(const String &resp);
bool connectToWiFi();
void sendJsonToFirebase(const String &json, const String &deviceId, const String &phoneNumber);
void sendControlSMS(const String &phoneNumber, const String &message);
void sendSolenoidToFirebase(const String &deviceId, bool state);
void cleanupSMSMemory();
void showSMSStorageStatus();
String getSetLocationFromFirebase(const String &deviceId);
bool parseCoordinates(const String &coordStr, float &lat, float &lon);
float calculateDistance(float lat1, float lon1, float lat2, float lon2);
void updateBuzzerInFirebase(const String &deviceId, bool buzzerActive);
void updateDismissAlert(const String &deviceId, bool dismissed);
void sendSetLocationToDevice(const String &phoneNumber, const String &deviceId);
void updateActiveDevice(const String &deviceId, const String &phoneNumber, bool buzzerActive, bool buzzerDismissed, bool solenoidActive);
void checkActiveDevicesForDismissal();

// =====================================================================
// SETUP
// ====================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("===== MASTER SMS RECEIVER - DEBUG TOOL =====");
  
  // Optional: Reset WiFi settings - uncomment to force WiFi reconfiguration
  // wifiManager.resetSettings();
  
  // Initialize WiFi connection
  connectToWiFi();
  
  // Initialize SIM7600
  sim7600.begin(SIM_BAUD, SERIAL_8N1, SIM_RX, SIM_TX);
  delay(3000);
  // Disable command echo for cleaner parsing
  sendAT("ATE0", 500);
  
  // Configure SIM7600 for SMS
  sendAT("AT", 1000);                 // Check connectivity
  sendAT("AT+CMGF=1", 1000);          // Set to Text Mode
  sendAT("AT+CPMS=\"SM\",\"SM\",\"SM\"", 1000); // Use SIM storage
  // Disable unsolicited new-message notifications; we'll poll instead
  sendAT("AT+CNMI=0,0,0,0,0", 1000);
  
  // Clean up SMS memory on startup
  cleanupSMSMemory();
  lastSMSCleanup = millis();
  
  Serial.println("\n✅ SIM7600 Initialized. Waiting for SMS messages...");
}

// =====================================================================
// MAIN LOOP
// =====================================================================
void loop() {
  readAllSMS();
  
  // Periodic SMS cleanup
  if (millis() - lastSMSCleanup > SMS_CLEANUP_INTERVAL) {
    cleanupSMSMemory();
    lastSMSCleanup = millis();
  }
  
  // Periodic Firebase check for active devices
  if (millis() - lastFirebaseCheck > FIREBASE_CHECK_INTERVAL) {
    checkActiveDevicesForDismissal();
    lastFirebaseCheck = millis();
  }
  
  delay(5000);  // Poll every 5 seconds
}

// =====================================================================
// SMS READING FUNCTIONS
// =====================================================================

// Poll for UNREAD SMS messages, then fetch each with CMGR
void readAllSMS() {
  sim7600.println("AT+CMGL=\"REC UNREAD\"");  // List UNREAD messages
  delay(1000);

  if (!sim7600.available()) return;

  String listResp = sim7600.readString();

  // Any unread messages?
  if (listResp.indexOf("+CMGL:") == -1) return;

  Serial.println("\n--- NEW SMS DETECTED ---");

  int searchPos = 0;
  while (true) {
    int tagPos = listResp.indexOf("+CMGL:", searchPos);
    if (tagPos == -1) break;

    // Locate the numeric index right after ':'
    int colon = listResp.indexOf(':', tagPos);
    if (colon == -1) break;

    int i = colon + 1;
    while (i < listResp.length() && listResp[i] == ' ') i++; // skip spaces
    int j = i;
    while (j < listResp.length() && isdigit(listResp[j])) j++;
    if (i == j) { searchPos = colon + 1; continue; }

    int msgIndex = listResp.substring(i, j).toInt();

    // Read and print the SMS content using CMGR
    readSMS(msgIndex);

    // Delete after reading to keep mailbox clean
    sendAT((String("AT+CMGD=") + msgIndex).c_str(), 1000);

    searchPos = j;
  }

  Serial.println("---------------------------------");

  // Update signal quality after processing messages
  showSignalQuality();
}

// Send an AT command and print the response
String sendAT(const char *cmd, int waitTime) {
  Serial.print(">> Sending: ");
  Serial.println(cmd);
  
  sim7600.println(cmd);
  delay(waitTime);
  
  String response = readModem();
  return response;
}

// Read and print response from the modem
String readModem() {
  String response = "";
  while (sim7600.available()) {
    response += (char)sim7600.read();
  }
  
  if (response.length() > 0) {
    Serial.print("<< Received: ");
    Serial.println(response);
  }
  return response;
} 

// =====================================================================
// ADDITIONAL UTILITY FUNCTIONS
// =====================================================================

// Translate AT+CSQ response into human-readable signal quality
void showSignalQuality() {
  String resp = sendAT("AT+CSQ", 500);
  int idx = resp.indexOf("+CSQ:");
  if (idx != -1) {
    int comma = resp.indexOf(',', idx);
    if (comma != -1) {
      String rssiStr = resp.substring(idx + 6, comma);
      int rssi = rssiStr.toInt();
      if (rssi == 99) {
        Serial.println("Signal Quality: Unknown");
      } else {
        int dBm = -113 + 2 * rssi;
        String quality;
        if (rssi <= 9) quality = "Poor";
        else if (rssi <= 14) quality = "Fair";
        else if (rssi <= 19) quality = "Good";
        else quality = "Excellent";
        Serial.print("Signal Quality: ");
        Serial.print(quality);
        Serial.print(" (");
        Serial.print(dBm);
        Serial.println(" dBm)");
      }
    }
  }
}

// Read a single SMS by index using AT+CMGR
void readSMS(int index) {
  String cmd = "AT+CMGR=" + String(index);
  String resp = sendAT(cmd.c_str(), 1000);

  // Extract phone number from CMGR response
  String phoneNumber = extractPhoneNumberFromCMGR(resp);
  if (phoneNumber.length() > 0) {
    Serial.println("SMS from: " + phoneNumber);
  }

  // Extract message body from CMGR response
  String smsBody = extractSMSContent(resp);
  if (smsBody.length() == 0) {
    Serial.println("❌ Failed to extract SMS body.");
    return;
  }

  Serial.println("SMS Raw Body: " + smsBody);
  
  // Extract device ID
  String tempDeviceId = smsBody.substring(0, smsBody.indexOf(','));
  tempDeviceId.replace("§", "_");
  tempDeviceId.replace("\xA7", "_");
  
  // Send setLocation to slave first
  Serial.println("\n📍 SENDING SETLOCATION TO SLAVE DEVICE...");
  sendSetLocationToDevice(phoneNumber, tempDeviceId);
  delay(3000); // Give time for slave to receive and process
  
  // Debug: Print each field
  Serial.println("\n=== PARSING SMS DATA ===");
  String debugTokens[16];
  int debugStart = 0;
  for (int i = 0; i < 16; i++) {
    int comma = smsBody.indexOf(',', debugStart);
    if (comma == -1) {
      debugTokens[i] = smsBody.substring(debugStart);
    } else {
      debugTokens[i] = smsBody.substring(debugStart, comma);
      debugStart = comma + 1;
    }
  }
  
  Serial.println("Device ID: " + debugTokens[0]);
  Serial.println("Timestamp: " + debugTokens[1]);
  Serial.println("Temperature: " + debugTokens[2] + "°C");
  Serial.println("Humidity: " + debugTokens[3] + "%");
  Serial.println("Latitude: " + debugTokens[4]);
  Serial.println("Longitude: " + debugTokens[5]);
  Serial.println("Altitude: " + debugTokens[6] + "m");
  Serial.println("Tilt detected: " + String(debugTokens[7] == "1" ? "YES" : "NO"));
  Serial.println("Fall detected: " + String(debugTokens[8] == "1" ? "YES" : "NO"));
  Serial.println("Limit switch: " + String(debugTokens[9] == "1" ? "PRESSED (lid closed)" : "NOT PRESSED (lid open)"));
  Serial.println("Solenoid: " + String(debugTokens[10] == "1" ? "ACTIVE" : "INACTIVE"));
  Serial.println("Accel X: " + debugTokens[11] + "g");
  Serial.println("Accel Y: " + debugTokens[12] + "g");
  Serial.println("Accel Z: " + debugTokens[13] + "g");
  Serial.println("Battery: " + debugTokens[14] + "V");
  Serial.println("Wake reason: " + debugTokens[15]);
  Serial.println("======================");

  // Convert CSV to JSON for Firebase
  String json = csvToFirebaseJson(smsBody);
  if (json.length() == 0) {
    Serial.println("❌ Failed to parse SMS CSV format.");
    return;
  }

  Serial.println("\n📦 Firebase JSON Payload:\n" + json + "\n");

  // Send to Firebase
  String deviceId = smsBody.substring(0, smsBody.indexOf(','));
  // Fix character encoding issue: replace section sign with underscore
  deviceId.replace("§", "_");
  deviceId.replace("\xA7", "_"); // Section sign in hex
  
  Serial.println("Device ID: " + deviceId);
  
  // Extract solenoid state from SMS (field 10, index 10 in CSV)
  String solenoidStateFromSMS = extractFieldFromCSV(smsBody, 10);
  
  // Send sensor data to Firebase
  sendJsonToFirebase(json, deviceId, phoneNumber);
  
  // Send solenoid state separately
  sendSolenoidToFirebase(deviceId, solenoidStateFromSMS == "1");
  
  // Get setLocation from Firebase for geofencing info
  String setLocation = getSetLocationFromFirebase(deviceId);
  Serial.println("Set location: " + setLocation);
  
  // GEOFENCING LOGIC - Determine if buzzer should be active
  bool shouldBuzzerBeActive = false;
  bool lidOpen = (debugTokens[9] == "0"); // Limit switch NOT pressed means lid is open
  float currentLat = debugTokens[4].toFloat();
  float currentLon = debugTokens[5].toFloat();
  
  if (lidOpen) {
    Serial.println("\n=== GEOFENCING CHECK ===");
    Serial.println("Lid is OPEN - checking location...");
    
    if (currentLat != 0.0 && currentLon != 0.0 && setLocation.length() > 0) {
      float setLat, setLon;
      if (parseCoordinates(setLocation, setLat, setLon)) {
        float distance = calculateDistance(currentLat, currentLon, setLat, setLon);
        Serial.println("Current location: " + String(currentLat, 6) + ", " + String(currentLon, 6));
        Serial.println("Set location: " + String(setLat, 6) + ", " + String(setLon, 6));
        Serial.println("Distance: " + String(distance, 1) + " meters");
        
        if (distance > 50.0) {
          shouldBuzzerBeActive = true;
          Serial.println("🚨 OUTSIDE SAFE ZONE - BUZZER SHOULD ACTIVATE!");
        } else {
          Serial.println("✅ Within safe zone - buzzer remains off");
        }
      }
    } else {
      // If GPS not ready or no setLocation, activate buzzer as safety measure
      if (currentLat == 0.0 || currentLon == 0.0) {
        Serial.println("⚠️ GPS not ready - activating buzzer as safety measure");
        shouldBuzzerBeActive = true;
      }
      if (setLocation.length() == 0) {
        Serial.println("⚠️ No setLocation defined - activating buzzer");
        shouldBuzzerBeActive = true;
      }
    }
  } else {
    Serial.println("Lid is CLOSED - buzzer not needed");
  }
  
  // Update buzzer state in Firebase if it needs to change
  if (shouldBuzzerBeActive) {
    updateBuzzerInFirebase(deviceId, true);
  }
  
  // Fetch and relay Firebase control states
  Serial.println("\n=== FETCHING FIREBASE CONTROL STATES ===");
  
  // 1. setLocation already fetched above, just send it to slave
  if (setLocation.length() > 0) {
    Serial.println("📍 SENDING SETLOCATION TO SLAVE: " + setLocation);
    sendSetLocationToDevice(phoneNumber, deviceId);
    delay(2000);
  }
  
  // 2. Get SOLENOID state
  String solenoidUrl = String(FIREBASE_HOST) + "/tracking_box/" + deviceId + "/solenoid.json?auth=" + FIREBASE_AUTH;
  DBG_FB("Fetching solenoid from: " + solenoidUrl);
  HTTPClient httpSolenoid;
  httpSolenoid.begin(solenoidUrl);
  int solenoidCode = httpSolenoid.GET();
  bool solenoidState = false;
  if (solenoidCode == 200) {
    String solenoidResp = httpSolenoid.getString();
    DBG_FB("Raw solenoid response from Firebase: " + solenoidResp);
    solenoidResp.replace("\"", "");
    solenoidResp.trim();
    solenoidState = (solenoidResp == "true");
    DBG_FB("Parsed solenoid state: " + String(solenoidState));
  } else {
    DBG_FB("Failed to get solenoid state, HTTP code: " + String(solenoidCode));
    if (solenoidCode == 404) {
      DBG_FB("Solenoid path not found - it may not exist in Firebase yet");
    }
  }
  httpSolenoid.end();
  Serial.println("🔐 SOLENOID: " + String(solenoidState ? "ACTIVATE" : "OFF"));
  
  // 3. Use calculated buzzer state (from geofencing logic above)
  bool buzzerState = shouldBuzzerBeActive;
  Serial.println("🔔 BUZZER: " + String(buzzerState ? "ACTIVATE" : "OFF"));
  
  // 4. Get BUZZER DISMISSED state
  String buzzerDismissedUrl = String(FIREBASE_HOST) + "/tracking_box/" + deviceId + "/sensorData/buzzerDismissed.json?auth=" + FIREBASE_AUTH;
  HTTPClient httpBuzzerDismissed;
  httpBuzzerDismissed.begin(buzzerDismissedUrl);
  int buzzerDismissedCode = httpBuzzerDismissed.GET();
  bool buzzerDismissedState = false;
  if (buzzerDismissedCode == 200) {
    String buzzerDismissedResp = httpBuzzerDismissed.getString();
    buzzerDismissedResp.replace("\"", "");
    buzzerDismissedState = (buzzerDismissedResp == "true");
  }
  httpBuzzerDismissed.end();
  Serial.println("🔕 BUZZER DISMISSED: " + String(buzzerDismissedState ? "YES" : "NO"));
  
  // If buzzer is dismissed, override the buzzer state to OFF
  if (buzzerDismissedState && buzzerState) {
    buzzerState = false;
    Serial.println("🔕 Buzzer overridden to OFF due to dismissal");
  }
  
  // 5. Send control command to slave
  String controlMsg = "CMD," + String(buzzerState ? "1" : "0") + "," + 
                      String(solenoidState ? "1" : "0") + "," +
                      String(buzzerDismissedState ? "1" : "0");
  
  Serial.println("\n📨 SENDING TO SLAVE: " + controlMsg);
  delay(2000);
  sendControlSMS(phoneNumber, controlMsg);
  
  // Always track devices that communicate with us
  updateActiveDevice(deviceId, phoneNumber, buzzerState, buzzerDismissedState, solenoidState);
}

// Extracts the text part (after first CRLF) from CMGR response
String extractSMSContent(const String &resp) {
  // Locate header line starting with +CMGR:
  int hdrPos = resp.indexOf("+CMGR:");
  if (hdrPos == -1) {
    return "";
  }

  // Find end of header (newline)
  int bodyStart = resp.indexOf('\n', hdrPos);
  if (bodyStart == -1) return "";
  bodyStart++; // move past newline

  // Skip potential CR
  if (bodyStart < resp.length() && resp[bodyStart] == '\r') bodyStart++;

  // Body ends at the next CR or LF before blank line / OK
  int bodyEnd = resp.indexOf('\r', bodyStart);
  if (bodyEnd == -1) bodyEnd = resp.indexOf('\n', bodyStart);
  if (bodyEnd == -1) bodyEnd = resp.length();

  String body = resp.substring(bodyStart, bodyEnd);
  body.trim();

  return body;
}

// Converts CSV string from TrackingBoxMain to Firebase-ready JSON
String csvToFirebaseJson(const String &csv) {
  const int FIELD_COUNT = 16;  // Updated to 16 fields
  String tokens[FIELD_COUNT];
  int start = 0;
  for (int i = 0; i < FIELD_COUNT; i++) {
    int comma = csv.indexOf(',', start);
    if (comma == -1) {
      tokens[i] = csv.substring(start);
      if (i < FIELD_COUNT - 1) {
        // not enough fields
        return "";
      }
    } else {
      tokens[i] = csv.substring(start, comma);
      start = comma + 1;
    }
    tokens[i].trim();
  }

  // Parse GPS coordinates
  String latStr = tokens[4];
  String lonStr = tokens[5];
  float latDec = latStr.toFloat();
  float lonDec = lonStr.toFloat();
  
  // Determine currentLocation based on GPS validity
  String currentLocation;
  bool gpsValid = (latDec != 0.0 && lonDec != 0.0);
  if (gpsValid) {
    currentLocation = String(latDec, 6) + ", " + String(lonDec, 6);
  } else {
    currentLocation = "GPS Initializing. Please Wait. . .";
  }

  // Build JSON following exact structure
  String json = "{";
  
  // Accelerometer object
  json += "\"accelerometer\":{";
  json += "\"x\":" + tokens[11] + ",";
  json += "\"y\":" + tokens[12] + ",";
  json += "\"z\":" + tokens[13];
  json += "},";
  
  // Battery voltage
  json += "\"batteryVoltage\":" + tokens[14] + ",";
  
  // Boot count (not in SMS, default to 1)
  json += "\"bootCount\":1,";
  
  // Don't set buzzer state here - it will be determined by geofencing logic
  // Set to false initially, will be updated after location check
  
  // Only include buzzerIsActive, NOT buzzerDismissed (that's in dismissAlert)
  json += "\"buzzerIsActive\":false,";
  
  // Coarse fix (assume true if no GPS)
  json += "\"coarseFix\":" + String(!gpsValid ? "true" : "false") + ",";
  
  // Current location
  json += "\"currentLocation\":\"" + currentLocation + "\",";
  
  // Fall detected
  json += "\"fallDetected\":" + String(tokens[8] == "1" ? "true" : "false") + ",";
  
  // GPS fix valid
  json += "\"gpsFixValid\":" + String(gpsValid ? "true" : "false") + ",";
  
  // Humidity
  json += "\"humidity\":" + tokens[3] + ",";
  
  // Limit switch
  json += "\"limitSwitchPressed\":" + String(tokens[9] == "1" ? "true" : "false") + ",";
  
  // Temperature
  json += "\"temp\":" + tokens[2] + ",";
  
  // Tilt detected
  json += "\"tiltDetected\":" + String(tokens[7] == "1" ? "true" : "false") + ",";
  
  // Timestamp
  json += "\"timestamp\":" + tokens[1] + ",";
  
  // Using CGPS (assume true for SMS mode)
  json += "\"usingCGPS\":true,";
  
  // Wake up reason
  json += "\"wakeUpReason\":\"" + tokens[15] + "\"";
  
  json += "}";
  return json;
} 

// ---------------------------------------------------------------------
// WIFI + FIREBASE SENDER
// ---------------------------------------------------------------------

bool connectToWiFi() {
  // Stage 1: Try quick connection with saved credentials first
  if (WiFi.SSID().length() > 0) {
    Serial.println("Attempting quick WiFi connection to saved network: " + WiFi.SSID());
    WiFi.mode(WIFI_STA);
    WiFi.begin(); // Uses saved SSID and password
    
    // Wait maximum 15 seconds for connection
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
      DBG_FB("✓ WiFi connected: " + WiFi.localIP().toString());
      return true;
    } else {
      Serial.println("✗ Quick connection failed - saved network not available");
    }
  }
  
  // Stage 2: Use WiFiManager for new network configuration or if no saved credentials
  Serial.println("Starting WiFiManager for network configuration...");
  
  // Set shorter timeouts for faster access point creation
  wifiManager.setConfigPortalTimeout(30); // 30 seconds for portal
  wifiManager.setConnectTimeout(15); // 15 seconds for connection attempts
  
  // Set custom IP for configuration portal
  wifiManager.setAPStaticIPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  
  // Try to connect or show configuration portal
  if (wifiManager.autoConnect("ProxyServer-Setup")) {
    DBG_FB("✓ WiFi connected via WiFiManager: " + WiFi.localIP().toString());
    return true;
  } else {
    DBG_FB("✗ WiFi connection failed or configuration portal timeout");
    return false;
  }
}

void sendJsonToFirebase(const String &json, const String &deviceId, const String &phoneNumber) {
  if (WiFi.status() != WL_CONNECTED) {
    if (!connectToWiFi()) {
      DBG_FB("✗ Cannot send to Firebase – WiFi unavailable");
      return;
    }
  }

  // Send sensorData
  String url = String(FIREBASE_HOST) + "/tracking_box/" + deviceId + "/sensorData.json?auth=" + FIREBASE_AUTH;
  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  DBG_FB("Uploading sensorData to: " + url);

  int code = http.PUT(json);
  if (code > 0 && code < 400) {
    DBG_FB("✓ Firebase sensorData PUT success, code " + String(code));
  } else {
    DBG_FB("✗ Firebase sensorData PUT failed, code " + String(code));
  }
  http.end();
}

// ---------------------------------------------------------------------
// HELPER FUNCTIONS
// ---------------------------------------------------------------------

// Extract phone number from CMGR response
String extractPhoneNumberFromCMGR(const String &resp) {
  // CMGR response format: +CMGR: "REC UNREAD","+639184652918","","24/01/23,10:30:45+32"
  int cmgrPos = resp.indexOf("+CMGR:");
  if (cmgrPos == -1) return "";
  
  int firstQuote = resp.indexOf('"', cmgrPos);
  if (firstQuote == -1) return "";
  
  int secondQuote = resp.indexOf('"', firstQuote + 1);
  if (secondQuote == -1) return "";
  
  int thirdQuote = resp.indexOf('"', secondQuote + 1);
  if (thirdQuote == -1) return "";
  
  int fourthQuote = resp.indexOf('"', thirdQuote + 1);
  if (fourthQuote == -1) return "";
  
  return resp.substring(thirdQuote + 1, fourthQuote);
}


// Send control SMS to slave device
void sendControlSMS(const String &phoneNumber, const String &message) {
  DBG_FB("Sending control SMS to: " + phoneNumber);
  DBG_FB("Control message: " + message);
  
  // Set SMS mode
  sendAT("AT+CMGF=1", 500);
  
  // Set character set
  sendAT("AT+CSCS=\"GSM\"", 500);
  
  // Set recipient
  sim7600.print("AT+CMGS=\"");
  sim7600.print(phoneNumber);
  sim7600.println("\"");
  delay(1000);
  
  // Wait for prompt
  unsigned long timeout = millis() + 10000;
  bool promptFound = false;
  while (millis() < timeout) {
    if (sim7600.available()) {
      char c = sim7600.read();
      if (c == '>') {
        promptFound = true;
        break;
      }
    }
  }
  
  if (!promptFound) {
    DBG_FB("❌ Failed to get SMS prompt");
    return;
  }
  
  // Send message content
  sim7600.print(message);
  delay(500);
  
  // Send Ctrl+Z to finish SMS
  sim7600.write(26);
  delay(5000);
  
  // Check response
  String response = readModem();
  if (response.indexOf("OK") != -1 || response.indexOf("+CMGS:") != -1) {
    DBG_FB("✓ Control SMS sent successfully");
    DBG_FB("✓ Message content: " + message);
  } else {
    DBG_FB("❌ Failed to send control SMS");
    DBG_FB("Response: " + response);
  }
}

// ---------------------------------------------------------------------
// HELPER FUNCTIONS
// ---------------------------------------------------------------------

// Extract a specific field from CSV string
String extractFieldFromCSV(const String &csv, int fieldIndex) {
  int start = 0;
  int currentField = 0;
  
  while (currentField < fieldIndex) {
    int comma = csv.indexOf(',', start);
    if (comma == -1) {
      return ""; // Field not found
    }
    start = comma + 1;
    currentField++;
  }
  
  int nextComma = csv.indexOf(',', start);
  if (nextComma == -1) {
    return csv.substring(start);
  } else {
    return csv.substring(start, nextComma);
  }
}

// Send solenoid state to Firebase (separate from sensorData)
void sendSolenoidToFirebase(const String &deviceId, bool state) {
  if (WiFi.status() != WL_CONNECTED) {
    DBG_FB("✗ Cannot send solenoid state – WiFi unavailable");
    return;
  }
  
  String url = String(FIREBASE_HOST) + "/tracking_box/" + deviceId + "/solenoid.json?auth=" + FIREBASE_AUTH;
  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  
  String payload = state ? "true" : "false";
  DBG_FB("Sending solenoid state: " + payload + " to: " + url);
  
  int code = http.PUT(payload);
  if (code > 0 && code < 400) {
    DBG_FB("✓ Firebase solenoid PUT success, code " + String(code));
  } else {
    DBG_FB("✗ Firebase solenoid PUT failed, code " + String(code));
  }
  http.end();
}

// ---------------------------------------------------------------------
// SMS MEMORY CLEANUP
// ---------------------------------------------------------------------

// Clean up SMS memory by deleting all read messages
void cleanupSMSMemory() {
  Serial.println("\n🧹 Cleaning up SMS memory...");
  
  // Delete all messages (read, unread, sent, unsent)
  // AT+CMGD=1,4 deletes all messages
  String response = sendAT("AT+CMGD=1,4", 5000);
  
  if (response.indexOf("OK") != -1) {
    Serial.println("✅ SMS memory cleaned successfully");
  } else {
    Serial.println("❌ Failed to clean SMS memory");
    
    // Alternative: Try deleting by type
    Serial.println("Trying alternative deletion method...");
    
    // Delete all read messages
    sendAT("AT+CMGD=1,1", 2000);
    
    // Delete all sent messages  
    sendAT("AT+CMGD=1,2", 2000);
    
    // Delete all unsent messages
    sendAT("AT+CMGD=1,3", 2000);
  }
  
  // Show SMS storage status
  showSMSStorageStatus();
}

// Show SMS storage status
void showSMSStorageStatus() {
  String response = sendAT("AT+CPMS?", 1000);
  Serial.println("SMS Storage Status: " + response);
}

// Get setLocation from Firebase for a specific device
String getSetLocationFromFirebase(const String &deviceId) {
  if (WiFi.status() != WL_CONNECTED) {
    DBG_FB("✗ Cannot get setLocation – WiFi unavailable");
    return "";
  }
  
  String url = String(FIREBASE_HOST) + "/tracking_box/" + deviceId + "/details/setLocation.json?auth=" + FIREBASE_AUTH;
  HTTPClient http;
  http.begin(url);
  int code = http.GET();
  String setLocation = "";
  
  if (code == 200) {
    setLocation = http.getString();
    DBG_FB("Raw setLocation from Firebase: " + setLocation);
    setLocation.replace("\"", "");
    setLocation.trim();
    DBG_FB("Cleaned setLocation: " + setLocation);
  } else {
    DBG_FB("Failed to get setLocation, HTTP code: " + String(code));
  }
  http.end();
  
  return setLocation;
}

// Parse coordinate string (supports decimal format "lat,lon")
bool parseCoordinates(const String &coordStr, float &lat, float &lon) {
  int commaIndex = coordStr.indexOf(',');
  if (commaIndex == -1) return false;
  
  String latStr = coordStr.substring(0, commaIndex);
  String lonStr = coordStr.substring(commaIndex + 1);
  
  latStr.trim();
  lonStr.trim();
  
  lat = latStr.toFloat();
  lon = lonStr.toFloat();
  
  return (lat != 0.0 && lon != 0.0);
}

// Calculate distance between two GPS coordinates in meters using Haversine formula
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000.0; // Earth radius in meters
  float dLat = (lat2 - lat1) * DEG_TO_RAD;
  float dLon = (lon2 - lon1) * DEG_TO_RAD;
  
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
            sin(dLon / 2) * sin(dLon / 2);
            
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

// Update buzzer state in Firebase
void updateBuzzerInFirebase(const String &deviceId, bool buzzerActive) {
  if (WiFi.status() != WL_CONNECTED) {
    DBG_FB("✗ Cannot update buzzer state – WiFi unavailable");
    return;
  }
  
  String url = String(FIREBASE_HOST) + "/tracking_box/" + deviceId + "/sensorData/buzzerIsActive.json?auth=" + FIREBASE_AUTH;
  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  
  String payload = buzzerActive ? "true" : "false";
  DBG_FB("Updating buzzer state to: " + payload);
  
  int code = http.PUT(payload);
  if (code > 0 && code < 400) {
    DBG_FB("✓ Firebase buzzer state updated, code " + String(code));
  } else {
    DBG_FB("✗ Firebase buzzer state update failed, code " + String(code));
  }
  http.end();
}


// Update dismissAlert in Firebase
void updateDismissAlert(const String &deviceId, bool dismissed) {
  if (WiFi.status() != WL_CONNECTED) {
    DBG_FB("✗ Cannot update dismissAlert – WiFi unavailable");
    return;
  }
  
  // Create dismissAlert object with dismissed flag and timestamp
  String url = String(FIREBASE_HOST) + "/tracking_box/" + deviceId + "/dismissAlert.json?auth=" + FIREBASE_AUTH;
  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  
  // Create JSON with dismissed flag and timestamp
  String json = "{";
  json += "\"dismissed\":" + String(dismissed ? "true" : "false") + ",";
  json += "\"timestamp\":" + String(millis());
  json += "}";
  
  DBG_FB("Updating dismissAlert: " + json);
  
  int code = http.PUT(json);
  if (code > 0 && code < 400) {
    DBG_FB("✓ Firebase dismissAlert updated, code " + String(code));
  } else {
    DBG_FB("✗ Firebase dismissAlert update failed, code " + String(code));
  }
  http.end();
}



// Send setLocation to device via SMS
void sendSetLocationToDevice(const String &phoneNumber, const String &deviceId) {
  // Get setLocation from Firebase
  String setLocation = getSetLocationFromFirebase(deviceId);
  
  if (setLocation.length() == 0) {
    DBG_FB("No setLocation found for " + deviceId);
    return;
  }
  
  // Format message: SETLOC,deviceId,lat,lon
  float lat, lon;
  if (!parseCoordinates(setLocation, lat, lon)) {
    DBG_FB("Failed to parse setLocation coordinates");
    return;
  }
  
  String message = "SETLOC," + deviceId + "," + String(lat, 6) + "," + String(lon, 6);
  
  DBG_FB("Sending setLocation SMS to " + phoneNumber + ": " + message);
  
  // Send SMS
  sendAT("AT+CMGF=1", 500);
  sendAT("AT+CSCS=\"GSM\"", 500);
  
  sim7600.print("AT+CMGS=\"");
  sim7600.print(phoneNumber);
  sim7600.println("\"");
  delay(1000);
  
  // Wait for prompt
  unsigned long timeout = millis() + 10000;
  bool promptFound = false;
  while (millis() < timeout) {
    if (sim7600.available()) {
      char c = sim7600.read();
      if (c == '>') {
        promptFound = true;
        break;
      }
    }
  }
  
  if (!promptFound) {
    DBG_FB("❌ Failed to get SMS prompt for setLocation");
    return;
  }
  
  // Send message content
  sim7600.print(message);
  delay(500);
  
  // Send Ctrl+Z
  sim7600.write(26);
  delay(3000);
  
  // Check response
  String response = readModem();
  if (response.indexOf("OK") != -1 || response.indexOf("+CMGS:") != -1) {
    DBG_FB("✓ SetLocation SMS sent successfully");
  } else {
    DBG_FB("❌ Failed to send setLocation SMS");
  }
}

// Update or add active device to tracking list
void updateActiveDevice(const String &deviceId, const String &phoneNumber, bool buzzerActive, bool buzzerDismissed, bool solenoidActive) {
  // Find existing device or add new one
  int index = -1;
  for (int i = 0; i < activeDeviceCount; i++) {
    if (activeDevices[i].deviceId == deviceId) {
      index = i;
      break;
    }
  }
  
  if (index == -1 && activeDeviceCount < MAX_DEVICES) {
    // Add new device
    index = activeDeviceCount++;
  }
  
  if (index != -1) {
    activeDevices[index].deviceId = deviceId;
    activeDevices[index].phoneNumber = phoneNumber;
    activeDevices[index].buzzerActive = buzzerActive;
    activeDevices[index].buzzerDismissed = buzzerDismissed;
    activeDevices[index].solenoidActive = solenoidActive;
    activeDevices[index].lastChecked = millis();
    DBG_FB("Updated active device: " + deviceId + " (buzzer=" + String(buzzerActive) + ", dismissed=" + String(buzzerDismissed) + ", solenoid=" + String(solenoidActive) + ")");
  }
}

// Check all active devices for control state changes (buzzerDismissed and solenoid)
void checkActiveDevicesForDismissal() {
  if (activeDeviceCount == 0) return;
  
  if (WiFi.status() != WL_CONNECTED) {
    if (!connectToWiFi()) {
      return;
    }
  }
  
  DBG_FB("\n🔍 Checking " + String(activeDeviceCount) + " active devices for control state changes...");
  
  for (int i = 0; i < activeDeviceCount; i++) {
    String deviceId = activeDevices[i].deviceId;
    DBG_FB("Checking device: " + deviceId);
    bool needsUpdate = false;
    bool currentBuzzerState = activeDevices[i].buzzerActive;
    bool currentBuzzerDismissed = activeDevices[i].buzzerDismissed;
    bool currentSolenoidState = activeDevices[i].solenoidActive;
    
    // 1. Check buzzerDismissed state from Firebase (only if buzzer is active)
    if (activeDevices[i].buzzerActive) {
      String buzzerDismissedUrl = String(FIREBASE_HOST) + "/tracking_box/" + deviceId + "/sensorData/buzzerDismissed.json?auth=" + FIREBASE_AUTH;
      HTTPClient httpBuzzerDismissed;
      httpBuzzerDismissed.begin(buzzerDismissedUrl);
      int buzzerDismissedCode = httpBuzzerDismissed.GET();
      bool newDismissedState = false;
      
      if (buzzerDismissedCode == 200) {
        String buzzerDismissedResp = httpBuzzerDismissed.getString();
        buzzerDismissedResp.replace("\"", "");
        newDismissedState = (buzzerDismissedResp == "true");
      }
      httpBuzzerDismissed.end();
      
      // Check if dismissed state has changed
      if (newDismissedState != currentBuzzerDismissed) {
        DBG_FB("📱 Device " + deviceId + " dismissal state changed: " + 
               String(currentBuzzerDismissed) + " → " + String(newDismissedState));
        currentBuzzerDismissed = newDismissedState;
        needsUpdate = true;
        
        // If now dismissed, turn off buzzer
        if (newDismissedState) {
          currentBuzzerState = false;
          updateBuzzerInFirebase(deviceId, false);
        }
      }
    }
    
    // 2. Check solenoid state from Firebase
    String solenoidUrl = String(FIREBASE_HOST) + "/tracking_box/" + deviceId + "/solenoid.json?auth=" + FIREBASE_AUTH;
    HTTPClient httpSolenoid;
    httpSolenoid.begin(solenoidUrl);
    int solenoidCode = httpSolenoid.GET();
    bool newSolenoidState = false;
    
    if (solenoidCode == 200) {
      String solenoidResp = httpSolenoid.getString();
      DBG_FB("Checking solenoid for " + deviceId + ": " + solenoidResp);
      solenoidResp.replace("\"", "");
      solenoidResp.trim();
      newSolenoidState = (solenoidResp == "true");
      DBG_FB("Parsed solenoid state: " + String(newSolenoidState) + " (was: " + String(currentSolenoidState) + ")");
    } else {
      DBG_FB("Failed to get solenoid state for " + deviceId + ", HTTP code: " + String(solenoidCode));
    }
    httpSolenoid.end();
    
    // Check if solenoid state has changed
    if (newSolenoidState != currentSolenoidState) {
      DBG_FB("🔐 Device " + deviceId + " solenoid state changed: " + 
             String(currentSolenoidState) + " → " + String(newSolenoidState));
      currentSolenoidState = newSolenoidState;
      needsUpdate = true;
    }
    
    // 3. Send updated control command if any state changed
    if (needsUpdate) {
      String controlMsg = "CMD," + String(currentBuzzerState ? "1" : "0") + "," + 
                          String(currentSolenoidState ? "1" : "0") + "," +
                          String(currentBuzzerDismissed ? "1" : "0");
      DBG_FB("📨 Sending updated control to " + activeDevices[i].phoneNumber + ": " + controlMsg);
      sendControlSMS(activeDevices[i].phoneNumber, controlMsg);
      
      // Update our tracking
      activeDevices[i].buzzerActive = currentBuzzerState;
      activeDevices[i].buzzerDismissed = currentBuzzerDismissed;
      activeDevices[i].solenoidActive = currentSolenoidState;
    }
    
    activeDevices[i].lastChecked = millis();
  }
  
  // Remove inactive devices (haven't been updated in 5 minutes)
  unsigned long timeout = 5 * 60 * 1000; // 5 minutes
  for (int i = activeDeviceCount - 1; i >= 0; i--) {
    if (millis() - activeDevices[i].lastChecked > timeout) {
      DBG_FB("Removing inactive device: " + activeDevices[i].deviceId);
      // Shift remaining devices
      for (int j = i; j < activeDeviceCount - 1; j++) {
        activeDevices[j] = activeDevices[j + 1];
      }
      activeDeviceCount--;
    }
  }
}