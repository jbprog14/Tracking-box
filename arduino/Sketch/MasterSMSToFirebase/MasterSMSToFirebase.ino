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
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>

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
const unsigned long SMS_CLEANUP_INTERVAL = 5 * 60 * 1000; // 5 minutes in milliseconds

// ---------------------------------------------------------------------
// FIREBASE & WIFI CONFIGURATION (copied from TrackingBoxMain)
// ---------------------------------------------------------------------
const char* WIFI_SSID = "archer_2.4G";
const char* WIFI_PASSWORD = "05132000";
const char* FIREBASE_HOST = "https://tracking-box-e17a1-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* FIREBASE_AUTH = "AIzaSyBQje281bPAt7MiJdK94ru1irAU8i3luzY";

// ---------------------------------------------------------------------
// MATH CONSTANTS
// ---------------------------------------------------------------------
#define DEG_TO_RAD 0.017453292519943295

// ---------------------------------------------------------------------
// DEBUG HELPER FOR FIREBASE ONLY
// ---------------------------------------------------------------------
const bool DEBUG_FB = true;
inline void DBG_FB(const String &msg) { if (DEBUG_FB) Serial.println(msg); }

// No longer needed; we'll poll for unread messages directly
// via AT+CMGL="REC UNREAD" and read them one-by-one with CMGR

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
void sendSolenoidToFirebase(const String &deviceId, bool state);
void cleanupSMSMemory();
void updateDismissAlert(const String &deviceId, bool dismissed);
void notifyPackageDelivery(const String &deviceId, float lat, float lon);
String getSetLocationFromFirebase(const String &deviceId);
bool parseCoordinates(const String &coordStr, float &lat, float &lon);
float calculateDistance(float lat1, float lon1, float lat2, float lon2);
void updateBuzzerInFirebase(const String &deviceId, bool buzzerActive);

// =====================================================================
// SETUP
// =================================S====================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("===== MASTER SMS RECEIVER - DEBUG MODE =====");
  
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
  Serial.println("Raw deviceId: " + deviceId);
  
  // Print each character's ASCII value for debugging
  Serial.print("Character codes: ");
  for (int i = 0; i < deviceId.length(); i++) {
    Serial.print((int)deviceId[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  // Fix character encoding issue: replace section sign with underscore
  deviceId.replace("§", "_");
  deviceId.replace("\xA7", "_"); // Section sign in hex
  
  // Also handle other potential encoding issues
  for (int i = 0; i < deviceId.length(); i++) {
    // Replace any non-printable characters (except underscore) with underscore
    if (deviceId[i] < 32 || deviceId[i] > 126) {
      if (deviceId[i] != '_') {
        deviceId[i] = '_';
      }
    }
  }
  
  Serial.println("Fixed deviceId: " + deviceId);
  
  // Extract solenoid state from SMS (field 10, index 10 in CSV)
  String solenoidState = extractFieldFromCSV(smsBody, 10);
  
  // Send sensor data to Firebase
  sendJsonToFirebase(json, deviceId, phoneNumber);
  
  // Send solenoid state separately
  sendSolenoidToFirebase(deviceId, solenoidState == "1");
  
  // If buzzer is active (lock breach detected), reset dismissAlert
  bool buzzerActive = (debugTokens[9] == "0") && (debugTokens[15].indexOf("LOCK BREACH") != -1);
  if (buzzerActive) {
    updateDismissAlert(deviceId, false);
  }
  
  // Check for package delivery - need to evaluate if within safe zone
  bool lidOpen = (debugTokens[9] == "0");
  float lat = debugTokens[4].toFloat();
  float lon = debugTokens[5].toFloat();
  
  // Get setLocation from Firebase to check distance
  String setLocation = getSetLocationFromFirebase(deviceId);
  bool withinSafeZone = false;
  
  if (lat != 0.0 && lon != 0.0 && setLocation.length() > 0) {
    // Parse setLocation and calculate distance
    float setLat, setLon;
    if (parseCoordinates(setLocation, setLat, setLon)) {
      float distance = calculateDistance(lat, lon, setLat, setLon);
      withinSafeZone = (distance <= 50.0); // 50 meters threshold
      Serial.println("Distance to safe zone: " + String(distance) + "m, Within safe zone: " + String(withinSafeZone ? "YES" : "NO"));
    }
  }
  
  // Determine actual buzzer state based on location
  bool actualBuzzerState = false;
  if (lidOpen && !withinSafeZone) {
    // Lock breach - lid open outside safe zone
    actualBuzzerState = true;
    Serial.println("🚨 Lock breach detected - lid open outside safe zone");
  } else if (lidOpen && withinSafeZone) {
    // Package delivered - lid open within safe zone
    actualBuzzerState = false;
    Serial.println("📦 Package delivered - lid open within safe zone");
    notifyPackageDelivery(deviceId, lat, lon);
  }
  
  // Update Firebase with correct buzzer state
  updateBuzzerInFirebase(deviceId, actualBuzzerState);
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
  
  // Don't determine buzzer state here - it will be set based on location check
  // Just report current buzzer state from the wake reason for now
  bool isLockBreach = (tokens[15].indexOf("LOCK BREACH") != -1);
  
  // Only include buzzerIsActive, NOT buzzerDismissed (that's in dismissAlert)
  json += "\"buzzerIsActive\":" + String(isLockBreach ? "true" : "false") + ",";
  
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
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  DBG_FB("Connecting to WiFi …");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(300);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    DBG_FB("✓ WiFi connected: " + WiFi.localIP().toString());
    return true;
  }
  DBG_FB("✗ WiFi connection failed");
  return false;
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
  
  // After sending data to Firebase, check control states and send back to slave
  checkAndSendControlStates(deviceId, phoneNumber);
}

// ---------------------------------------------------------------------
// CONTROL STATE MANAGEMENT
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

// Check Firebase for control flags and send SMS back to slave device
void checkAndSendControlStates(const String &deviceId, const String &phoneNumber) {
  if (WiFi.status() != WL_CONNECTED) {
    DBG_FB("✗ Cannot check control states – WiFi unavailable");
    return;
  }

  // Get buzzer state that was just calculated and stored
  String buzzerUrl = String(FIREBASE_HOST) + "/tracking_box/" + deviceId + "/sensorData/buzzerIsActive.json?auth=" + FIREBASE_AUTH;
  HTTPClient httpBuzzer;
  httpBuzzer.begin(buzzerUrl);
  int buzzerCode = httpBuzzer.GET();
  String buzzerState = "0";
  if (buzzerCode == 200) {
    buzzerState = httpBuzzer.getString();
    DBG_FB("Raw buzzer state from Firebase: " + buzzerState);
    buzzerState.replace("\"", "");
    buzzerState.replace("true", "1");
    buzzerState.replace("false", "0");
    buzzerState.trim();
  } else {
    DBG_FB("Failed to get buzzer state, HTTP code: " + String(buzzerCode));
  }
  httpBuzzer.end();

  // Get solenoid state
  String solenoidUrl = String(FIREBASE_HOST) + "/tracking_box/" + deviceId + "/solenoid.json?auth=" + FIREBASE_AUTH;
  HTTPClient httpSolenoid;
  httpSolenoid.begin(solenoidUrl);
  int solenoidCode = httpSolenoid.GET();
  String solenoidState = "0";
  if (solenoidCode == 200) {
    solenoidState = httpSolenoid.getString();
    DBG_FB("Raw solenoid state from Firebase: " + solenoidState);
    solenoidState.replace("\"", "");
    solenoidState.replace("true", "1");
    solenoidState.replace("false", "0");
    solenoidState.trim();
  } else {
    DBG_FB("Failed to get solenoid state, HTTP code: " + String(solenoidCode));
  }
  httpSolenoid.end();

  // Get dismiss state from dismissAlert node
  String dismissUrl = String(FIREBASE_HOST) + "/tracking_box/" + deviceId + "/dismissAlert/dismissed.json?auth=" + FIREBASE_AUTH;
  HTTPClient httpDismiss;
  httpDismiss.begin(dismissUrl);
  int dismissCode = httpDismiss.GET();
  String dismissState = "0";
  if (dismissCode == 200) {
    dismissState = httpDismiss.getString();
    DBG_FB("Raw dismiss state from Firebase dismissAlert: " + dismissState);
    dismissState.replace("\"", "");
    dismissState.replace("true", "1");
    dismissState.replace("false", "0");
    dismissState.trim();
  } else {
    DBG_FB("Failed to get dismiss state, HTTP code: " + String(dismissCode));
    // If dismissAlert doesn't exist, assume not dismissed
    dismissState = "0";
  }
  httpDismiss.end();

  // Format control message: CMD,buzzer,solenoid,dismiss
  String controlMsg = "CMD," + buzzerState + "," + solenoidState + "," + dismissState;
  
  DBG_FB("Control states from Firebase: " + controlMsg);
  
  // Send control SMS back to the slave device that sent the sensor data
  if (phoneNumber.length() > 0) {
    sendControlSMS(phoneNumber, controlMsg);
  } else {
    DBG_FB("❌ No phone number available to send control SMS");
  }
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
  } else {
    DBG_FB("❌ Failed to send control SMS");
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

// Notify package delivery to Firebase
void notifyPackageDelivery(const String &deviceId, float lat, float lon) {
  if (WiFi.status() != WL_CONNECTED) {
    DBG_FB("✗ Cannot notify package delivery – WiFi unavailable");
    return;
  }
  
  // Create delivery notification in Firebase - same path as website expects
  String url = String(FIREBASE_HOST) + "/tracking_box/" + deviceId + "/delivery.json?auth=" + FIREBASE_AUTH;
  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  
  // Create JSON with delivery info matching website format
  String json = "{";
  json += "\"delivered\":true,";
  json += "\"awaitingSolenoid\":true,";
  json += "\"deliveryLocation\":\"" + String(lat, 6) + ", " + String(lon, 6) + "\",";
  json += "\"deliveryTime\":" + String(millis()) + ",";
  json += "\"message\":\"Package delivered successfully. Security lock awaiting activation.\"";
  json += "}";
  
  DBG_FB("Package delivery notification: " + json);
  
  int code = http.PUT(json);
  if (code > 0 && code < 400) {
    DBG_FB("✓ Package delivery notification sent, code " + String(code));
  } else {
    DBG_FB("✗ Package delivery notification failed, code " + String(code));
  }
  http.end();
} 