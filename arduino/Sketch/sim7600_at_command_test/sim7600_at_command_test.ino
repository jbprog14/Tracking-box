/*
 * SIM7600G-H AT Command Test Suite
 * Based on working configuration: UART2, RX=18, TX=19 @ 115200
 * 
 * This sketch tests all essential AT commands for:
 * - Basic communication
 * - Network connectivity
 * - HTTP/HTTPS operations
 * - GPS/GNSS functionality
 * - Firebase readiness
 */

#include <HardwareSerial.h>
#include <ArduinoJson.h>

// Working configuration from diagnostic
#define SIM7600_RX_PIN 18
#define SIM7600_TX_PIN 19
#define SIM7600_BAUD 115200

// Firebase configuration for testing
const String FIREBASE_HOST = "https://wetick-762c7-default-rtdb.asia-southeast1.firebasedatabase.app";
const String DEVICE_ID = "box_test";

HardwareSerial sim7600(2);  // UART2

// Test categories
enum TestCategory {
  BASIC_COMM,
  SIM_STATUS,
  NETWORK_REG,
  DATA_CONN,
  HTTP_TEST,
  HTTPS_TEST,
  GPS_TEST,
  FIREBASE_TEST
};

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(2000);
  
  Serial.println("\n================================================");
  Serial.println("       SIM7600G-H AT COMMAND TEST SUITE");
  Serial.println("================================================");
  Serial.println("Configuration: UART2, RX=GPIO18, TX=GPIO19");
  Serial.println("Baud Rate: 115200");
  Serial.println("Module: SIMCOM_SIM7600G-H");
  Serial.println("================================================\n");
  
  // Initialize SIM7600 with working configuration
  sim7600.begin(SIM7600_BAUD, SERIAL_8N1, SIM7600_RX_PIN, SIM7600_TX_PIN);
  sim7600.setRxBufferSize(2048);
  delay(1000);
  
  // Clear any pending data
  flushBuffer();
  
  // Run comprehensive tests
  runAllTests();
  
  Serial.println("\n================================================");
  Serial.println("         ENTERING INTERACTIVE MODE");
  Serial.println("================================================");
  Serial.println("Type AT commands to send to SIM7600G");
  Serial.println("Special commands:");
  Serial.println("  'test' - Run all tests again");
  Serial.println("  'http' - Test HTTP GET request");
  Serial.println("  'https' - Test HTTPS to Firebase");
  Serial.println("  'gps' - Get GPS location");
  Serial.println("================================================\n");
}

void loop() {
  // Interactive AT command mode
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.equals("test")) {
      runAllTests();
    } else if (cmd.equals("http")) {
      testHTTPGet();
    } else if (cmd.equals("https")) {
      testFirebaseConnection();
    } else if (cmd.equals("gps")) {
      testGPS();
    } else if (cmd.length() > 0) {
      Serial.println(">>> " + cmd);
      sim7600.println(cmd);
      String response = readResponse(5000);
      Serial.print(response);
    }
  }
  
  // Echo any unsolicited messages from SIM7600
  if (sim7600.available()) {
    Serial.print(sim7600.readString());
  }
}

void runAllTests() {
  Serial.println("Starting comprehensive AT command tests...\n");
  
  // 1. Basic Communication
  testCategory(BASIC_COMM);
  
  // 2. SIM Card Status
  testCategory(SIM_STATUS);
  
  // 3. Network Registration
  testCategory(NETWORK_REG);
  
  // 4. Data Connection
  testCategory(DATA_CONN);
  
  // 5. HTTP Test
  testCategory(HTTP_TEST);
  
  // 6. HTTPS Test
  testCategory(HTTPS_TEST);
  
  // 7. GPS Test
  testCategory(GPS_TEST);
  
  // 8. Firebase Test
  testCategory(FIREBASE_TEST);
  
  Serial.println("\n✅ All tests completed!");
}

void testCategory(TestCategory category) {
  switch(category) {
    case BASIC_COMM:
      Serial.println("1. BASIC COMMUNICATION");
      Serial.println("----------------------");
      sendATCommand("AT", "Basic AT");
      sendATCommand("ATE1", "Echo ON");
      sendATCommand("AT+CGMM", "Model Info");
      sendATCommand("AT+CGMR", "Firmware Version");
      sendATCommand("AT+CGSN", "IMEI Number");
      Serial.println();
      break;
      
    case SIM_STATUS:
      Serial.println("2. SIM CARD STATUS");
      Serial.println("------------------");
      sendATCommand("AT+CPIN?", "SIM Status");
      sendATCommand("AT+CCID", "SIM Card ID");
      sendATCommand("AT+CNUM", "Phone Number");
      Serial.println();
      break;
      
    case NETWORK_REG:
      Serial.println("3. NETWORK REGISTRATION");
      Serial.println("-----------------------");
      sendATCommand("AT+CREG?", "Network Registration");
      sendATCommand("AT+COPS?", "Network Operator");
      sendATCommand("AT+CSQ", "Signal Quality");
      sendATCommand("AT+CPSI?", "System Info");
      Serial.println();
      break;
      
    case DATA_CONN:
      Serial.println("4. DATA CONNECTION");
      Serial.println("------------------");
      sendATCommand("AT+CGATT?", "GPRS Attachment");
      sendATCommand("AT+CGACT?", "PDP Context");
      sendATCommand("AT+CGPADDR", "IP Address");
      sendATCommand("AT+CGDCONT?", "APN Settings");
      Serial.println();
      break;
      
    case HTTP_TEST:
      Serial.println("5. HTTP CAPABILITY");
      Serial.println("------------------");
      testHTTPGet();
      Serial.println();
      break;
      
    case HTTPS_TEST:
      Serial.println("6. HTTPS CAPABILITY");
      Serial.println("-------------------");
      testHTTPS();
      Serial.println();
      break;
      
    case GPS_TEST:
      Serial.println("7. GPS/GNSS TEST");
      Serial.println("----------------");
      testGPS();
      Serial.println();
      break;
      
    case FIREBASE_TEST:
      Serial.println("8. FIREBASE CONNECTION TEST");
      Serial.println("---------------------------");
      testFirebaseConnection();
      Serial.println();
      break;
  }
}

void sendATCommand(String cmd, String description) {
  Serial.print("  " + description + ": ");
  sim7600.println(cmd);
  String response = readResponse(2000);
  
  if (response.indexOf("OK") >= 0) {
    // Extract useful info from response
    int start = response.indexOf(":");
    if (start > 0) {
      int end = response.indexOf("\r", start);
      if (end > start) {
        String info = response.substring(start + 1, end);
        info.trim();
        Serial.println("✓ " + info);
      } else {
        Serial.println("✓");
      }
    } else {
      Serial.println("✓");
    }
  } else if (response.indexOf("ERROR") >= 0) {
    Serial.println("✗ ERROR");
  } else {
    Serial.println("✗ No response");
  }
}

void testHTTPGet() {
  Serial.println("  Testing HTTP GET...");
  
  // Terminate any existing HTTP session
  sim7600.println("AT+HTTPTERM");
  delay(500);
  flushBuffer();
  
  // Initialize HTTP
  sim7600.println("AT+HTTPINIT");
  String response = readResponse(3000);
  if (response.indexOf("OK") < 0) {
    Serial.println("    ✗ Failed to initialize HTTP");
    return;
  }
  
  // Set URL (using a simple test endpoint)
  sim7600.println("AT+HTTPPARA=\"URL\",\"http://httpbin.org/get\"");
  response = readResponse(2000);
  if (response.indexOf("OK") < 0) {
    Serial.println("    ✗ Failed to set URL");
    sim7600.println("AT+HTTPTERM");
    return;
  }
  
  // Perform GET request
  sim7600.println("AT+HTTPACTION=0");
  response = readResponse(10000);
  
  // Check status
  if (response.indexOf("+HTTPACTION: 0,200") >= 0) {
    Serial.println("    ✓ HTTP GET successful (200 OK)");
    
    // Read response
    sim7600.println("AT+HTTPREAD=0,100");
    response = readResponse(2000);
    Serial.println("    Response preview: " + response.substring(0, 50) + "...");
  } else if (response.indexOf("+HTTPACTION:") >= 0) {
    Serial.println("    ✗ HTTP GET failed (non-200 status)");
  } else {
    Serial.println("    ✗ HTTP GET timeout");
  }
  
  // Cleanup
  sim7600.println("AT+HTTPTERM");
  delay(500);
}

void testHTTPS() {
  Serial.println("  Testing HTTPS...");
  
  // Configure SSL
  sendATCommand("AT+CSSLCFG=\"sslversion\",0,3", "SSL Version");
  sendATCommand("AT+CSSLCFG=\"authmode\",0,0", "Auth Mode");
  sendATCommand("AT+CSSLCFG=\"ignorelocaltime\",0,1", "Ignore Time");
  
  // Test HTTPS initialization
  sim7600.println("AT+HTTPTERM");
  delay(500);
  flushBuffer();
  
  sim7600.println("AT+HTTPINIT");
  String response = readResponse(3000);
  if (response.indexOf("OK") >= 0) {
    Serial.println("    ✓ HTTPS ready");
  } else {
    Serial.println("    ✗ HTTPS initialization failed");
  }
  
  sim7600.println("AT+HTTPTERM");
  delay(500);
}

void testGPS() {
  Serial.println("  Testing GPS/GNSS...");
  
  // Power on GPS
  sim7600.println("AT+CGPS=1");
  String response = readResponse(2000);
  if (response.indexOf("OK") < 0 && response.indexOf("ERR=507") < 0) {
    Serial.println("    ✗ Failed to power on GPS");
    return;
  }
  
  delay(2000);  // Give GPS time to get a fix
  
  // Get GPS info
  sim7600.println("AT+CGPSINFO");
  response = readResponse(2000);
  
  if (response.indexOf("+CGPSINFO:") >= 0) {
    int start = response.indexOf(":");
    int end = response.indexOf("\r", start);
    if (end > start) {
      String gpsData = response.substring(start + 1, end);
      gpsData.trim();
      if (gpsData.length() > 10 && gpsData.indexOf(",,,") < 0) {
        Serial.println("    ✓ GPS Fix: " + gpsData);
      } else {
        Serial.println("    ⚠️  GPS powered but no fix yet");
      }
    }
  } else {
    Serial.println("    ✗ No GPS response");
  }
  
  // Power off GPS to save battery
  sim7600.println("AT+CGPS=0");
  delay(500);
}

void testFirebaseConnection() {
  Serial.println("  Testing Firebase connection...");
  
  // Build Firebase URL
  String url = FIREBASE_HOST + "/" + DEVICE_ID + "/test.json";
  
  // Initialize HTTP
  sim7600.println("AT+HTTPTERM");
  delay(500);
  flushBuffer();
  
  sim7600.println("AT+HTTPINIT");
  String response = readResponse(3000);
  if (response.indexOf("OK") < 0) {
    Serial.println("    ✗ Failed to initialize HTTP");
    return;
  }
  
  // Set Firebase URL
  String urlCmd = "AT+HTTPPARA=\"URL\",\"" + url + "\"";
  sim7600.println(urlCmd);
  response = readResponse(3000);
  if (response.indexOf("OK") < 0) {
    Serial.println("    ✗ Failed to set Firebase URL");
    sim7600.println("AT+HTTPTERM");
    return;
  }
  
  // Set content type for JSON
  sim7600.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  response = readResponse(2000);
  
  // Prepare test data
  StaticJsonDocument<200> doc;
  doc["timestamp"] = millis();
  doc["test"] = true;
  doc["module"] = "SIM7600G-H";
  
  String jsonData;
  serializeJson(doc, jsonData);
  
  // Set data length and send
  String dataCmd = "AT+HTTPDATA=" + String(jsonData.length()) + ",10000";
  sim7600.println(dataCmd);
  response = readResponse(2000);
  
  if (response.indexOf("DOWNLOAD") >= 0) {
    sim7600.print(jsonData);
    delay(100);
    
    // Send PUT request
    sim7600.println("AT+HTTPACTION=1");
    response = readResponse(15000);
    
    if (response.indexOf("+HTTPACTION: 1,200") >= 0) {
      Serial.println("    ✓ Firebase PUT successful!");
      Serial.println("    Data written to: " + url);
    } else if (response.indexOf("+HTTPACTION: 1,401") >= 0) {
      Serial.println("    ⚠️  Firebase authentication required");
    } else {
      Serial.println("    ✗ Firebase connection failed");
    }
  }
  
  // Cleanup
  sim7600.println("AT+HTTPTERM");
  delay(500);
}

String readResponse(int timeout) {
  String response = "";
  unsigned long start = millis();
  
  while (millis() - start < timeout) {
    while (sim7600.available()) {
      char c = sim7600.read();
      response += c;
    }
    
    // Check for complete response
    if (response.indexOf("OK\r\n") >= 0 || 
        response.indexOf("ERROR\r\n") >= 0 ||
        response.indexOf("+HTTPACTION:") >= 0 ||
        response.indexOf("DOWNLOAD") >= 0) {
      delay(50);  // Small delay to catch any trailing data
      while (sim7600.available()) {
        response += (char)sim7600.read();
      }
      break;
    }
    
    delay(10);
  }
  
  return response;
}

void flushBuffer() {
  while (sim7600.available()) {
    sim7600.read();
  }
}