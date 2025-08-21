#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// WiFi credentials
const char* ssid = "archer_2.4G";
const char* password = "05132000";

// Firebase configuration
const char* firebaseURL = "https://tracking-box-e17a1-default-rtdb.asia-southeast1.firebasedatabase.app";

// SIM7600G Serial connection (adjust pins as needed)
#define SIM7600G_RX 16
#define SIM7600G_TX 17
HardwareSerial sim7600g(2);

void setup() {
  Serial.begin(115200);
  sim7600g.begin(115200, SERIAL_8N1, SIM7600G_RX, SIM7600G_TX);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected!");
  
  delay(2000);
}

void loop() {
  // Read sensor data (replace with your actual sensors)
  float temperature = 25.3;
  int humidity = 60;
  unsigned long timestamp = millis() / 1000;
  
  // Read from Firebase via ESP32 (for verification)
  readFromFirebaseViaESP32();
  
  // Read from Firebase via SIM7600G (your working method)
  readFromFirebaseViaSIM7600G();
  
  // Send to Firebase via ESP32 (for verification)
  sendToFirebaseViaESP32(temperature, humidity, timestamp);
  
  // Send to Firebase via SIM7600G
  sendToFirebaseViaSIM7600G(temperature, humidity, timestamp);
  
  delay(30000); // Send every 30 seconds
}

void readFromFirebaseViaESP32() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(String(firebaseURL) + "/.json");
    
    int httpResponseCode = http.GET();
    
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("ESP32 Read Response Code: " + String(httpResponseCode));
      Serial.println("ESP32 Read Data: " + response);
    } else {
      Serial.println("ESP32 Read Error: " + String(httpResponseCode));
    }
    
    http.end();
  }
}

void readFromFirebaseViaSIM7600G() {
  Serial.println("\n=== Reading from Firebase via SIM7600G ===");
  
  // Your working read sequence
  sendATCommand("AT+HTTPTERM", 1000);
  sendATCommand("AT+HTTPINIT", 2000);
  sendATCommand("AT+HTTPPARA=\"URL\",\"" + String(firebaseURL) + "/.json\"", 2000);
  sendATCommand("AT+HTTPACTION=0", 5000);  // GET request
  
  // Note: You'll need to check the response and use AT+HTTPREAD=0,length
  Serial.println("Check the +HTTPACTION response above, then use:");
  Serial.println("AT+HTTPREAD=0,data_length");
  Serial.println("=====================================\n");
}

void sendToFirebaseViaESP32(float temp, int humid, unsigned long ts) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(String(firebaseURL) + "/esp32_data.json");
    http.addHeader("Content-Type", "application/json");
    
    // Create JSON
    StaticJsonDocument<200> doc;
    doc["temperature"] = temp;
    doc["humidity"] = humid;
    doc["timestamp"] = ts;
    doc["source"] = "ESP32";
    
    String jsonString;
    serializeJson(doc, jsonString);
    
    int httpResponseCode = http.PUT(jsonString);
    
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("ESP32 -> Firebase: " + String(httpResponseCode));
      Serial.println("Response: " + response);
    } else {
      Serial.println("ESP32 Firebase Error: " + String(httpResponseCode));
    }
    
    http.end();
  }
}

void sendToFirebaseViaSIM7600G(float temp, int humid, unsigned long ts) {
  Serial.println("\n=== Sending via SIM7600G ===");
  
  // Create JSON string
  String jsonData = "{\"temperature\":" + String(temp) + 
                   ",\"humidity\":" + String(humid) + 
                   ",\"timestamp\":" + String(ts) + 
                   ",\"source\":\"SIM7600G\"}";
  
  Serial.println("JSON to send: " + jsonData);
  Serial.println("JSON length: " + String(jsonData.length()));
  
  // Send AT commands to SIM7600G
  sendATCommand("AT+HTTPTERM", 1000);
  sendATCommand("AT+HTTPINIT", 2000);
  sendATCommand("AT+HTTPPARA=\"URL\",\"" + String(firebaseURL) + "/sim7600g_data.json\"", 2000);
  sendATCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 1000);
  
  // Prepare data
  String dataCmd = "AT+HTTPDATA=" + String(jsonData.length()) + ",10000";
  sim7600g.println(dataCmd);
  Serial.println("Sent: " + dataCmd);
  
  // Wait for DOWNLOAD prompt
  delay(1000);
  
  // Send JSON data byte by byte to avoid terminal issues
  for (int i = 0; i < jsonData.length(); i++) {
    sim7600g.write(jsonData[i]);
    delayMicroseconds(100); // Small delay between characters
  }
  
  Serial.println("JSON data sent to SIM7600G");
  delay(1000);
  
  // Execute HTTP action
  sendATCommand("AT+HTTPACTION=1", 5000);
  
  // Read response
  sendATCommand("AT+HTTPREAD=0,100", 2000);
}

void sendATCommand(String command, int timeout) {
  Serial.println("Sending: " + command);
  sim7600g.println(command);
  
  long int time = millis();
  while ((time + timeout) > millis()) {
    while (sim7600g.available()) {
      String response = sim7600g.readString();
      Serial.print("SIM7600G: " + response);
    }
  }
  Serial.println();
}

// Helper function to print formatted JSON for manual testing
void printFormattedJSON(float temp, int humid, unsigned long ts) {
  Serial.println("\n=== Copy this JSON for manual testing ===");
  String json = "{\"temperature\":" + String(temp) + 
                ",\"humidity\":" + String(humid) + 
                ",\"timestamp\":" + String(ts) + "}";
  Serial.println(json);
  Serial.println("Length: " + String(json.length()));
  Serial.println("=========================================\n");
}