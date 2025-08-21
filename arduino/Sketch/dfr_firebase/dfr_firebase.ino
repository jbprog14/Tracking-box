// ESP32 + SIM7600G Firebase Controller (No WiFi Required)
// This sketch uses ONLY the SIM7600G cellular connection for internet access

// SIM7600G Serial connection
#define SIM7600G_RX 16
#define SIM7600G_TX 17
HardwareSerial sim7600g(2);

// Firebase configuration
const char* firebaseURL = "https://tracking-box-e17a1-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* apn = "internet";  // Change to your carrier's APN

// Timing variables
unsigned long lastSensorRead = 0;
unsigned long sensorInterval = 30000;  // 30 seconds

void setup() {
  Serial.begin(115200);
  sim7600g.begin(115200, SERIAL_8N1, SIM7600G_RX, SIM7600G_TX);
  
  Serial.println("ESP32 + SIM7600G Firebase Controller Starting...");
  delay(3000);
  
  // Initialize SIM7600G
  initializeSIM7600G();
}

void loop() {
  // Read sensors every 30 seconds
  if (millis() - lastSensorRead >= sensorInterval) {
    // Read sensor data (replace with your actual sensors)
    float temperature = readTemperatureSensor();
    int humidity = readHumiditySensor();
    unsigned long timestamp = millis() / 1000;
    
    Serial.println("\n=== SENSOR READING ===");
    Serial.println("Temperature: " + String(temperature) + "°C");
    Serial.println("Humidity: " + String(humidity) + "%");
    Serial.println("Timestamp: " + String(timestamp));
    
    // Send to Firebase
    sendSensorDataToFirebase(temperature, humidity, timestamp);
    
    // Read latest data from Firebase (using working method)
    readFirebaseWorking();
    
    lastSensorRead = millis();
  }
  
  // Check for any incoming serial data from SIM7600G
  if (sim7600g.available()) {
    String response = sim7600g.readString();
    Serial.print("SIM7600G: " + response);
  }
  
  delay(1000);
}

void initializeSIM7600G() {
  Serial.println("\n=== INITIALIZING SIM7600G ===");
  
  // Basic initialization sequence
  sendATCommand("AT", 2000);                                    // Test communication
  sendATCommand("AT+CFUN=1", 5000);                            // Set full functionality
  sendATCommand("AT+CPIN?", 2000);                             // Check SIM status
  sendATCommand("AT+CREG?", 2000);                             // Check network registration
  sendATCommand("AT+CGATT=1", 10000);                          // Attach to GPRS service
  
  // Configure SSL/TLS for HTTPS
  sendATCommand("AT+CSSLCFG=\"sslversion\",0,3", 2000);        // Set TLS 1.2
  sendATCommand("AT+CSSLCFG=\"authmode\",0,0", 2000);          // Disable cert verification
  sendATCommand("AT+CSSLCFG=\"ignorelocaltime\",0,1", 2000);   // Ignore RTC time
  
  // Configure PDP context
  sendATCommand("AT+CGDCONT=1,\"IP\",\"" + String(apn) + "\"", 2000);  // Set APN
  sendATCommand("AT+CGACT=1,1", 10000);                        // Activate PDP context
  
  // Check signal strength
  sendATCommand("AT+CSQ", 2000);                               // Signal quality
  
  Serial.println("SIM7600G initialization complete!");
}

void sendSensorDataToFirebase(float temp, int humid, unsigned long ts) {
  Serial.println("\n=== SENDING DATA TO FIREBASE ===");
  
  // Create JSON string
  String jsonData = "{\"temperature\":" + String(temp) + 
                   ",\"humidity\":" + String(humid) + 
                   ",\"timestamp\":" + String(ts) + 
                   ",\"device\":\"ESP32_SIM7600G\"}";
  
  Serial.println("JSON: " + jsonData);
  Serial.println("Length: " + String(jsonData.length()));
  
  // HTTP sequence for writing
  sendATCommand("AT+HTTPTERM", 1000);
  sendATCommand("AT+HTTPINIT", 2000);
  sendATCommand("AT+HTTPPARA=\"URL\",\"" + String(firebaseURL) + "/sensor_data/" + String(ts) + ".json\"", 2000);
  sendATCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 1000);
  
  // Send data
  String dataCmd = "AT+HTTPDATA=" + String(jsonData.length()) + ",10000";
  Serial.println("Sending: " + dataCmd);
  sim7600g.println(dataCmd);
  
  // Wait for DOWNLOAD prompt
  delay(1000);
  
  // Send JSON data byte by byte
  Serial.println("Sending JSON data...");
  for (int i = 0; i < jsonData.length(); i++) {
    sim7600g.write(jsonData[i]);
    delayMicroseconds(100);
  }
  
  delay(1000);
  
  // Execute HTTP PUT
  sendATCommand("AT+HTTPACTION=1", 10000);
  
  // Read response
  Serial.println("Reading response...");
  delay(2000);
  sendATCommand("AT+HTTPREAD=0,200", 3000);
}

void readLatestDataFromFirebase() {
  Serial.println("\n=== READING FROM FIREBASE ===");
  
  // HTTP sequence for reading - use simple path that works
  sendATCommand("AT+HTTPTERM", 1000);
  sendATCommand("AT+HTTPINIT", 2000);
  sendATCommand("AT+HTTPPARA=\"URL\",\"" + String(firebaseURL) + "/.json\"", 2000);  // Read entire database
  sendATCommand("AT+HTTPACTION=0", 10000);  // GET request
  
  // Note: Check the +HTTPACTION response for data length, then read
  Serial.println("Check +HTTPACTION response above for data length");
  Serial.println("Then manually use: AT+HTTPREAD=0,data_length");
}

void sendATCommand(String command, int timeout) {
  Serial.println("Sending: " + command);
  sim7600g.println(command);
  
  unsigned long startTime = millis();
  String response = "";
  
  while (millis() - startTime < timeout) {
    if (sim7600g.available()) {
      response += sim7600g.readString();
    }
    delay(10);
  }
  
  if (response.length() > 0) {
    Serial.print("Response: " + response);
  }
  delay(100);
}

// Sensor reading functions (replace with your actual sensor code)
float readTemperatureSensor() {
  // Replace with your actual temperature sensor reading
  // Example: DS18B20, DHT22, BME280, etc.
  return 20.0 + (millis() % 1000) / 100.0;  // Simulated data
}

int readHumiditySensor() {
  // Replace with your actual humidity sensor reading
  // Example: DHT22, BME280, SHT30, etc.
  return 50 + (millis() % 500) / 10;  // Simulated data
}

// Utility functions for specific Firebase operations
void sendSingleValue(String path, String value) {
  Serial.println("\n=== SENDING SINGLE VALUE ===");
  Serial.println("Path: " + path);
  Serial.println("Value: " + value);
  
  sendATCommand("AT+HTTPTERM", 1000);
  sendATCommand("AT+HTTPINIT", 2000);
  sendATCommand("AT+HTTPPARA=\"URL\",\"" + String(firebaseURL) + "/" + path + ".json\"", 2000);
  sendATCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 1000);
  
  String dataCmd = "AT+HTTPDATA=" + String(value.length()) + ",10000";
  sim7600g.println(dataCmd);
  delay(1000);
  
  for (int i = 0; i < value.length(); i++) {
    sim7600g.write(value[i]);
    delayMicroseconds(100);
  }
  
  delay(1000);
  sendATCommand("AT+HTTPACTION=1", 5000);
  sendATCommand("AT+HTTPREAD=0,100", 2000);
}

void readSpecificPath(String path) {
  Serial.println("\n=== READING SPECIFIC PATH ===");
  Serial.println("Path: " + path);
  
  sendATCommand("AT+HTTPTERM", 1000);
  sendATCommand("AT+HTTPINIT", 2000);
  sendATCommand("AT+HTTPPARA=\"URL\",\"" + String(firebaseURL) + "/" + path + ".json\"", 2000);
  sendATCommand("AT+HTTPACTION=0", 5000);
  
  Serial.println("Check +HTTPACTION response for data length");
}

// Working Firebase read function (uses your tested sequence)
void readFirebaseWorking() {
  Serial.println("\n=== FIREBASE READ (WORKING METHOD) ===");
  
  sendATCommand("AT+HTTPTERM", 1000);
  sendATCommand("AT+HTTPINIT", 2000);
  sendATCommand("AT+HTTPPARA=\"URL\",\"" + String(firebaseURL) + "/.json\"", 2000);
  sendATCommand("AT+HTTPACTION=0", 5000);
  
  // Wait for +HTTPACTION response to show data length
  delay(3000);
  
  // Then automatically try to read some data
  Serial.println("Attempting to read response...");
  sendATCommand("AT+HTTPREAD=0,200", 3000);  // Try reading first 200 bytes
}

// Helper function to create properly formatted JSON
String createSensorJSON(float temp, int humid, unsigned long ts) {
  return "{\"temperature\":" + String(temp) + 
         ",\"humidity\":" + String(humid) + 
         ",\"timestamp\":" + String(ts) + 
         ",\"device\":\"ESP32_SIM7600G\"}";
}

// Test functions you can call from loop() for debugging
void testFirebaseConnection() {
  Serial.println("\n=== TESTING FIREBASE CONNECTION ===");
  sendSingleValue("test", "\"Hello Firebase\"");
}

void checkNetworkStatus() {
  Serial.println("\n=== NETWORK STATUS ===");
  sendATCommand("AT+CREG?", 2000);    // Network registration
  sendATCommand("AT+CGACT?", 2000);   // PDP context status
  sendATCommand("AT+CSQ", 2000);      // Signal quality
  sendATCommand("AT+COPS?", 2000);    // Current operator
}