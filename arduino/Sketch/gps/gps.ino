#include <HardwareSerial.h>
HardwareSerial sim7600(1); // UART1 on ESP32 (RX=16, TX=17)

void setup() {
  Serial.begin(115200);
  sim7600.begin(115200, SERIAL_8N1, 16, 17);
  delay(3000);
  Serial.println("==== SIM7600 GPS (CGNSS) Tracker ====");
  flushSIM();
  sendAT("AT", "Check Communication");
  
  // Enable GNSS (GPS)
  if (!sendAT("AT+CGNSS=1", "Enable GNSS")) {
    Serial.println("⚠️ Failed to enable GNSS. Check SIM7600 firmware or wiring.");
    return;
  }
  Serial.println("🛰️ Waiting for GPS fix...");
}

void loop() {
  delay(2000); // Wait between reads
  flushSIM();
  sim7600.println("AT+CGNSSINFO");
  
  String response = waitForResponse(3000);
  if (response.indexOf("+CGNSSINFO:") != -1) {
    Serial.println("📡 GPS Response:");
    Serial.println(response);
    
    if (hasValidFix(response)) {
      String lat = extractField(response, 4);
      String lat_dir = extractField(response, 5);
      String lon = extractField(response, 6);
      String lon_dir = extractField(response, 7);
      String utc_time = extractField(response, 9);
      String altitude = extractField(response, 10);
      
      Serial.println("✅ GPS Fix Acquired!");
      Serial.print("Latitude: "); Serial.print(lat); Serial.print(" "); Serial.println(lat_dir);
      Serial.print("Longitude: "); Serial.print(lon); Serial.print(" "); Serial.println(lon_dir);
      Serial.print("Altitude: "); Serial.println(altitude);
      Serial.print("UTC Time: "); Serial.println(utc_time);
      
      // Convert to decimal degrees and create Google Maps link
      if (lat.length() > 0 && lon.length() > 0) {
        double lat_decimal = convertToDecimal(lat, lat_dir);
        double lon_decimal = convertToDecimal(lon, lon_dir);
        
        Serial.println("\n📍 Google Maps Coordinates:");
        Serial.print("Decimal: "); 
        Serial.print(lat_decimal, 6); Serial.print(", "); Serial.println(lon_decimal, 6);
        
        Serial.println("🌐 Google Maps Link:");
        Serial.print("https://www.google.com/maps?q=");
        Serial.print(lat_decimal, 6); Serial.print(","); Serial.println(lon_decimal, 6);
        Serial.println();
      }
    } else {
      Serial.println("⏳ No fix yet. Waiting...");
    }
  } else {
    Serial.println("❌ Failed to get GPS data.");
    Serial.println(response);
  }
}

// Convert GPS coordinates from DDMM.MMMMMM format to decimal degrees
double convertToDecimal(String coord, String direction) {
  if (coord.length() < 4) return 0.0;
  
  // Find the decimal point
  int dotIndex = coord.indexOf('.');
  if (dotIndex == -1) return 0.0;
  
  // Extract degrees (everything before the last 2 digits before decimal)
  String degreesStr = coord.substring(0, dotIndex - 2);
  // Extract minutes (last 2 digits before decimal + everything after decimal)
  String minutesStr = coord.substring(dotIndex - 2);
  
  double degrees = degreesStr.toDouble();
  double minutes = minutesStr.toDouble();
  
  double decimal = degrees + (minutes / 60.0);
  
  // Apply direction (S and W are negative)
  if (direction == "S" || direction == "W") {
    decimal = -decimal;
  }
  
  return decimal;
}

bool sendAT(const char* cmd, const char* label) {
  Serial.print("[CMD] "); Serial.print(label); Serial.print(" -> "); Serial.println(cmd);
  flushSIM();
  sim7600.println(cmd);
  
  String response = waitForResponse(3000);
  Serial.println("[RESPONSE]"); Serial.println(response);
  return response.indexOf("OK") != -1;
}

String waitForResponse(unsigned long timeoutMs) {
  unsigned long timeout = millis() + timeoutMs;
  String response = "";
  
  while (millis() < timeout) {
    while (sim7600.available()) {
      char c = sim7600.read();
      response += c;
    }
    if (response.indexOf("OK") != -1 || response.indexOf("ERROR") != -1) break;
  }
  return response;
}

void flushSIM() {
  while (sim7600.available()) sim7600.read();
}

// Extract comma-separated field from +CGNSSINFO response
String extractField(String data, int fieldIndex) {
  int index = data.indexOf(":");
  if (index == -1) return "";
  
  data = data.substring(index + 1);
  data.trim();
  
  int start = 0;
  for (int i = 0; i < fieldIndex; i++) {
    start = data.indexOf(',', start) + 1;
    if (start == 0) return "";
  }
  
  int end = data.indexOf(',', start);
  if (end == -1) end = data.length();
  
  return data.substring(start, end);
}

// GPS fix status: 00 = no fix, 01 = 2D fix, 02 = 3D fix
bool hasValidFix(String response) {
  String fix = extractField(response, 1);
  fix.trim(); // Remove any whitespace
  return fix == "01" || fix == "02";
}