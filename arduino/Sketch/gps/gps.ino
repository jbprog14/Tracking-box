#include <HardwareSerial.h>

HardwareSerial sim7600(1); // UART1 on ESP32 (RX=16, TX=17)

void setup() {
  Serial.begin(115200);
  sim7600.begin(115200, SERIAL_8N1, 16, 17);

  delay(3000); // Wait for module to boot

  Serial.println("===================================");
  Serial.println("SIM7600G-H GPS Compatibility Test");
  Serial.println("===================================");

  flushSIM();

  sendAT("AT", "Check basic AT response");
  sendAT("AT+CPIN?", "Check SIM status");

  sendAT("AT+CGMR", "Firmware version");
  sendAT("AT+CUSBMODE?", "USB mode");

  sendAT("AT+CGPS=?", "Check AT+CGPS support");
  sendAT("AT+CGNSPWR=?", "Check AT+CGNSPWR support");
  sendAT("AT+CGNSS=?", "Check AT+CGNSS support");

  Serial.println("\n=== Trying to enable GPS if possible ===");

  // Attempt AT+CGPS
  sendAT("AT+CGPS=1,1", "Try enabling GPS via AT+CGPS");
  delay(500);
  sendAT("AT+CGPSINF=0", "Request GPS info (CGPSINF)");

  // Attempt AT+CGNS
  sendAT("AT+CGNSPWR=1", "Try enabling GPS via AT+CGNSPWR");
  delay(500);
  sendAT("AT+CGNSINF", "Request GPS info (CGNSINF)");

  // Attempt AT+CGNSS
  sendAT("AT+CGNSS=1", "Try enabling GPS via AT+CGNSS");
  delay(500);
  sendAT("AT+CGNSSINFO", "Request GPS info (CGNSSINFO)");

  Serial.println("\nCheck output above to determine which GPS command works.");
}

void loop() {
  // Idle
}

// Utility to flush input
void flushSIM() {
  while (sim7600.available()) sim7600.read();
}

// Utility to send AT command and print raw response
void sendAT(const char* cmd, const char* label) {
  Serial.print("\n[CMD] ");
  Serial.print(label);
  Serial.print(" -> ");
  Serial.println(cmd);

  flushSIM();
  sim7600.println(cmd);

  unsigned long timeout = millis() + 3000;
  String response = "";

  while (millis() < timeout) {
    while (sim7600.available()) {
      char c = sim7600.read();
      response += c;
    }
    if (response.indexOf("OK") != -1 || response.indexOf("ERROR") != -1) break;
  }

  Serial.print("[RESPONSE] ");
  Serial.println(response);
}