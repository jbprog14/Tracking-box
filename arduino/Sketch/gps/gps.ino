#include <HardwareSerial.h>

#define RXD2 16  // ESP32 RX (connect to SIM7600 TX)
#define TXD2 17  // ESP32 TX (connect to SIM7600 RX)

HardwareSerial SerialAT(2);  // Use UART2

void setup() {
  Serial.begin(115200);     // Serial monitor
  delay(1000);
  Serial.println("Initializing...");

  SerialAT.begin(115200, SERIAL_8N1, RXD2, TXD2);  // SIM7600

  delay(3000);

  sendATCommand("AT");                     // Test AT
  sendATCommand("ATE0");                   // Turn off echo
  sendATCommand("AT+CPIN?");               // Check SIM status
  sendATCommand("AT+CSQ");                 // Signal quality
  sendATCommand("AT+CGNSPWR=1");           // Alternative GNSS power command
  sendATCommand("AT+CGPS=1,1");            // Start GPS in standalone mode
}

void loop() {
  Serial.println("Checking GPS status...");
  sendATCommand("AT+CGPSSTATUS?");
  delay(2000);

  Serial.println("Getting GPS location...");
  sendATCommand("AT+CGPSINFO");
  delay(5000);  // Give GPS time to lock, increase if needed
}

// Utility function to send AT commands
void sendATCommand(String cmd) {
  Serial.println(">> " + cmd);
  SerialAT.println(cmd);

  long timeout = millis() + 3000;
  while (millis() < timeout) {
    if (SerialAT.available()) {
      String res = SerialAT.readStringUntil('\n');
      res.trim();
      if (res.length() > 0) {
        Serial.println("<< " + res);
      }
    }
  }
  Serial.println();
}