#define MODEM_RX 16
#define MODEM_TX 17
#define MODEM_BAUD 115200

HardwareSerial SerialAT(1);

void setup() {
  Serial.begin(115200);      // Debug Serial
  SerialAT.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  
  Serial.println("Initializing SIM7600G GPS sequence...");

  sendAT("AT+CGPS=0", 2000);  // Turn off GPS first
  sendAT("AT+CGNSSMODE=15,1", 2000);
  sendAT("AT+CGPSNMEA=200191", 2000);
  sendAT("AT+CGPSNMEARATE=1", 2000);
  sendAT("AT+CGPS=1", 2000);  // Turn on GPS
  sendAT("AT+CGPSINFOCFG=1,31", 2000);

  Serial.println("\nWaiting 3 minutes for GPS to get signal...");
  delay(1 * 60 * 1000); // 3 minutes

  sendAT("AT+CGPSINFOCFG=0,31", 2000);
  sendAT("AT+CGPSPMD?", 2000);
  sendAT("AT+CGPSNMEA?", 2000);

  //Get Location Via GNSS,GPS,CLBS
  sendAT("AT+CGNSSINFO", 2000);
  sendAT("AT+CGPSINFO", 2000);
  sendAT("AT+CLBS=1,1", 2000);

  Serial.println("\nGPS test complete.");
}

void loop() {
  // nothing here
}

void sendAT(String cmd, int delayMs) {
  Serial.println("\n>> " + cmd);
  SerialAT.println(cmd);
  long timeout = millis() + delayMs;
  while (millis() < timeout) {
    while (SerialAT.available()) {
      char c = SerialAT.read();
      Serial.write(c);
    }
  }
}
