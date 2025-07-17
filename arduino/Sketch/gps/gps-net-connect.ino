#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024

#include <TinyGsmClient.h>
#include <HardwareSerial.h>

// SIM7600 Serial
HardwareSerial SerialAT(2); // UART2: GPIO16 (RX), GPIO17 (TX)
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

// SIM APN credentials
const char apn[] = "smartlte"; // Change if you're using another carrier
const char user[] = "";
const char pass[] = "";

// Test domain to "ping"
const char* testHost = "google.com";
const int testPort = 80;

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Booting modem...");
  SerialAT.begin(115200, SERIAL_8N1, 16, 17); // RX, TX
  delay(3000);

  // Restart the modem
  Serial.println("Restarting modem...");
  modem.restart();
  delay(1000);

  String modemInfo = modem.getModemInfo();
  Serial.println("Modem Info: " + modemInfo);

  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println(" ❌ Failed");
    while (true);
  }
  Serial.println(" ✅ Connected");

  // Connect to GPRS
  Serial.print("Connecting to APN: ");
  Serial.println(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println(" ❌ GPRS failed");
    while (true);
  }
  Serial.println(" ✅ GPRS connected");

  // Run HTTP ping test
  runPingTest();
}

void runPingTest() {
  Serial.println("Running HTTP-based ping test...");
  if (!client.connect(testHost, testPort)) {
    Serial.println("❌ Unable to reach " + String(testHost));
    return;
  }

  Serial.println("✅ Connected to " + String(testHost));
  client.println("HEAD / HTTP/1.1");
  client.println("Host: " + String(testHost));
  client.println("Connection: close");
  client.println();

  // Wait for response
  unsigned long timeout = millis() + 5000;
  while (client.connected() && millis() < timeout) {
    while (client.available()) {
      String line = client.readStringUntil('\n');
      Serial.println(line);
    }
  }

  client.stop();
  Serial.println("✅ Ping test complete.");
}

void loop() {
  // Nothing here for now
}
