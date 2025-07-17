#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <HardwareSerial.h>

HardwareSerial SerialAT(2); // RX=16, TX=17
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

const char apn[] = "smartlte";
const char user[] = "";
const char pass[] = "";

// REPLACE with your Firebase host (no https://)
const char FIREBASE_HOST[] = "your-project-id.firebaseio.com";

// OPTIONAL: If using token
// const char AUTH[] = "your_firebase_auth_token";

const int FIREBASE_PORT = 80;
HttpClient http(client, FIREBASE_HOST, FIREBASE_PORT);

void setup() {
  Serial.begin(115200);
  delay(100);

  SerialAT.begin(115200, SERIAL_8N1, 16, 17);
  delay(3000);

  Serial.println("Initializing modem...");
  modem.restart();

  if (!modem.waitForNetwork()) {
    Serial.println("❌ Network failed");
    return;
  }

  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println("❌ GPRS failed");
    return;
  }

  Serial.println("✅ Connected to Internet");

  sendToFirebase();
}

void sendToFirebase() {
  // Replace with your Firebase path
  String path = "/sensorData.json"; // Add "?auth=your_token" if needed

  String jsonData = "{\"temperature\":28,\"humidity\":65}";

  Serial.println("Posting to Firebase...");
  http.beginRequest();
  http.post(path);
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", jsonData.length());
  http.beginBody();
  http.print(jsonData);
  http.endRequest();

  int statusCode = http.responseStatusCode();
  String response = http.responseBody();

  Serial.print("Status Code: ");
  Serial.println(statusCode);
  Serial.print("Response: ");
  Serial.println(response);

  if (statusCode == 200) {
    Serial.println("✅ Data sent successfully.");
  } else {
    Serial.println("❌ Failed to send data.");
  }
}

void loop() {
  // nothing
}
