#include <HardwareSerial.h>

#define SIM_RX 16
#define SIM_TX 17
#define SIM_BAUD 115200

HardwareSerial sim7600(1);

void sendAT(const char *cmd, int waitTime);
void readModem();
void readNewestSMS();

void setup() {
  Serial.begin(115200);
  sim7600.begin(SIM_BAUD, SERIAL_8N1, SIM_RX, SIM_TX);

  Serial.println("Initializing SIM7600...");

  sendAT("AT", 500);
  sendAT("AT+CMGF=1", 500);                   // Text mode
  sendAT("AT+CPMS=\"SM\",\"SM\",\"SM\"", 500); // Use SIM storage
  sendAT("AT+CMGR=0", 500); 


  Serial.println("Starting continuous SMS read loop...");
}

void loop() {
  readNewestSMS();
  delay(2000);  // Wait 2 seconds before checking again
}

// --- FUNCTIONS ---

void readNewestSMS() {
  sim7600.println("AT+CMGL=\"REC UNREAD\"");  // List unread messages
  delay(1000);

  if (sim7600.available()) {
    String response = sim7600.readString();
    if (response.indexOf("+CMGL:") != -1) {
      Serial.println("=== NEW MESSAGE ===");
      Serial.println(response);
      Serial.println("===================");

      // Find message index to delete
      int indexStart = response.indexOf("+CMGL: ") + 7;
      int indexEnd = response.indexOf(",", indexStart);
      int smsIndex = response.substring(indexStart, indexEnd).toInt();

      Serial.printf("Will delete SMS index: %d in 10 seconds...\n", smsIndex);
      delay(10000);

      // Delete SMS
      sim7600.print("AT+CMGD=");
      sim7600.println(smsIndex);
      delay(500);
      Serial.println("SMS deleted.");
    }
  }
}

void sendAT(const char *cmd, int waitTime) {
  sim7600.println(cmd);
  delay(waitTime);
  readModem();
}

void readModem() {
  while (sim7600.available()) {
    Serial.write(sim7600.read());
  }
}