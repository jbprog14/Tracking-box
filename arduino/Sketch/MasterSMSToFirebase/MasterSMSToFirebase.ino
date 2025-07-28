/*
 * =====================================================================
 * MASTER SMS RECEIVER - DEBUG TOOL
 * =====================================================================
 * 
 * This sketch acts as a simple Master device that only receives and
 * displays SMS messages from any sender. It does not parse the
 * content or communicate with Firebase.
 *
 * This is useful for debugging the SMS sending functionality of the
 * TrackingBoxMain.ino sketch.
 *
 * =====================================================================
 */

#include <HardwareSerial.h>

// =====================================================================
// PIN DEFINITIONS
// =====================================================================
#define SIM_RX 16
#define SIM_TX 17
#define SIM_BAUD 115200

// =====================================================================
// GLOBAL OBJECTS
// =====================================================================
HardwareSerial sim7600(1);

// =====================================================================
// FUNCTION DECLARATIONS
// =====================================================================
void readModem();
void readAllSMS();
void sendAT(const char *cmd, int waitTime);

// =====================================================================
// SETUP
// =================================S====================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("===== MASTER SMS RECEIVER - DEBUG MODE =====");
  
  // Initialize SIM7600
  sim7600.begin(SIM_BAUD, SERIAL_8N1, SIM_RX, SIM_TX);
  delay(3000);
  
  // Configure SIM7600 for SMS
  sendAT("AT", 1000);                 // Check connectivity
  sendAT("AT+CMGF=1", 1000);          // Set to Text Mode
  sendAT("AT+CPMS=\"SM\",\"SM\",\"SM\"", 1000); // Use SIM storage
  sendAT("AT+CNMI=1,2,0,0,0", 1000); // New SMS indication
  
  Serial.println("\n✅ SIM7600 Initialized. Waiting for SMS messages...");
}

// =====================================================================
// MAIN LOOP
// =====================================================================
void loop() {
  readAllSMS();
  delay(5000);  // Check for new messages every 5 seconds
}

// =====================================================================
// SMS READING FUNCTIONS
// =====================================================================

// Read all SMS messages from storage
void readAllSMS() {
  sim7600.println("AT+CMGL=\"ALL\"");  // List ALL messages
  delay(1000);

  if (sim7600.available()) {
    String response = sim7600.readString();
    
    // Check if there are any messages in the response
    if (response.indexOf("+CMGL:") != -1) {
      Serial.println("\n--- NEW MESSAGE(S) RECEIVED ---");
      Serial.print(response);
      Serial.println("---------------------------------");

      // Optional: Delete all messages after reading to avoid re-reading them
      // Uncomment the line below to enable auto-deletion
      // sendAT("AT+CMGD=1,4", 2000); // Delete all messages
    }
  }
}

// Send an AT command and print the response
void sendAT(const char *cmd, int waitTime) {
  Serial.print(">> Sending: ");
  Serial.println(cmd);
  
  sim7600.println(cmd);
  delay(waitTime);
  
  readModem();
}

// Read and print response from the modem
void readModem() {
  String response = "";
  while (sim7600.available()) {
    response += (char)sim7600.read();
  }
  
  if (response.length() > 0) {
    Serial.print("<< Received: ");
    Serial.println(response);
  }
} 