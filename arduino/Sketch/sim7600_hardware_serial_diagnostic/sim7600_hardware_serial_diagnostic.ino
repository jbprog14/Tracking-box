/*
 * SIM7600G Hardware Serial Comprehensive Diagnostic
 * Tests all possible hardware serial configurations
 * Helps identify wiring and configuration issues
 */

// Pin configurations to test
struct UARTConfig {
  int uartNum;
  int rxPin;
  int txPin;
  int baudRate;
  const char* description;
};

// Test configurations - all possible UART combinations
UARTConfig configs[] = {
  // UART1 configurations (your current setup)
  {1, 16, 17, 115200, "UART1: RX=16, TX=17 @ 115200"},
  {1, 16, 17, 9600,   "UART1: RX=16, TX=17 @ 9600"},
  {1, 16, 17, 57600,  "UART1: RX=16, TX=17 @ 57600"},
  {1, 17, 16, 115200, "UART1: RX=17, TX=16 @ 115200 (SWAPPED)"},
  {1, 17, 16, 9600,   "UART1: RX=17, TX=16 @ 9600 (SWAPPED)"},
  
  // UART2 configurations
  {2, 16, 17, 115200, "UART2: RX=16, TX=17 @ 115200"},
  {2, 16, 17, 9600,   "UART2: RX=16, TX=17 @ 9600"},
  {2, 17, 16, 115200, "UART2: RX=17, TX=16 @ 115200 (SWAPPED)"},
  {2, 18, 19, 115200, "UART2: RX=18, TX=19 @ 115200"},
  {2, 18, 19, 9600,   "UART2: RX=18, TX=19 @ 9600"},
  {2, 19, 18, 115200, "UART2: RX=19, TX=18 @ 115200 (SWAPPED)"},
  
  // UART1 with alternative pins
  {1, 4, 5, 115200,   "UART1: RX=4, TX=5 @ 115200"},
  {1, 4, 5, 9600,     "UART1: RX=4, TX=5 @ 9600"},
  {1, 5, 4, 115200,   "UART1: RX=5, TX=4 @ 115200 (SWAPPED)"},
  {1, 12, 13, 115200, "UART1: RX=12, TX=13 @ 115200"},
  {1, 12, 13, 9600,   "UART1: RX=12, TX=13 @ 9600"},
  
  // UART2 with alternative pins
  {2, 4, 5, 115200,   "UART2: RX=4, TX=5 @ 115200"},
  {2, 4, 5, 9600,     "UART2: RX=4, TX=5 @ 9600"},
  {2, 12, 13, 115200, "UART2: RX=12, TX=13 @ 115200"},
  {2, 12, 13, 9600,   "UART2: RX=12, TX=13 @ 9600"}
};

const int NUM_CONFIGS = sizeof(configs) / sizeof(configs[0]);
HardwareSerial* sim7600 = nullptr;

// Power control pin (if connected)
#define SIM7600_PWRKEY -1  // Set to actual pin if PWRKEY is connected
#define SIM7600_RESET  -1  // Set to actual pin if RESET is connected

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(2000);
  
  Serial.println("\n\n================================================");
  Serial.println("    SIM7600G HARDWARE SERIAL DIAGNOSTIC");
  Serial.println("================================================");
  Serial.println("This will test all hardware UART configurations");
  Serial.println("to identify the correct setup for your module.\n");
  
  // Power cycle module if pins are defined
  if (SIM7600_PWRKEY > 0) {
    Serial.println("Power cycling SIM7600G module...");
    powerCycleModule();
  }
  
  if (SIM7600_RESET > 0) {
    Serial.println("Resetting SIM7600G module...");
    resetModule();
  }
  
  Serial.println("\nStarting diagnostic tests...\n");
  
  // Test each configuration
  int workingConfigs = 0;
  
  for (int i = 0; i < NUM_CONFIGS; i++) {
    Serial.println("------------------------------------------------");
    Serial.println(configs[i].description);
    Serial.println("------------------------------------------------");
    
    // Clean up previous instance
    if (sim7600 != nullptr) {
      sim7600->end();
      delete sim7600;
      sim7600 = nullptr;
    }
    
    // Create new HardwareSerial instance
    if (configs[i].uartNum == 0) {
      sim7600 = &Serial;  // UART0 (usually USB)
      Serial.println("⚠️  UART0 typically used for USB - skipping");
      continue;
    } else if (configs[i].uartNum == 1) {
      sim7600 = new HardwareSerial(1);
    } else if (configs[i].uartNum == 2) {
      sim7600 = new HardwareSerial(2);
    }
    
    // Initialize with specific pins and baud rate
    sim7600->begin(configs[i].baudRate, SERIAL_8N1, configs[i].rxPin, configs[i].txPin);
    sim7600->setRxBufferSize(2048);  // Increase buffer size
    delay(500);
    
    // Flush any existing data
    while (sim7600->available()) {
      sim7600->read();
    }
    
    // Run diagnostic tests
    bool success = runDiagnosticTests();
    
    if (success) {
      Serial.println("✅ CONFIGURATION WORKING!");
      workingConfigs++;
      
      // Run extended tests on working configuration
      runExtendedTests();
    } else {
      Serial.println("❌ Configuration failed");
    }
    
    Serial.println();
    delay(500);
  }
  
  // Print summary
  Serial.println("\n================================================");
  Serial.println("              DIAGNOSTIC SUMMARY");
  Serial.println("================================================");
  Serial.print("Working configurations found: ");
  Serial.println(workingConfigs);
  
  if (workingConfigs == 0) {
    Serial.println("\n⚠️  NO WORKING CONFIGURATIONS FOUND!");
    Serial.println("\nPOSSIBLE ISSUES:");
    Serial.println("1. WIRING: Check TX/RX connections (should be crossed)");
    Serial.println("2. POWER: SIM7600G needs 3.7-4.2V and up to 2A");
    Serial.println("3. GROUND: Ensure common ground between ESP32 and SIM7600G");
    Serial.println("4. MODULE: Module might be off or in sleep mode");
    Serial.println("5. ANTENNA: Ensure antenna is connected");
    Serial.println("\nTROUBLESHOOTING STEPS:");
    Serial.println("1. Measure voltage at SIM7600G VCC (should be 3.7-4.2V)");
    Serial.println("2. Check STATUS LED on SIM7600G module");
    Serial.println("3. Try manual AT commands via USB-to-TTL adapter");
    Serial.println("4. Power cycle the module (PWRKEY pin)");
  } else {
    Serial.println("\n✅ Use one of the working configurations above");
    Serial.println("   in your main sketch.");
  }
}

void loop() {
  // Interactive mode - type AT commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      if (sim7600 != nullptr) {
        Serial.println("Sending: " + cmd);
        sim7600->println(cmd);
        delay(100);
        String response = readResponse(2000);
        if (response.length() > 0) {
          Serial.println("Response:");
          Serial.println(response);
        } else {
          Serial.println("No response");
        }
      } else {
        Serial.println("No serial connection active. Restart to test again.");
      }
    }
  }
}

bool runDiagnosticTests() {
  Serial.print("  Testing AT command... ");
  
  // Try multiple times with different delays
  for (int attempt = 0; attempt < 3; attempt++) {
    // Send AT command
    sim7600->println("AT");
    String response = readResponse(1000);
    
    if (response.indexOf("OK") >= 0) {
      Serial.println("✓");
      
      // Test additional commands
      Serial.print("  Getting module info... ");
      sim7600->println("AT+CGMM");
      response = readResponse(1000);
      
      if (response.indexOf("SIM") >= 0 || response.indexOf("SIMCOM") >= 0) {
        // Extract module name
        int start = response.indexOf("SIM");
        if (start >= 0) {
          int end = response.indexOf("\r", start);
          if (end > start) {
            Serial.print("✓ ");
            Serial.println(response.substring(start, end));
          } else {
            Serial.println("✓");
          }
        } else {
          Serial.println("✓");
        }
        return true;
      } else {
        Serial.println("✗");
        return false;
      }
    }
    
    if (attempt < 2) {
      delay(500);  // Wait before retry
    }
  }
  
  Serial.println("✗");
  return false;
}

void runExtendedTests() {
  Serial.println("  Running extended diagnostics:");
  
  // Check SIM card
  Serial.print("    SIM Card: ");
  sim7600->println("AT+CPIN?");
  String response = readResponse(2000);
  if (response.indexOf("READY") >= 0) {
    Serial.println("✓ Ready");
  } else if (response.indexOf("SIM PIN") >= 0) {
    Serial.println("⚠️  PIN Required");
  } else if (response.indexOf("SIM PUK") >= 0) {
    Serial.println("⚠️  PUK Required");
  } else {
    Serial.println("✗ Not detected or error");
  }
  
  // Check network registration
  Serial.print("    Network: ");
  sim7600->println("AT+CREG?");
  response = readResponse(2000);
  if (response.indexOf(",1") >= 0 || response.indexOf(",5") >= 0) {
    Serial.println("✓ Registered");
  } else if (response.indexOf(",2") >= 0) {
    Serial.println("⚠️  Searching...");
  } else {
    Serial.println("✗ Not registered");
  }
  
  // Check signal strength
  Serial.print("    Signal: ");
  sim7600->println("AT+CSQ");
  response = readResponse(2000);
  int start = response.indexOf("+CSQ:");
  if (start >= 0) {
    int comma = response.indexOf(",", start);
    if (comma > start + 5) {
      String rssi = response.substring(start + 5, comma);
      rssi.trim();
      int rssiVal = rssi.toInt();
      if (rssiVal == 99) {
        Serial.println("✗ No signal");
      } else if (rssiVal < 10) {
        Serial.print("⚠️  Weak (");
        Serial.print(rssiVal);
        Serial.println("/31)");
      } else {
        Serial.print("✓ Good (");
        Serial.print(rssiVal);
        Serial.println("/31)");
      }
    } else {
      Serial.println("? Unable to parse");
    }
  } else {
    Serial.println("✗ No response");
  }
  
  // Check HTTP capability
  Serial.print("    HTTP: ");
  sim7600->println("AT+HTTPINIT");
  response = readResponse(3000);
  if (response.indexOf("OK") >= 0) {
    Serial.println("✓ Supported");
    sim7600->println("AT+HTTPTERM");
    readResponse(1000);
  } else if (response.indexOf("ERROR") >= 0) {
    // Try to terminate and reinit
    sim7600->println("AT+HTTPTERM");
    delay(500);
    sim7600->println("AT+HTTPINIT");
    response = readResponse(3000);
    if (response.indexOf("OK") >= 0) {
      Serial.println("✓ Supported");
      sim7600->println("AT+HTTPTERM");
      readResponse(1000);
    } else {
      Serial.println("✗ Not available");
    }
  }
}

String readResponse(int timeout) {
  String response = "";
  unsigned long start = millis();
  
  while (millis() - start < timeout) {
    while (sim7600->available()) {
      char c = sim7600->read();
      response += c;
    }
    
    // Check for complete response
    if (response.indexOf("OK\r\n") >= 0 || 
        response.indexOf("ERROR\r\n") >= 0 ||
        response.indexOf("> ") >= 0) {
      break;
    }
    
    delay(10);
  }
  
  return response;
}

void powerCycleModule() {
  pinMode(SIM7600_PWRKEY, OUTPUT);
  
  // Power off
  digitalWrite(SIM7600_PWRKEY, HIGH);
  delay(1500);
  digitalWrite(SIM7600_PWRKEY, LOW);
  delay(1500);
  
  // Power on
  digitalWrite(SIM7600_PWRKEY, HIGH);
  delay(500);
  digitalWrite(SIM7600_PWRKEY, LOW);
  delay(500);
  digitalWrite(SIM7600_PWRKEY, HIGH);
  
  // Wait for module to boot
  Serial.println("Waiting for module to boot...");
  delay(5000);
}

void resetModule() {
  pinMode(SIM7600_RESET, OUTPUT);
  
  digitalWrite(SIM7600_RESET, LOW);
  delay(100);
  digitalWrite(SIM7600_RESET, HIGH);
  delay(3000);
}