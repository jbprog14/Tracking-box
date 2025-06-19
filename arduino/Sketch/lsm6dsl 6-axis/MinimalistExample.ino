#include <Wire.h>

// LSM6DSL I2C addresses (primary and alternate)
#define LSM6DSL_ADDRESS_1 0x6A  // Primary address (SDO/SA0 connected to VCC)
#define LSM6DSL_ADDRESS_2 0x6B  // Alternate address (SDO/SA0 connected to GND)

uint8_t LSM6DSL_ADDRESS = LSM6DSL_ADDRESS_1;  // Will be determined during init

// LSM6DSL Register addresses
#define LSM6DSL_WHO_AM_I 0x0F
#define LSM6DSL_CTRL1_XL 0x10
#define LSM6DSL_CTRL2_G 0x11
#define LSM6DSL_CTRL3_C 0x12
#define LSM6DSL_OUTX_L_XL 0x28
#define LSM6DSL_OUTX_H_XL 0x29
#define LSM6DSL_OUTY_L_XL 0x2A
#define LSM6DSL_OUTY_H_XL 0x2B
#define LSM6DSL_OUTZ_L_XL 0x2C
#define LSM6DSL_OUTZ_H_XL 0x2D

// Tilt detection parameters
#define TILT_THRESHOLD 0.9  // Approximately 90 degrees (cos(90°) ≈ 0)
#define SAMPLE_INTERVAL 50  // milliseconds

// Variables
float accelX, accelY, accelZ;
bool tiltDetected = false;
unsigned long lastSampleTime = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);  // Give time for Serial Monitor to connect
  
  Serial.println("ESP32 LSM6DSL Tilt Interrupt Detection");
  Serial.println("=====================================");
  
  // Initialize I2C with explicit pins
  Wire.begin(21, 22);  // SDA, SCL
  Wire.setClock(100000);  // 100kHz I2C speed
  
  Serial.println("I2C initialized on pins SDA=21, SCL=22");
  
  // Scan for I2C devices
  scanI2C();
  
  // Initialize LSM6DSL
  if (initLSM6DSL()) {
    Serial.println("LSM6DSL initialized successfully!");
    Serial.print("Using I2C address: 0x");
    Serial.println(LSM6DSL_ADDRESS, HEX);
  } else {
    Serial.println("Failed to initialize LSM6DSL!");
    Serial.println("Check wiring and connections:");
    Serial.println("- VCC to 3.3V");
    Serial.println("- GND to GND");
    Serial.println("- SDA to GPIO 21");
    Serial.println("- SCL to GPIO 22");
    while(1) {
      delay(1000);
      Serial.println("Retrying initialization...");
      if (initLSM6DSL()) {
        Serial.println("LSM6DSL initialized successfully!");
        break;
      }
    }
  }
  
  Serial.println("Monitoring for 90-degree tilts...");
  Serial.println("Threshold: ±0.9g on any axis");
  Serial.println("");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Sample at defined interval
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentTime;
    
    // Read accelerometer data
    readAccelerometer();
    
    // Check for tilt interrupt condition
    checkTiltInterrupt();
    
    // Optional: Display continuous readings (uncomment if needed)
    // displayAccelData();
  }
}

bool initLSM6DSL() {
  Serial.println("Attempting to initialize LSM6DSL...");
  
  // Try both possible I2C addresses
  uint8_t addresses[] = {LSM6DSL_ADDRESS_1, LSM6DSL_ADDRESS_2};
  
  for (int i = 0; i < 2; i++) {
    LSM6DSL_ADDRESS = addresses[i];
    Serial.print("Trying address 0x");
    Serial.println(LSM6DSL_ADDRESS, HEX);
    
    // Check WHO_AM_I register
    uint8_t whoAmI = readRegister(LSM6DSL_WHO_AM_I);
    Serial.print("WHO_AM_I response: 0x");
    Serial.println(whoAmI, HEX);
    
    if (whoAmI == 0x6A) {
      Serial.println("LSM6DSL found!");
      
      // Configure accelerometer
      Serial.println("Configuring accelerometer...");
      writeRegister(LSM6DSL_CTRL1_XL, 0x60);  // ODR = 208 Hz, ±2g
      
      // Configure gyroscope
      Serial.println("Configuring gyroscope...");
      writeRegister(LSM6DSL_CTRL2_G, 0x60);   // ODR = 208 Hz, 1000 dps
      
      // Enable block data update
      Serial.println("Enabling block data update...");
      writeRegister(LSM6DSL_CTRL3_C, 0x40);
      
      delay(100); // Allow sensor to stabilize
      
      // Test read to verify communication
      readAccelerometer();
      Serial.print("Initial readings - X: ");
      Serial.print(accelX, 3);
      Serial.print("g, Y: ");
      Serial.print(accelY, 3);
      Serial.print("g, Z: ");
      Serial.print(accelZ, 3);
      Serial.println("g");
      
      return true;
    }
  }
  
  Serial.println("LSM6DSL not found at either address!");
  return false;
}

void readAccelerometer() {
  // Read raw accelerometer data (16-bit, little endian)
  uint8_t xlData[6];
  
  for (int i = 0; i < 6; i++) {
    xlData[i] = readRegister(LSM6DSL_OUTX_L_XL + i);
  }
  
  // Combine high and low bytes
  int16_t rawX = (xlData[1] << 8) | xlData[0];
  int16_t rawY = (xlData[3] << 8) | xlData[2];
  int16_t rawZ = (xlData[5] << 8) | xlData[4];
  
  // Convert to g (±2g full scale, 16-bit resolution)
  accelX = rawX * 2.0 / 32768.0;
  accelY = rawY * 2.0 / 32768.0;
  accelZ = rawZ * 2.0 / 32768.0;
}

void checkTiltInterrupt() {
  bool currentTiltState = false;
  String tiltDirection = "";
  
  // Check if any axis exceeds the tilt threshold
  if (abs(accelX) > TILT_THRESHOLD) {
    currentTiltState = true;
    tiltDirection = (accelX > 0) ? "+X" : "-X";
  }
  else if (abs(accelY) > TILT_THRESHOLD) {
    currentTiltState = true;
    tiltDirection = (accelY > 0) ? "+Y" : "-Y";
  }
  else if (abs(accelZ) > TILT_THRESHOLD) {
    currentTiltState = true;
    tiltDirection = (accelZ > 0) ? "+Z" : "-Z";
  }
  
  // Trigger interrupt on state change (edge detection)
  if (currentTiltState && !tiltDetected) {
    // Rising edge - tilt detected
    Serial.println("*** TILT INTERRUPT TRIGGERED ***");
    Serial.print("Direction: ");
    Serial.println(tiltDirection);
    Serial.print("Acceleration values - X: ");
    Serial.print(accelX, 3);
    Serial.print("g, Y: ");
    Serial.print(accelY, 3);
    Serial.print("g, Z: ");
    Serial.print(accelZ, 3);
    Serial.println("g");
    Serial.print("Timestamp: ");
    Serial.print(millis());
    Serial.println(" ms");
    Serial.println("********************************");
    Serial.println("");
  }
  else if (!currentTiltState && tiltDetected) {
    // Falling edge - returned to normal position
    Serial.println("--- Tilt interrupt cleared ---");
    Serial.println("");
  }
  
  tiltDetected = currentTiltState;
}

void displayAccelData() {
  Serial.print("X: ");
  Serial.print(accelX, 3);
  Serial.print("g | Y: ");
  Serial.print(accelY, 3);
  Serial.print("g | Z: ");
  Serial.print(accelZ, 3);
  Serial.println("g");
}

void scanI2C() {
  Serial.println("Scanning I2C bus...");
  int deviceCount = 0;
  
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found! Check wiring.");
  } else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" I2C device(s)");
  }
  Serial.println("");
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(LSM6DSL_ADDRESS);
  Wire.write(reg);
  uint8_t error = Wire.endTransmission(false);
  
  if (error != 0) {
    Serial.print("I2C write error: ");
    Serial.println(error);
    return 0xFF;
  }
  
  Wire.requestFrom(LSM6DSL_ADDRESS, (uint8_t)1);
  if (Wire.available() < 1) {
    Serial.println("I2C read error: No data available");
    return 0xFF;
  }
  
  return Wire.read();
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(LSM6DSL_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.print("I2C write error: ");
    Serial.println(error);
  }
}