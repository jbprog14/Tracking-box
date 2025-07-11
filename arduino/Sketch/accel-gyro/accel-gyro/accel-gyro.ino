/*
 * =====================================================================
 * LSM6DSL ACCELEROMETER TEST
 * =====================================================================
 * * Tests the LSM6DSL 6-axis Accelerometer/Gyroscope for tilt and fall detection.
 * * Pin Configuration:
 * - SDA: GPIO21 (I2C Data)
 * - SCL: GPIO22 (I2C Clock) 
 * - INT1: GPIO34 (Accelerometer Interrupt - not used in this test)
 * * Sensor:
 * - LSM6DSL: 6-axis Accelerometer/Gyroscope (I2C Address: 0x6A or 0x6B)
 * * =====================================================================
 */

#include <Wire.h>
#include <math.h> // Required for sqrt and abs functions for magnitude calculation

// Pin Definitions
#define SDA_PIN 21
#define SCL_PIN 22
#define INT1_PIN 34   // Not used in this test, but available for interrupts

// LSM6DSL I2C addresses
#define LSM6DSL_ADDR1 0x6A
#define LSM6DSL_ADDR2 0x6B
uint8_t lsm6dsl_address = LSM6DSL_ADDR1;

// LSM6DSL Register addresses
#define LSM6DSL_WHO_AM_I 0x0F
#define LSM6DSL_CTRL1_XL 0x10
#define LSM6DSL_CTRL3_C 0x12
#define LSM6DSL_OUTX_L_XL 0x28

// Variables for accelerometer data
float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
bool lsm6dsl_ok = false;
bool tiltDetected = false;
bool fallDetected = false; // New flag for fall detection

// Timing
unsigned long lastReading = 0;
const unsigned long READING_INTERVAL = 100; // Increased reading frequency to 100ms for better fall detection

// Thresholds for detection
// TILT_THRESHOLD: Z-axis acceleration threshold. If Z-axis accel drops below this
// (e.g., when the sensor is tilted 90 degrees, Z-axis acceleration approaches 0g).
// A small non-zero value like 0.1g is used to account for sensor noise and minor variations.
const float TILT_THRESHOLD_Z_AXIS = 0.1; // g (if Z-axis accel is below this, consider tilted 90 degrees)

// FALL_THRESHOLD: Magnitude of total acceleration for freefall detection.
// In freefall, the sensor experiences close to 0g, so the magnitude of the
// acceleration vector (sqrt(X^2 + Y^2 + Z^2)) should be very low.
// Your provided FALL_THRESHOLD = 1.1 is close to 1g (normal gravity),
// which is not indicative of freefall. A value close to 0 is required.
const float FALL_THRESHOLD_MAGNITUDE = 0.2; // g (if total accel magnitude is below this, consider falling)


void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("======================================");
  Serial.println("   LSM6DSL ACCELEROMETER TEST");
  Serial.println("======================================");
  Serial.println("Pin Config: SDA=21, SCL=22, INT1=34");
  Serial.println("");
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100kHz
  Serial.println("I2C initialized");
  
  // Scan for I2C devices
  scanI2C();
  
  // Initialize LSM6DSL
  Serial.println("Initializing LSM6DSL...");
  if (initLSM6DSL()) {
    lsm6dsl_ok = true;
    Serial.print("✓ LSM6DSL initialized successfully at address 0x");
    Serial.println(lsm6dsl_address, HEX);
  } else {
    Serial.println("✗ LSM6DSL initialization failed");
  }
  
  Serial.println("");
  Serial.println("Starting sensor readings...");
  Serial.println("Format: Accel X,Y,Z | Magnitude | Tilt Status | Fall Status");
  Serial.println("=====================================");
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastReading >= READING_INTERVAL) {
    lastReading = currentTime;
    
    // Read accelerometer data
    readLSM6DSL();
    
    // Check for tilt and fall conditions
    checkTilt();
    checkFall(); // New function call
    
    // Display readings
    displayReadings();
  }
}

void readLSM6DSL() {
  if (lsm6dsl_ok) {
    // Read raw accelerometer data from registers
    uint8_t rawData[6];
    for (int i = 0; i < 6; i++) {
      rawData[i] = readLSM6DSLRegister(LSM6DSL_OUTX_L_XL + i);
    }
    
    // Convert to signed 16-bit values (low byte | high byte << 8)
    int16_t rawX = (rawData[1] << 8) | rawData[0];
    int16_t rawY = (rawData[3] << 8) | rawData[2];
    int16_t rawZ = (rawData[5] << 8) | rawData[4];
    
    // Convert to 'g' values.
    // For a ±2g full-scale range (configured in CTRL1_XL), the sensitivity is
    // 0.061 mg/LSB. 1 mg = 0.001 g, so 0.061 mg/LSB = 0.000061 g/LSB.
    // Alternatively, for ±2g, the full range is 4g (from -2g to +2g) represented by 65536 LSBs (2^16).
    // So, 1 LSB = 4g / 65536 LSB = 0.000061035 g/LSB.
    // To get 'g' from raw reading, it's (raw_value * 2.0) / 32768.0 for a +/-2g scale.
    accelX = (float)rawX * 2.0 / 32768.0;
    accelY = (float)rawY * 2.0 / 32768.0;
    accelZ = (float)rawZ * 2.0 / 32768.0;
  } else {
    accelX = accelY = accelZ = 0.0;
  }
}

void checkTilt() {
  bool newTiltState = false;
  
  if (lsm6dsl_ok) {
    // A simplified tilt detection: if the Z-axis acceleration is close to 0g,
    // it implies the sensor is oriented roughly 90 degrees relative to gravity along X or Y axis.
    if (accelZ < TILT_THRESHOLD_Z_AXIS) { // Using fabs for absolute value
      newTiltState = true;
    }
  }
  
  // Detect tilt state changes and print messages
  if (newTiltState && !tiltDetected) {
    Serial.println("🚨 TILT DETECTED! 🚨");
  } else if (!newTiltState && tiltDetected) {
    Serial.println("✅ Tilt cleared");
  }
  
  tiltDetected = newTiltState; // Update the global tilt flag
}

void checkFall() {
  bool newFallState = false;
  
  if (lsm6dsl_ok) {
    // Calculate the magnitude of the acceleration vector.
    // In freefall, the apparent acceleration on all axes should be close to 0g.
    float accelerationMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    
    if (accelerationMagnitude < FALL_THRESHOLD_MAGNITUDE) {
      newFallState = true;
    }
  }
  
  // Detect fall state changes and print messages
  if (newFallState && !fallDetected) {
    Serial.println("💥 FALL DETECTED! (Freefall) 💥");
  } else if (!newFallState && fallDetected) {
    Serial.println("⬆️ Fall recovered");
  }
  
  fallDetected = newFallState; // Update the global fall flag
}

void displayReadings() {
  // Accelerometer readings
  if (lsm6dsl_ok) {
    Serial.print("Accel: ");
    Serial.print("X:");
    Serial.print(accelX, 2);
    Serial.print(" Y:");
    Serial.print(accelY, 2);
    Serial.print(" Z:");
    Serial.print(accelZ, 2);
    Serial.print("g");

    // Calculate and display magnitude
    float accelerationMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    Serial.print(" | Mag: ");
    Serial.print(accelerationMagnitude, 2);
    Serial.print("g");
  } else {
    Serial.print("Accel: ERROR | Mag: ERROR");
  }
  
  Serial.print(" | ");
  
  // Tilt status
  if (tiltDetected) {
    Serial.print("Tilt: YES");
  } else {
    Serial.print("Tilt: NO");
  }

  Serial.print(" | ");

  // Fall status
  if (fallDetected) {
    Serial.print("Fall: YES");
  } else {
    Serial.print("Fall: NO");
  }
  
  Serial.println("");
}

bool initLSM6DSL() {
  // Try both possible I2C addresses (0x6A and 0x6B)
  uint8_t addresses[] = {LSM6DSL_ADDR1, LSM6DSL_ADDR2};
  
  for (int i = 0; i < 2; i++) {
    lsm6dsl_address = addresses[i];
    
    // Read WHO_AM_I register to verify sensor presence (should be 0x6A)
    uint8_t whoAmI = readLSM6DSLRegister(LSM6DSL_WHO_AM_I);
    
    if (whoAmI == 0x6A) { // LSM6DSL's WHO_AM_I value is 0x6A
      // Configure accelerometer:
      // CTRL1_XL: ODR (Output Data Rate) and FS (Full Scale)
      // ODR_XL[3:0] = 0110 (208 Hz) - good for responsiveness
      // FS_XL[1:0] = 00 (+/-2g) - common and provides good resolution
      writeLSM6DSLRegister(LSM6DSL_CTRL1_XL, 0x60); 

      // CTRL3_C: Common control register
      // BDU (Block Data Update) = 1 (0x40): Ensures data is updated only after all 6 bytes are read, preventing reading inconsistent data.
      writeLSM6DSLRegister(LSM6DSL_CTRL3_C, 0x40); 
      
      delay(100); // Give sensor a moment to stabilize after configuration
      return true; // Sensor initialized successfully
    }
  }
  
  return false; // Sensor not found or WHO_AM_I mismatch
}

uint8_t readLSM6DSLRegister(uint8_t reg) {
  Wire.beginTransmission(lsm6dsl_address); // Start I2C transmission to sensor address
  Wire.write(reg);                         // Send register address to read from
  uint8_t error = Wire.endTransmission(false); // End transmission, but keep connection active (send restart condition)
  
  if (error != 0) return 0xFF; // Return 0xFF (error) if transmission failed
  
  Wire.requestFrom(lsm6dsl_address, (uint8_t)1); // Request 1 byte from the sensor
  if (Wire.available() < 1) return 0xFF;       // Return 0xFF if no byte received
  
  return Wire.read(); // Read and return the received byte
}

void writeLSM6DSLRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(lsm6dsl_address); // Start I2C transmission to sensor address
  Wire.write(reg);                         // Send register address to write to
  Wire.write(value);                       // Send the value to write
  Wire.endTransmission();                  // End transmission (send stop condition)
}

void scanI2C() {
  Serial.println("Scanning I2C devices...");
  int deviceCount = 0;
  
  // Iterate through all possible 7-bit I2C addresses
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address); // Begin transmission to current address
    uint8_t error = Wire.endTransmission(); // End transmission to check for ACK from device
    
    if (error == 0) { // If error is 0, device acknowledged (found)
      Serial.print("Found device at 0x");
      if (address < 16) Serial.print("0"); // Add leading zero for single-digit hex
      Serial.print(address, HEX); // Print address in hexadecimal
      
      // Identify known devices by their common I2C addresses
      if (address == 0x6A) Serial.print(" (LSM6DSL)");
      if (address == 0x6B) Serial.print(" (LSM6DSL Alt)");
      
      Serial.println(""); // New line for next device
      deviceCount++; // Increment count of found devices
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.print("Total devices found: ");
    Serial.println(deviceCount);
  }
  Serial.println(""); // Blank line for formatting
}
