/*
 * =====================================================================
 * COMBINED SENSOR TEST - SHT31 + LSM6DSL
 * =====================================================================
 * 
 * Tests both temperature/humidity and accelerometer sensors together
 * 
 * Pin Configuration:
 * - SDA: GPIO21 (I2C Data)
 * - SCL: GPIO22 (I2C Clock) 
 * - INT1: GPIO34 (Accelerometer Interrupt - not used in this test)
 * 
 * Sensors:
 * - SHT31: Temperature & Humidity (I2C Address: 0x44)
 * - LSM6DSL: 6-axis Accelerometer/Gyroscope (I2C Address: 0x6A or 0x6B)
 * 
 * =====================================================================
 */

#include <Wire.h>
#include "Adafruit_SHT31.h"

// Pin Definitions
#define SDA_PIN 21
#define SCL_PIN 22
#define INT1_PIN 34  // Not used in this test, but available for interrupts

// LSM6DSL I2C addresses
#define LSM6DSL_ADDR1 0x6A
#define LSM6DSL_ADDR2 0x6B
uint8_t lsm6dsl_address = LSM6DSL_ADDR1;

// LSM6DSL Register addresses
#define LSM6DSL_WHO_AM_I 0x0F
#define LSM6DSL_CTRL1_XL 0x10
#define LSM6DSL_CTRL3_C 0x12
#define LSM6DSL_OUTX_L_XL 0x28

// Sensor objects
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// Variables
float temperature = 0.0;
float humidity = 0.0;
float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
bool sht31_ok = false;
bool lsm6dsl_ok = false;
bool tiltDetected = false;

// Timing
unsigned long lastReading = 0;
const unsigned long READING_INTERVAL = 1000; // 1 second
const float TILT_THRESHOLD = 0.0; // 0m/s2 on z axis if tilted 90deg
const float FALL_THRESHOLD = 1.1; //acceleration of falling

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("======================================");
  Serial.println("   COMBINED SENSOR TEST");
  Serial.println("   SHT31 + LSM6DSL");
  Serial.println("======================================");
  Serial.println("Pin Config: SDA=21, SCL=22, INT1=34");
  Serial.println("");
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100kHz
  Serial.println("I2C initialized");
  
  // Scan for I2C devices
  scanI2C();
  
  // Initialize SHT31
  Serial.println("Initializing SHT31...");
  if (sht31.begin(0x44)) {
    sht31_ok = true;
    Serial.println("✓ SHT31 initialized successfully");
  } else {
    Serial.println("✗ SHT31 initialization failed");
  }
  
  // Initialize LSM6DSL
  Serial.println("Initializing LSM6DSL...");
  if (initLSM6DSL()) {
    lsm6dsl_ok = true;
    Serial.print("✓ LSM6DSL initialized at address 0x");
    Serial.println(lsm6dsl_address, HEX);
  } else {
    Serial.println("✗ LSM6DSL initialization failed");
  }
  
  Serial.println("");
  Serial.println("Starting sensor readings...");
  Serial.println("Format: Temp | Humidity | Accel X,Y,Z | Tilt");
  Serial.println("=====================================");
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastReading >= READING_INTERVAL) {
    lastReading = currentTime;
    
    // Read all sensors
    readSHT31();
    readLSM6DSL();
    checkTilt();
    
    // Display readings
    displayReadings();
  }
}

void readSHT31() {
  if (sht31_ok) {
    temperature = sht31.readTemperature();
    humidity = sht31.readHumidity();
    
    // Check for invalid readings
    if (isnan(temperature)) temperature = -999;
    if (isnan(humidity)) humidity = -999;
  } else {
    temperature = -999;
    humidity = -999;
  }
}

void readLSM6DSL() {
  if (lsm6dsl_ok) {
    // Read raw accelerometer data
    uint8_t rawData[6];
    for (int i = 0; i < 6; i++) {
      rawData[i] = readLSM6DSLRegister(LSM6DSL_OUTX_L_XL + i);
    }
    
    // Convert to signed 16-bit values
    int16_t rawX = (rawData[1] << 8) | rawData[0];
    int16_t rawY = (rawData[3] << 8) | rawData[2];
    int16_t rawZ = (rawData[5] << 8) | rawData[4];
    
    // Convert to g values (±2g full scale)
    accelX = rawX * 2.0 / 32768.0;
    accelY = rawY * 2.0 / 32768.0;
    accelZ = rawZ * 2.0 / 32768.0;
  } else {
    accelX = accelY = accelZ = 0.0;
  }
}

void checkTilt() {
  bool newTiltState = false;
  
  if (lsm6dsl_ok) {
    // Check if any axis exceeds tilt threshold
    if (accelZ < TILT_THRESHOLD) {
      newTiltState = true;
    }
  }
  
  // Detect tilt state changes
  if (newTiltState && !tiltDetected) {
    Serial.println("🚨 TILT DETECTED! 🚨");
  } else if (!newTiltState && tiltDetected) {
    Serial.println("✅ Tilt cleared");
  }
  
  tiltDetected = newTiltState;
}

void displayReadings() {
  // Temperature
  if (temperature != -999) {
    Serial.print("Temp: ");
    Serial.print(temperature, 1);
    Serial.print("°C");
  } else {
    Serial.print("Temp: ERROR");
  }
  
  Serial.print(" | ");
  
  // Humidity
  if (humidity != -999) {
    Serial.print("Humidity: ");
    Serial.print(humidity, 0);
    Serial.print("%");
  } else {
    Serial.print("Humidity: ERROR");
  }
  
  Serial.print(" | ");
  
  // Accelerometer
  if (lsm6dsl_ok) {
    Serial.print("Accel: ");
    Serial.print("X:");
    Serial.print(accelX, 2);
    Serial.print(" Y:");
    Serial.print(accelY, 2);
    Serial.print(" Z:");
    Serial.print(accelZ, 2);
    Serial.print("g");
  } else {
    Serial.print("Accel: ERROR");
  }
  
  Serial.print(" | ");
  
  // Tilt status
  if (tiltDetected) {
    Serial.print("Tilt: YES");
  } else {
    Serial.print("Tilt: NO");
  }
  
  Serial.println("");
}

bool initLSM6DSL() {
  // Try both possible I2C addresses
  uint8_t addresses[] = {LSM6DSL_ADDR1, LSM6DSL_ADDR2};
  
  for (int i = 0; i < 2; i++) {
    lsm6dsl_address = addresses[i];
    
    // Check WHO_AM_I register
    uint8_t whoAmI = readLSM6DSLRegister(LSM6DSL_WHO_AM_I);
    
    if (whoAmI == 0x6A) {
      // Configure accelerometer: 208Hz, ±2g
      writeLSM6DSLRegister(LSM6DSL_CTRL1_XL, 0x60);
      // Enable block data update
      writeLSM6DSLRegister(LSM6DSL_CTRL3_C, 0x40);
      
      delay(100); // Allow sensor to stabilize
      return true;
    }
  }
  
  return false;
}

uint8_t readLSM6DSLRegister(uint8_t reg) {
  Wire.beginTransmission(lsm6dsl_address);
  Wire.write(reg);
  uint8_t error = Wire.endTransmission(false);
  
  if (error != 0) return 0xFF;
  
  Wire.requestFrom(lsm6dsl_address, (uint8_t)1);
  if (Wire.available() < 1) return 0xFF;
  
  return Wire.read();
}

void writeLSM6DSLRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(lsm6dsl_address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void scanI2C() {
  Serial.println("Scanning I2C devices...");
  int deviceCount = 0;
  
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Found device at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Identify known devices
      if (address == 0x44) Serial.print(" (SHT31)");
      if (address == 0x6A) Serial.print(" (LSM6DSL)");
      if (address == 0x6B) Serial.print(" (LSM6DSL Alt)");
      
      Serial.println("");
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.print("Total devices found: ");
    Serial.println(deviceCount);
  }
  Serial.println("");
} 