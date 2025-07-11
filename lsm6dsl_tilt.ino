// lsm6dsl_tilt.ino
// Basic sketch to read acceleration & gyroscope data from the LSM6DSL (6-DoF IMU)
// and compute simple tilt angles (pitch & roll). Output is sent to the Serial Monitor.
//
// Connection (I2C):
//   LSM6DSL SDA -> Arduino SDA (A4 on UNO)
//   LSM6DSL SCL -> Arduino SCL (A5 on UNO)
//   LSM6DSL GND -> Arduino GND
//   LSM6DSL VCC -> 3V3 (preferred) or 5V (check your breakout)
//   LSM6DSL SDO -> GND (gives I2C address 0x6A) or VCC (address 0x6B)
// NOTE: Use pull-up resistors (4.7kΩ typical) on SDA/SCL if your breakout doesn't have them.

#include <Wire.h>

// ===== ESP32 I²C pin mapping (change if you use non-default pins) =====
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 21   // default ESP32 SDA
#endif

#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 22   // default ESP32 SCL
#endif

// Two possible I²C addresses depending on the SA0/SDO pin
#define LSM6DSL_ADDR1 0x6A // SA0 low
#define LSM6DSL_ADDR2 0x6B // SA0 high

// The code will probe both and set this variable to the one that replies
uint8_t lsm6dsl_addr = LSM6DSL_ADDR1;

// Register addresses (from datasheet)
#define REG_FUNC_CFG_ACCESS 0x01
#define REG_WHO_AM_I       0x0F
#define REG_CTRL1_XL       0x10
#define REG_CTRL2_G        0x11
#define REG_CTRL3_C        0x12
#define REG_CTRL8_XL       0x17
#define REG_OUTX_L_G       0x22  // first gyro register (auto-increment follows)
#define REG_OUTX_L_XL      0x28  // first accel register (auto-increment follows)

// Sensitivity factors (datasheet)
// FS_XL = ±2 g  -> 0.061 mg/LSB
// FS_G  = ±250 dps -> 8.75 mdps/LSB
const float ACC_SENS   = 0.061f / 1000.0f;   // g per LSB
const float GYRO_SENS  = 8.75f  / 1000.0f;   // dps per LSB

// Variables for timing
unsigned long lastMicros = 0;
float pitch = 0.0f, roll = 0.0f; // complementary-filter angles (deg)

// Write a single byte to a register
void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(lsm6dsl_addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Read multiple bytes starting at reg into buffer
void readBytes(uint8_t reg, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(lsm6dsl_addr);
  Wire.write(reg | 0x80); // set auto-increment bit (MSB)
  Wire.endTransmission(false); // restart for read
  Wire.requestFrom(lsm6dsl_addr, count);
  for (uint8_t i = 0; i < count && Wire.available(); i++) {
    dest[i] = Wire.read();
  }
}

void setup() {
  Serial.begin(115200);
  // ESP32 instantly provides Serial, no need for while (!Serial)

  // Start I²C using explicit ESP32 pins
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000); // 400 kHz for faster reads

  // Probe possible I²C addresses (0x6A or 0x6B)
  uint8_t possible_addresses[] = {LSM6DSL_ADDR1, LSM6DSL_ADDR2};
  uint8_t who = 0;
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t addr = possible_addresses[i];
    Wire.beginTransmission(addr);
    Wire.write(REG_WHO_AM_I);
    if (Wire.endTransmission(false) == 0) {           // ping OK
      Wire.requestFrom(addr, (uint8_t)1);
      if (Wire.available()) {
        who = Wire.read();
        if (who == 0x6A) {  // correct device ID
          lsm6dsl_addr = addr;
          break;
        }
      }
    }
  }

  Serial.print("Found LSM6DSL at 0x"); Serial.print(lsm6dsl_addr, HEX);
  Serial.print("  WHO_AM_I = 0x"); Serial.println(who, HEX);

  // Reset device & enable auto-increment
  writeReg(REG_CTRL3_C, 0x44); // IF_INC=1 (bit2), SW_RESET=1 (bit0) — reset will clear after 1ms
  delay(10);
  writeReg(REG_CTRL3_C, 0x04);  // IF_INC=1

  // Accelerometer: 104 Hz (ODR=0100), 2 g full-scale (FS=00)
  writeReg(REG_CTRL1_XL, 0x40);

  // Gyroscope: 104 Hz, 250 dps full-scale
  writeReg(REG_CTRL2_G, 0x40);

  // Optional: improve accel filtering (high-performance mode, 400 Hz filter cutoff)
  writeReg(REG_CTRL8_XL, 0x09);

  lastMicros = micros();
}

void loop() {
  uint8_t rawData[12];

  // Read gyro (6) + accel (6) bytes in two separate transactions for clarity
  readBytes(REG_OUTX_L_G, 6, rawData);
  int16_t gx = (int16_t)(rawData[1] << 8 | rawData[0]);
  int16_t gy = (int16_t)(rawData[3] << 8 | rawData[2]);
  int16_t gz = (int16_t)(rawData[5] << 8 | rawData[4]);

  readBytes(REG_OUTX_L_XL, 6, rawData);
  int16_t ax = (int16_t)(rawData[1] << 8 | rawData[0]);
  int16_t ay = (int16_t)(rawData[3] << 8 | rawData[2]);
  int16_t az = (int16_t)(rawData[5] << 8 | rawData[4]);

  // Convert to physical units
  float gxf = gx * GYRO_SENS; // dps
  float gyf = gy * GYRO_SENS;
  float gzf = gz * GYRO_SENS;

  float axf = ax * ACC_SENS;  // g
  float ayf = ay * ACC_SENS;
  float azf = az * ACC_SENS;

  // Calculate accelerometer angles (deg)
  float accPitch = atan2(ayf, sqrt(axf*axf + azf*azf)) * 180.0f / PI;
  float accRoll  = atan2(-axf, azf) * 180.0f / PI;

  // Complementary filter to fuse gyro + accel
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f; // seconds
  lastMicros = now;

  // Integrate gyro to get angles
  pitch += gxf * dt;
  roll  += gyf * dt;

  // Apply complementary filter (98% gyro, 2% accel)
  pitch = pitch * 0.98f + accPitch * 0.02f;
  roll  = roll  * 0.98f + accRoll  * 0.02f;

  // Output to Serial Monitor
  Serial.print("Acc (g): ");
  Serial.print(axf, 3); Serial.print(", ");
  Serial.print(ayf, 3); Serial.print(", ");
  Serial.print(azf, 3); Serial.print(" | Gyro (dps): ");
  Serial.print(gxf, 2); Serial.print(", ");
  Serial.print(gyf, 2); Serial.print(", ");
  Serial.print(gzf, 2); Serial.print(" | Tilt -> Pitch: ");
  Serial.print(pitch, 1); Serial.print(" deg, Roll: ");
  Serial.print(roll, 1);
  Serial.println(" deg");

  delay(10); // ~100 Hz output
} 