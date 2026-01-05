#include <Arduino.h>
//#include <math.h>
#include <Wire.h>
#include <MadgwickFilter.h>
#include <I2CHelpers.h>
//Avoided using #include <cmath> for pow() cuz it's slower and heavy

//Define GPIO pins
#define SDA_PIN 18
#define SCL_PIN 4

// IMU-Specific registers and addresses
#define IMU_ADDRESS 0x68
#define SMPLRT_DIV 0x19
#define MPU_CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define FIFO_EN 0x23
#define I2C_MST_CTRL 0x24
#define INT_STATUS 0x3A
#define MPU6050_INT_PIN_CFG 0x37 // Configures interrupt behavior
#define MPU6050_INT_ENABLE 0x38
#define ACCEL_OUT 0x3B // First register in the 14bytes of accelerometer, temperature and gyro values that appear sequentially
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B // Sleep register
#define FIFO_COUNT 0x72
#define FIFO_R_W 0x74

// Full-scale select values (bits [4-3])
#define ACCEL_FS_2G 0x00
#define ACCEL_FS_8G 0x10
#define GYRO_FS_250DPS 0x00
#define GYRO_FS_2000DPS 0x18

// MPU6050 sensitivity conversion factors (LSB per physical unit)
#define ACCEL_LSB_PER_G_2G 16384.0f
#define ACCEL_LSB_PER_G_8G 4096.0f

#define GYRO_LSB_PER_DPS_250 131.0f
#define GYRO_LSB_PER_DPS_2000 16.4f

// Declare signed integer values that we will read. 
int16_t ax, ay, az;
int16_t temp;
int16_t gx, gy, gz;

//Declare converted float variables
float ax_g, ay_g, az_g, a_magm;
float a_bias_x, a_bias_y, a_bias_z; // per-axis accelerometer bias
float pitch_acc, roll_acc; // Angle estimates from accelerometer alone, in degrees. These are noisy but don't drift over time like the gyroscopes, so we can combine them in filter for ultimate accuracy
float temp_c;
float gx_dps, gy_dps, gz_dps, g_mag;
float g_bias_x, g_bias_y, g_bias_z; // per-axis gyroscope bias

// Runtime conversion factors in LSB per unit
float accelLsbPerG = ACCEL_LSB_PER_G_8G;     // default: +-8g
float gyroLsbPerDps = GYRO_LSB_PER_DPS_2000; // default: +-2000 dps
                                  
// Complementary fusion values
float pitch, yaw, roll; // Pitch and Roll from accelerometer and gyro fusion, yaw from just gyro
float alpha = 0.99; // Alpha value for complementary filter. Higher means more reliance on gyroscope vs. accelerometer
bool firstFilterUpdate = true;

//Declare additional variables for delta time calculation
uint32_t prevTime;
uint32_t currTime;
float deltaTime;
                          
// Quarternion variables
float qW;
float qX;
float qY;
float qZ;

MadgwickFilter madgwickFilter;

//Function Declarations
bool calibrateBiasFIFO();
bool getSensors();

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // Initializes the I2c controller on the ESP32 and sets pins as open-drain outputs.
  Wire.setClock(400000); // Use I2C in fast mode
  
  if (!calibrateBiasFIFO()) {
    Serial.println("FIFO bias calibration failed; using zero biases.");
    a_bias_x = a_bias_y = a_bias_z = 0.0f;
    g_bias_x = g_bias_y = g_bias_z = 0.0f;
  }

  // Switch back to higher normal modes and ranges after calibration.
  writeRegisters(IMU_ADDRESS, PWR_MGMT_1, 0x03); // Re-lock PLL to X-Gyro for stable timing reference
  writeRegisters(IMU_ADDRESS, SMPLRT_DIV, 0x03); // Drop sample rate to 250Hz (optional, but matches library)
  writeRegisters(IMU_ADDRESS, MPU_CONFIG, 0x03); // Switch filter to 44Hz (cleaner for runtime complementary filter)//
  
  writeRegisters(IMU_ADDRESS, ACCEL_CONFIG, ACCEL_FS_8G);
  writeRegisters(IMU_ADDRESS, GYRO_CONFIG, GYRO_FS_2000DPS);
  accelLsbPerG = ACCEL_LSB_PER_G_8G;
  gyroLsbPerDps = GYRO_LSB_PER_DPS_2000;
  
//  a_bias_z -= 1.0; 

  //Print biases
  Serial.print("\nAccel biases: x=");
  Serial.print(a_bias_x);
  Serial.print(" y=");
  Serial.print(a_bias_y);
  Serial.print(" z=");
  Serial.println(a_bias_z);
  Serial.print("Gyro biases: x=");
  Serial.print(g_bias_x);
  Serial.print(" y=");
  Serial.print(g_bias_y);
  Serial.print(" z=");
  Serial.println(g_bias_z);

  madgwickFilter.begin(0.10f);
  delay(5000);
  prevTime = micros(); // Seed timer so first loop gets a valid dt
}

void loop() { //Reading sensors loop

  if (!getSensors()) { // Get sensor values
    return;
  }
             
  //Correct for bias by subtracting the average bias calculated during setup from each measurement. This helps to improve accuracy by accounting for any consistent offset in the sensor readings.
  ax_g -= a_bias_x;
  ay_g -= a_bias_y;
  az_g -= a_bias_z;
  gx_dps -= g_bias_x;
  gy_dps -= g_bias_y;
  gz_dps -= g_bias_z;

  // Elapsed time between IMU samples in seconds. Used for gyro integration.
  Serial.print("dt: ");
  Serial.println(deltaTime, 6);
  
  // Print corrected (bias‑subtracted) values
  Serial.print("Corrected ax:"); Serial.print(ax_g);
  Serial.print(" ay:"); Serial.print(ay_g);
  Serial.print(" az:"); Serial.println(az_g);
  Serial.print("a_mag:"); Serial.print(sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g));
  Serial.print("\nCorrected gx:"); Serial.print(gx_dps);
  Serial.print(" gy:"); Serial.print(gy_dps);
  Serial.print(" gz:"); Serial.println(gz_dps);
  Serial.print(" g_mag:"); Serial.println(sqrt(gx_dps*gx_dps + gy_dps*gy_dps + gz_dps*gz_dps));

  //Inverse tangent of accel values for pitch and roll estimates
  pitch_acc = atan2(ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI; // inverse tan of (gravity along axis / perpendicular). Converts radians to degrees by multiplying by 180/PI.
  roll_acc  = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * 180.0 / PI;
 
  // Use accelerometer angle as initial estimate for complementary filter since gyro isn't an absolute value.
  if (firstFilterUpdate) {
    pitch = pitch_acc; 
    roll = roll_acc;
    yaw = 0.0f;
    firstFilterUpdate = false;
  } else {
    // Integrate gyro rates over deltaTime, then blend with accel angle to limit drift.
    pitch = alpha * (pitch + gy_dps * deltaTime) + (1 - alpha) * pitch_acc;
    roll = alpha * (roll + gx_dps * deltaTime) + (1 - alpha) * roll_acc;
    // Yaw has no accelerometer reference, so it is pure gyro integration.
    yaw += gz_dps * deltaTime; 
    if (deltaTime > 0.0f) {
      madgwickFilter.updateIMU(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, deltaTime);
    }
  } 

  Serial.print("Madgwick roll:"); Serial.print(madgwickFilter.getRoll());
  Serial.print(" pitch:"); Serial.print(madgwickFilter.getPitch());
  Serial.print(" yaw:"); Serial.println(madgwickFilter.getYaw());

  //delay(1000);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}

// Function to get sensor data and start timer
bool getSensors() {
  currTime = micros();
  deltaTime = (currTime - prevTime) / 1000000.0f;
  prevTime = currTime;
  
  uint8_t raw[14];
  if (!readRegisters(IMU_ADDRESS, ACCEL_OUT, raw, 14)) return false; // Read all the IMU sensor data into raw array

  /* Convert raw accelerometer values to g using the active LSB-per-g sensitivity. */
  ax_g = (int16_t)(raw[0]  << 8 | raw[1])  / accelLsbPerG;
  ay_g = (int16_t)(raw[2]  << 8 | raw[3])  / accelLsbPerG;
  az_g = (int16_t)(raw[4]  << 8 | raw[5])  / accelLsbPerG;
  // raw[6] and raw[7] are temperature. Not necessary for this so we skip.
  /* Convert raw gyroscope values to degrees/second. */
  gx_dps = (int16_t)(raw[8]  << 8 | raw[9])  / gyroLsbPerDps;
  gy_dps = (int16_t)(raw[10] << 8 | raw[11]) / gyroLsbPerDps;
  gz_dps = (int16_t)(raw[12] << 8 | raw[13]) / gyroLsbPerDps;

  return true;
}

bool calibrateBiasFIFO() {
  const uint8_t NUM_ROUNDS = 5; // 5 × 40 samples ≈ 200 total for strong noise reduction.
  uint8_t data[12];
  uint8_t fifoCountRaw[2];
  uint32_t totalPackets = 0;
  int32_t accelAccum[3] = {0, 0, 0}; 
  int32_t gyroAccum[3]  = {0, 0, 0};

  a_bias_x = a_bias_y = a_bias_z = 0.0f;
  g_bias_x = g_bias_y = g_bias_z = 0.0f;

  writeRegisters(IMU_ADDRESS, PWR_MGMT_1, 0x80); // Resets device and clears registers
  delay(100); // Time delay for hardware reset to take place

  writeRegisters(IMU_ADDRESS, PWR_MGMT_1,         0x00); // Disable sleep mode
  delay(20); // Small settlement time

  writeRegisters(IMU_ADDRESS, MPU_CONFIG,    0x01); // Set to wide bandwidth for fast response.
  writeRegisters(IMU_ADDRESS, SMPLRT_DIV,    0x00); // Set sample rate to 1kHz for faster bias testing. Sample rate is gyro output rate (1kHz here) divided by (1 + SMPLRT_DIV) 
  writeRegisters(IMU_ADDRESS, GYRO_CONFIG,   GYRO_FS_250DPS);
  writeRegisters(IMU_ADDRESS, ACCEL_CONFIG,  ACCEL_FS_2G);

  for (uint8_t round = 0; round < NUM_ROUNDS; round++) {
    writeRegisters(IMU_ADDRESS, USER_CTRL, 0x44); // Reset + enable FIFO in one step
    writeRegisters(IMU_ADDRESS, FIFO_EN,   0x78); // Enables FIFO for X,Y,Z gyros (bits[6-3]) and Accel
    delay(40); // 4, which is close to max per FIFO of 

    writeRegisters(IMU_ADDRESS, FIFO_EN, 0x00);
    if (!readRegisters(IMU_ADDRESS, FIFO_COUNT, fifoCountRaw, 2)) return false;

    uint16_t fifoCount = ((uint16_t)fifoCountRaw[0] << 8) | fifoCountRaw[1];
    if (fifoCount >= 1024) return false;            // FIFO overflow = corrupted data
    
    uint16_t packetCount = fifoCount / 12; // 12 bytes per packet: 6 for accel, 6 for gyro.
    if (packetCount == 0) return false;
    totalPackets += packetCount;

    for (uint16_t i = 0; i < packetCount; i++) {
      if (!readRegisters(IMU_ADDRESS, FIFO_R_W, data, 12)) return false;

      // Accumulate the sensor data
      accelAccum[0] += (int16_t)(((int16_t)data[0] << 8) | data[1]);
      accelAccum[1] += (int16_t)(((int16_t)data[2] << 8) | data[3]);
      accelAccum[2] += (int16_t)(((int16_t)data[4] << 8) | data[5]);
      gyroAccum[0]  += (int16_t)(((int16_t)data[6] << 8) | data[7]);
      gyroAccum[1]  += (int16_t)(((int16_t)data[8] << 8) | data[9]);
      gyroAccum[2]  += (int16_t)(((int16_t)data[10] << 8) | data[11]);
    }
  }

  int32_t accelBias[3], gyroBias[3];
  for (int i = 0; i < 3; i++) {
    accelBias[i] = (int32_t)(accelAccum[i] / (int64_t)totalPackets);
    gyroBias[i]  = (int32_t)(gyroAccum[i]  / (int64_t)totalPackets);
  }

  //Assumes IMU is flat horizontally. Adjust based on orientation.
  if (accelBias[2] > 0) {
    accelBias[2] -= (int32_t)ACCEL_LSB_PER_G_2G; // Subtract gravity since it's 1g when flat.
  } else {
    accelBias[2] += (int32_t)ACCEL_LSB_PER_G_2G; // Add gravity if it's negative, which could happen if the IMU is flipped.
  }
  a_bias_x = (float)accelBias[0] / ACCEL_LSB_PER_G_2G;
  a_bias_y = (float)accelBias[1] / ACCEL_LSB_PER_G_2G;
  a_bias_z = (float)accelBias[2] / ACCEL_LSB_PER_G_2G;
  g_bias_x = (float)gyroBias[0]  / GYRO_LSB_PER_DPS_250;
  g_bias_y = (float)gyroBias[1]  / GYRO_LSB_PER_DPS_250;
  g_bias_z = (float)gyroBias[2]  / GYRO_LSB_PER_DPS_250;

  return true;
}