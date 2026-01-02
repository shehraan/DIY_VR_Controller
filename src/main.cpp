#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <I2CHelpers.h>
//Avoided using #include <cmath> for pow() cuz it's slower and heavy

//Define GPIO pins
#define SDA_PIN 18
#define SCL_PIN 4

// IMU-Specific registers and addresses
#define IMU_ADDRESS 0x68
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

// Full-scale select values (bits [4:3])
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
float accelLsbPerG; 
float gyroLsbPerDps;
                                  
// Complementary fusion values
float pitch, yaw, roll; // Pitch and Roll from accelerometer and gyro fusion, yaw from just gyro
float alpha = 0.99; // Alpha value for complementary filter. Higher means more reliance on gyroscope vs. accelerometer
bool firstFilterUpdate = true;

//Declare additional variables for delta time calculation
uint32_t prevTime;
uint32_t currTime;
float deltaTime;
bool firstIMURead = true; // Flag to indicate if it's the first IMU reading, used for dt calculation initialization
                          
// Quarternion variables
float qW;
float qX;
float qY;
float qZ;

//Function Declarations
void getSensors();
bool dataAvailable();

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // Initializes the I2c controller on the ESP32 and sets pins as open-drain outputs.
  Wire.setClock(400000); // Use I2C in fast mode
  
  writeRegisters(IMU_ADDRESS, PWR_MGMT_1, 0); // Disable sleep mode by overwriting enabled (0x40)

  // Use finer ranges during bias collection for better sensitivity.
  writeRegisters(IMU_ADDRESS, ACCEL_CONFIG, ACCEL_FS_2G);
  writeRegisters(IMU_ADDRESS, GYRO_CONFIG, GYRO_FS_250DPS);
  accelLsbPerG = ACCEL_LSB_PER_G_2G;
  gyroLsbPerDps = GYRO_LSB_PER_DPS_250;
  
  delay(100); // Short delay to allow IMU to process configuration changes
  for (int j = 0; j < 500; j++) {
    getSensors(); // Get large sample of sensor data so it stabilizes and you get a good estimate of bias
    
    // Accumulate raw values for bias calculation
    a_bias_x += ax_g; 
    a_bias_y += ay_g;
    a_bias_z += az_g;
    g_bias_x += gx_dps;
    g_bias_y += gy_dps;
    g_bias_z += gz_dps;
    
    delay(20); // Short delay between readings
  }

  // compute average bias values once
  a_bias_x /= 500.0;
  a_bias_y /= 500.0;
  a_bias_z /= 500.0;
  g_bias_x /= 500.0;
  g_bias_y /= 500.0;
  g_bias_z /= 500.0;

  // Switch back to higher normal ranges after calibration.
  writeRegisters(IMU_ADDRESS, ACCEL_CONFIG, ACCEL_FS_8G); // Compromise between high precision like ±2g and high range like ±16g.
  writeRegisters(IMU_ADDRESS, GYRO_CONFIG, GYRO_FS_2000DPS); // Sets gyro to highest range.
  accelLsbPerG = ACCEL_LSB_PER_G_8G; 
  gyroLsbPerDps = GYRO_LSB_PER_DPS_2000;
  
  a_bias_z -= 1.0; //Assumes IMU is flat horizontally. Adjust based on orientation.

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

  firstIMURead = true; //Reset deltatime since we'll need it in loop
  delay(5000); 
}

void loop() { //Reading sensors loop

  getSensors(); // Read sensors and print values to Serial Monitor
             
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
    firstFilterUpdate = false;
  } else {
    // Integrate gyro rates over deltaTime, then blend with accel angle to limit drift.
    pitch = alpha * (pitch + gy_dps * deltaTime) + (1 - alpha) * pitch_acc;
    roll = alpha * (roll + gx_dps * deltaTime) + (1 - alpha) * roll_acc;
  }
  
  // Yaw has no accelerometer reference, so it is pure gyro integration.
  yaw = gz_dps * deltaTime; 

  //delay(1000);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}

// Function to get sensor data and start timer
void getSensors() {
  
  uint8_t raw[14];
  if (!readRegisters(IMU_ADDRESS, ACCEL_OUT, raw, 14)) return; // Read all the IMU sensor data into raw array

  /* Convert raw accelerometer values to g using the active LSB-per-g sensitivity. */
  ax_g = (int16_t)(raw[0]  << 8 | raw[1])  / accelLsbPerG;
  ay_g = (int16_t)(raw[2]  << 8 | raw[3])  / accelLsbPerG;
  az_g = (int16_t)(raw[4]  << 8 | raw[5])  / accelLsbPerG;
  // raw[6] and raw[7] are temperature. Not necessary for this so we skip.
  /* Convert raw gyroscope values to degrees/second. */
  gx_dps = (int16_t)(raw[8]  << 8 | raw[9])  / gyroLsbPerDps;
  gy_dps = (int16_t)(raw[10] << 8 | raw[11]) / gyroLsbPerDps;
  gz_dps = (int16_t)(raw[12] << 8 | raw[13]) / gyroLsbPerDps;

  // Start timer to compute deltaTime with.
  currTime = micros();
  if (firstIMURead) 
  { 
    // First sample has no previous timestamp, so force dt to 0.
    prevTime = currTime; 
    deltaTime = 0.0f; 
    firstIMURead = false; 
  }
  else { 
    // Convert microsecond difference to seconds
    deltaTime = (currTime - prevTime) / 1000000.0f; 
    prevTime = currTime; 
  }
}

bool dataAvailable() {
  uint8_t status;
  if (!readRegisters(IMU_ADDRESS, INT_STATUS, &status)) {
    return false;
  }
  return (status & 0x01) != 0;
}