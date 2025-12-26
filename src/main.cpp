#include <Arduino.h>
#include <Wire.h>
//Avoided using #include <cmath> for pow() cuz it's slower and heavy

//Loop counter
int i = 0;

// Declare signed integer values that we will read. 
int16_t ax, ay, az;
int16_t temp;
int16_t gx, gy, gz;

//Declare converted float variables
float ax_g, ay_g, az_g, a_magm;
float a_bias_x = 0.0, a_bias_y = 0.0, a_bias_z = 0.0; // per-axis accelerometer bias (initialized to 0.0)
float temp_c;
float gx_dps, gy_dps, gz_dps, g_mag;
float g_bias_x = 0.0, g_bias_y = 0.0, g_bias_z = 0.0; // per-axis gyroscope bias (initialized to 0.0)

//Define GPIO pins
#define SDA_PIN 18
#define SCL_PIN 4

// IMU-Specific registers and addresses
#define IMU_ADDRESS 0x68
#define SLEEP_REGISTER 0x6B // PWR_MGMT_1 register
#define GYRO_CONFIG_REGISTER 0x1B
#define ACCEL_CONFIG_REGISTER 0x1C
#define ACCEL_REGISTER 0x3B //First register in the 14bytes of accelerometer, temperature and gyro values that appear sequentially

// Reading sensors function
void readIMU() {
  
};

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // Initializes the I2c controller on the ESP32 and sets pins as open-drain outputs.

  Wire.beginTransmission(IMU_ADDRESS); //Device address to start transmission
  Wire.write(SLEEP_REGISTER); //Register address to write from
  Wire.write(0); // Disable sleep mode by overwriting enabled (0x40)
  Wire.endTransmission(); // Future additions go to consecutive registers and that one register write is complete, so we end transmission for now.

  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(ACCEL_CONFIG_REGISTER);
  Wire.write(0x10); // Sets acceleration measurement range to ±8g, compromise between high precision like ±2g and high range like ±16g. Range is selected by bits 3-4 in the configuration register. 
  Wire.endTransmission();

  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(GYRO_CONFIG_REGISTER);
  Wire.write(0x18); // Sets gyroscope measurement range to ±2000°/s
  Wire.endTransmission(); // Stop assumed true when brackets are empty
  
  delay(100); // Short delay to allow IMU to process configuration changes
  for (int j = 0; j < 500; j++) { 
    delay(10); // Delay between readings during stabilization
    
  }
}

void loop() { //Reading sensors loop
                         
  //Setup transmission
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(ACCEL_REGISTER); //Register address to read from 
  Wire.endTransmission(false); //endTransmission causes library to: send the START condition, sends the address, sends all buffered bytes. Here, adding "false" param means it does a repeated start instead of stopping (so the register pointer won't get reset and the IMU can expect to be read from now.), but still do everything else.
  Wire.requestFrom(IMU_ADDRESS, 14); 
  
  //I2C transfers 8 bits (1 byte) at a time so each measurement (16 bits total) is split into 2, which we must combine ourselves. 
  uint8_t axHigh = Wire.read();
  uint8_t axLow = Wire.read();
  ax = (axHigh << 8) | axLow; // Combine high byte with low byte
  ax_g = ax / 4096.0; // Converts raw to actual g value. Since accelerometer output is 16-bit signed, values go from: -32768 → +32767, with the ends representing ±8g.
  Serial.print("\nax: "); Serial.print(ax_g);

  uint8_t ayHigh = Wire.read();
  uint8_t ayLow = Wire.read();
  ay = (ayHigh << 8) | ayLow; // Combine high byte with low byte
  ay_g = ay / 4096.0; // Converts raw to actual g value. 
  Serial.print(" ay: "); Serial.print(ay_g);

  uint8_t azHigh = Wire.read();
  uint8_t azLow = Wire.read();
  az = (azHigh << 8) | azLow; // Combine high byte with low byte
  az_g = az / 4096.0; // Converts raw to actual g value. 
  Serial.print(" az: "); Serial.print(az_g);
  
  //Calculate magnitude of acceleration vector, which is useful for detecting events like impacts or free falls.
  float a_mag = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  Serial.print(" | a_mag: "); Serial.println(a_mag); 

  uint8_t tempHigh = Wire.read();
  uint8_t tempLow = Wire.read();
  temp = (tempHigh << 8) | tempLow; // Combine high byte with low byte
  temp_c = (temp/340.0) + 36.53; // Converts raw to real temp
  Serial.print("temp: "); Serial.println(temp_c);

  uint8_t gxHigh = Wire.read();
  uint8_t gxLow = Wire.read();
  gx = (gxHigh << 8) | gxLow; // Combine high byte with low byte
  gx_dps = gx / 16.384; // Converts raw to actual rotation value
  Serial.print("gx: "); Serial.print(gx_dps);

  uint8_t gyHigh = Wire.read();
  uint8_t gyLow = Wire.read();
  gy = (gyHigh << 8) | gyLow; // Combine high byte with low byte
  gy_dps = gy / 16.384; // Converts raw to actual rotation value
  Serial.print(" gy: "); Serial.print(gy_dps);

  uint8_t gzHigh = Wire.read();
  uint8_t gzLow = Wire.read();
  gz = (gzHigh << 8) | gzLow; // Combine high byte with low byte
  gz_dps = gz / 16.384; // Converts raw to actual rotation value
  Serial.print(" gz: "); Serial.println(gz_dps);

  delay(200);

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}