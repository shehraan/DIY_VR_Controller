#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "driver/ledc.h"
#include <MadgwickFilter.h>
#include <I2CHelpers.h>
#include <HIDTransport.h>

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

#define FILTER_MAX_BETA 0.15f
#define FILTER_MIN_BETA 0.015f
#define FILTER_DROPOFF 0.85f

#define CALIB_EEPROM_MAGIC 0xC411B1A5UL
#define CALIB_MAGIC_ADDR 0
#define CALIB_DATA_ADDR (CALIB_MAGIC_ADDR + sizeof(uint32_t))
#define MADGWICK_WARMUP_SAMPLES 500

// Define Joystick and Button pins
#define BUTTON_A 12
#define BUTTON_B 13
#define JOYSTICK_BUTTON 25
#define JOYSTICK_X 32
#define JOYSTICK_Y 33

// Declare signed integer values that we will read. 
//Declare converted float variables
float ax_g, ay_g, az_g;
float a_bias_x, a_bias_y, a_bias_z; // per-axis accelerometer bias
float pitch_acc, roll_acc; // Angle estimates from accelerometer alone, in degrees. These are noisy but don't drift over time like the gyroscopes, so we can combine them in filter for ultimate accuracy
float gx_dps, gy_dps, gz_dps;
float g_bias_x, g_bias_y, g_bias_z; // per-axis gyroscope bias

// Runtime conversion factors in LSB per unit
float accelLsbPerG = ACCEL_LSB_PER_G_8G;     // default: +-8g
float gyroLsbPerDps = GYRO_LSB_PER_DPS_2000; // default: +-2000 dps
                                  
// Complementary fusion values
float pitch, yaw, roll; // Pitch and Roll from accelerometer and gyro fusion, yaw from just gyro
float alpha = 0.99; // Alpha value for complementary filter. Higher means more reliance on gyroscope vs. accelerometer
bool firstFilterUpdate = true;
bool hidLinkWasReady = false;
uint32_t lastHidWaitPrintMs = 0;

//Declare additional variables for delta time calculation
uint32_t prevTime;
uint32_t currTime;
float deltaTime;

float gyroVelX;
float gyroVelY;
float gyroVelZ;

struct BiasCalibrationData {
  float accelBiasX;
  float accelBiasY;
  float accelBiasZ;
  float gyroBiasX;
  float gyroBiasY;
  float gyroBiasZ;
};

MadgwickFilter madgwickFilter;

namespace Haptics {

static constexpr uint8_t MOTOR_PIN = 14;
static constexpr uint8_t PWM_CHANNEL = 0;
static constexpr uint16_t PWM_FREQ_HZ = 200;
static constexpr uint8_t PWM_RES_BITS = 8;
static constexpr uint8_t DUTY_LOW = 170;
static constexpr uint8_t DUTY_MED = 220;
static constexpr uint8_t DUTY_HIGH = 255;
static constexpr size_t MAX_STEPS = 8;

enum class HapticClass : uint8_t {
  NONE = 0,
  CLICK,
  TAP, //Remove?
  PULSE,
  HEAVY_PULSE,
  SHORT_BUZZ,
  LONG_BUZZ,
  DOUBLE_CLICK,
  WARNING_BUZZ
};

enum class Strength : uint8_t {
  LEVEL_LOW = 0,
  LEVEL_MEDIUM,
  LEVEL_HIGH
};

struct HapticCommand {
  HapticClass effect;
  Strength strength;
  uint16_t sourceDurationMs;
  uint8_t sourceAmplitude;
};

struct HapticStep {
  uint8_t duty;
  uint16_t durationMs;
};

HapticStep steps[MAX_STEPS];
size_t stepCount = 0;
size_t stepIndex = 0;
bool active = false;
uint32_t stepStartMs = 0;

void motorWrite(uint8_t duty) {
  ledcWrite(PWM_CHANNEL, duty);
}

void stop() {
  motorWrite(0);
  active = false;
  stepCount = 0;
  stepIndex = 0;
}

uint8_t dutyForStrength(Strength strength) {
  switch (strength) {
    case Strength::LEVEL_LOW:
      return DUTY_LOW;
    case Strength::LEVEL_HIGH:
      return DUTY_HIGH;
    case Strength::LEVEL_MEDIUM:
    default:
      return DUTY_MED;
  }
}

Strength classifyStrength(uint8_t amplitude) {
  if (amplitude < 85) {
    return Strength::LEVEL_LOW;
  }
  if (amplitude < 170) {
    return Strength::LEVEL_MEDIUM;
  }
  return Strength::LEVEL_HIGH;
}

HapticCommand mapRumbleToClass(uint8_t amplitude, uint16_t durationMs) {
  HapticCommand cmd;
  cmd.effect = HapticClass::NONE;
  cmd.strength = classifyStrength(amplitude);
  cmd.sourceDurationMs = durationMs;
  cmd.sourceAmplitude = amplitude;

  if (durationMs == 0 || amplitude == 0) {
    return cmd;
  }

  if (durationMs <= 30) {
    cmd.effect = HapticClass::CLICK;
  } else if (durationMs <= 65) {
    cmd.effect = HapticClass::TAP;
  } else if (durationMs <= 120) {
    cmd.effect = (amplitude >= 180) ? HapticClass::HEAVY_PULSE : HapticClass::PULSE;
  } else if (durationMs <= 220) {
    cmd.effect = HapticClass::SHORT_BUZZ;
  } else {
    cmd.effect = HapticClass::LONG_BUZZ;
  }

  if (durationMs <= 45 && amplitude >= 220) {
    cmd.effect = HapticClass::DOUBLE_CLICK;
  }
  if (durationMs >= 350 && amplitude >= 200) {
    cmd.effect = HapticClass::WARNING_BUZZ;
  }
  return cmd;
}

void pushStep(uint8_t duty, uint16_t durationMs) {
  if (stepCount < MAX_STEPS) {
    steps[stepCount].duty = duty;
    steps[stepCount].durationMs = durationMs;
    stepCount++;
  }
}

void loadPattern(const HapticCommand &cmd) {
  stepCount = 0;
  stepIndex = 0;
  uint8_t baseDuty = dutyForStrength(cmd.strength);

  switch (cmd.effect) {
    case HapticClass::NONE:
      pushStep(0, 1);
      break;
    case HapticClass::CLICK:
      pushStep(baseDuty, 25);
      pushStep(0, 40);
      break;
    case HapticClass::TAP:
      pushStep(baseDuty, 50);
      pushStep(0, 50);
      break;
    case HapticClass::PULSE:
      pushStep(DUTY_MED, 90);
      pushStep(0, 60);
      break;
    case HapticClass::HEAVY_PULSE:
      pushStep(DUTY_HIGH, 120);
      pushStep(0, 70);
      break;
    case HapticClass::SHORT_BUZZ:
      pushStep(baseDuty, 170);
      pushStep(0, 80);
      break;
    case HapticClass::LONG_BUZZ:
      pushStep(baseDuty, 320);
      pushStep(0, 100);
      break;
    case HapticClass::DOUBLE_CLICK:
      pushStep(DUTY_HIGH, 25);
      pushStep(0, 70);
      pushStep(DUTY_HIGH, 25);
      pushStep(0, 80);
      break;
    case HapticClass::WARNING_BUZZ:
      pushStep(DUTY_HIGH, 120);
      pushStep(0, 70);
      pushStep(DUTY_HIGH, 120);
      pushStep(0, 70);
      pushStep(DUTY_HIGH, 120);
      pushStep(0, 100);
      break;
  }
}

void play(const HapticCommand &cmd) {
  loadPattern(cmd);
  if (stepCount == 0) {
    stop();
    return;
  }
  active = true;
  stepIndex = 0;
  stepStartMs = millis();
  motorWrite(steps[0].duty);
}

void begin() {
  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(MOTOR_PIN, PWM_CHANNEL);
  stop();
}

void onRumbleCommand(const HIDTransport::RumbleCommand &command) {
  play(mapRumbleToClass(command.amplitude, command.durationMs));
}

void update() {
  if (!active || stepCount == 0) {
    return;
  }

  uint32_t now = millis();
  if ((now - stepStartMs) >= steps[stepIndex].durationMs) {
    stepIndex++;
    if (stepIndex >= stepCount) {
      stop();
      return;
    }
    stepStartMs = now;
    motorWrite(steps[stepIndex].duty);
  }
}

} // namespace Haptics

//Function Declarations
bool calibrateBiasFIFO();
bool getSensors();
bool loadBiasesFromEEPROM();
void saveBiasesToEEPROM();
void applyAdaptiveBeta();
void runWarmupLoop();

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // Initializes the I2c controller on the ESP32 and sets pins as open-drain outputs.
  Wire.setClock(400000); // Use I2C in fast mode
  analogReadResolution(12);
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(JOYSTICK_BUTTON, INPUT_PULLUP);
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  Haptics::begin();
  HIDTransport::begin();
  Serial.println("HID transport initialized.");

  bool haveSavedCalibration = loadBiasesFromEEPROM();
  
  if (!haveSavedCalibration) {
    if (!calibrateBiasFIFO()) {
      Serial.println("FIFO bias calibration failed; using zero biases.");
      a_bias_x = a_bias_y = a_bias_z = 0.0f;
      g_bias_x = g_bias_y = g_bias_z = 0.0f;
    } else {
      saveBiasesToEEPROM();
    }
  } else {
    Serial.println("Loaded calibration biases from EEPROM.");
  }

  // Switch back to higher normal modes and ranges after calibration.
  writeRegisters(IMU_ADDRESS, PWR_MGMT_1, 0x03); // Re-lock PLL to X-Gyro for stable timing reference
  writeRegisters(IMU_ADDRESS, SMPLRT_DIV, 0x03); // Drop sample rate to 250Hz (optional, but matches library)
  writeRegisters(IMU_ADDRESS, MPU_CONFIG, 0x03); // Switch filter to 44Hz (cleaner for runtime complementary filter)//
  
  writeRegisters(IMU_ADDRESS, ACCEL_CONFIG, ACCEL_FS_8G);
  writeRegisters(IMU_ADDRESS, GYRO_CONFIG, GYRO_FS_2000DPS);
  accelLsbPerG = ACCEL_LSB_PER_G_8G;
  gyroLsbPerDps = GYRO_LSB_PER_DPS_2000;
  
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
  prevTime = micros(); // Seed timer so warmup and first loop get valid dt
  runWarmupLoop();
  prevTime = micros();
}

void loop() { //Reading sensors loop
  Haptics::update();

  HIDTransport::RumbleCommand rumbleCommand;
  if (HIDTransport::pollRumbleCommand(rumbleCommand)) {
    Haptics::onRumbleCommand(rumbleCommand);
  }

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

  applyAdaptiveBeta();

  //Inverse tangent of accel values for pitch and roll estimates
  pitch_acc = atan2(ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI; // inverse tan of (gravity along axis / perpendicular). Converts radians to degrees by multiplying by 180/PI.
  roll_acc  = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * 180.0 / PI;
 
  // Use accelerometer angle as initial estimate for complementary filter since gyro isn't an absolute value.
  if (firstFilterUpdate) {
    pitch = pitch_acc; 
    roll = roll_acc;
    yaw = 0.0f;
    firstFilterUpdate = false;
    if (deltaTime > 0.0f) {
      madgwickFilter.updateIMU(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, deltaTime);
    }
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

  gyroVelX += gx_dps * deltaTime;
  gyroVelY += gy_dps * deltaTime;
  gyroVelZ += gz_dps * deltaTime;
  gyroVelX *= FILTER_DROPOFF;
  gyroVelY *= FILTER_DROPOFF;
  gyroVelZ *= FILTER_DROPOFF;

  bool hidReadyNow = HIDTransport::ready();
  if (hidReadyNow) {
    bool buttonAPressed = (digitalRead(BUTTON_A) == LOW);
    bool buttonBPressed = (digitalRead(BUTTON_B) == LOW);
    bool joystickPressed = (digitalRead(JOYSTICK_BUTTON) == LOW);
    uint16_t joystickX = (uint16_t)analogRead(JOYSTICK_X);
    uint16_t joystickY = (uint16_t)analogRead(JOYSTICK_Y);
    uint8_t buttons = 0;
    if (buttonAPressed) {
      buttons |= 0x01;
    }
    if (buttonBPressed) {
      buttons |= 0x02;
    }
    if (joystickPressed) {
      buttons |= 0x04;
    }

    HIDTransport::sendQuaternion(
      madgwickFilter.getQuatW(),
      madgwickFilter.getQuatX(),
      madgwickFilter.getQuatY(),
      madgwickFilter.getQuatZ(),
      buttons,
      joystickX,
      joystickY
    );
    if (!hidLinkWasReady) {
      Serial.println("HID link connected. Streaming quaternion reports.");
    }
  } else {
    uint32_t nowMs = millis();
    if (hidLinkWasReady) {
      Serial.println("HID link disconnected. Waiting for reconnect...");
    } else if (nowMs - lastHidWaitPrintMs >= 2000U) {
      Serial.println("Waiting for HID host connection...");
      lastHidWaitPrintMs = nowMs;
    }
  }
  hidLinkWasReady = hidReadyNow;
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

bool loadBiasesFromEEPROM() {
  uint32_t storedMagic = 0;
  EEPROM.get(CALIB_MAGIC_ADDR, storedMagic);
  if (storedMagic != CALIB_EEPROM_MAGIC) {
    return false;
  }

  BiasCalibrationData stored = {0};
  EEPROM.get(CALIB_DATA_ADDR, stored);

  a_bias_x = stored.accelBiasX;
  a_bias_y = stored.accelBiasY;
  a_bias_z = stored.accelBiasZ;
  g_bias_x = stored.gyroBiasX;
  g_bias_y = stored.gyroBiasY;
  g_bias_z = stored.gyroBiasZ;
  return true;
}

void saveBiasesToEEPROM() {
  BiasCalibrationData stored;
  stored.accelBiasX = a_bias_x;
  stored.accelBiasY = a_bias_y;
  stored.accelBiasZ = a_bias_z;
  stored.gyroBiasX = g_bias_x;
  stored.gyroBiasY = g_bias_y;
  stored.gyroBiasZ = g_bias_z;

  EEPROM.put(CALIB_DATA_ADDR, stored);
  EEPROM.put(CALIB_MAGIC_ADDR, CALIB_EEPROM_MAGIC);
}

void applyAdaptiveBeta() {
  float av = gyroVelX * gyroVelX + gyroVelY * gyroVelY + gyroVelZ * gyroVelZ;
  if (av > 100.0f) {
    av = 100.0f;
  }

  float tunedBeta = av * (FILTER_MAX_BETA - FILTER_MIN_BETA) / 100.0f + FILTER_MIN_BETA;
  madgwickFilter.setBeta(tunedBeta);
}

void runWarmupLoop() {
  for (uint16_t i = 0; i < MADGWICK_WARMUP_SAMPLES; i++) {
    if (!getSensors()) {
      continue;
    }

    float warmAx = ax_g - a_bias_x;
    float warmAy = ay_g - a_bias_y;
    float warmAz = az_g - a_bias_z;
    float warmGx = gx_dps - g_bias_x;
    float warmGy = gy_dps - g_bias_y;
    float warmGz = gz_dps - g_bias_z;

    applyAdaptiveBeta();
    if (deltaTime > 0.0f) {
      madgwickFilter.updateIMU(warmGx, warmGy, warmGz, warmAx, warmAy, warmAz, deltaTime);
    }

    gyroVelX += warmGx * deltaTime;
    gyroVelY += warmGy * deltaTime;
    gyroVelZ += warmGz * deltaTime;
    gyroVelX *= FILTER_DROPOFF;
    gyroVelY *= FILTER_DROPOFF;
    gyroVelZ *= FILTER_DROPOFF;
    delay(2);
  }
}