#ifndef I2CRegisterIO_H
#define I2CRegisterIO_H

#include <Wire.h>
#include <Arduino.h>

// Create IO helper functions
bool writeRegisters(uint8_t addr, uint8_t reg, uint8_t data);
bool readRegisters(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len = 1);

#endif