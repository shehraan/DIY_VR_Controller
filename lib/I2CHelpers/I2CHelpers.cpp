#include "I2CHelpers.h"
#include "Wire.h"

bool writeRegisters(uint8_t addr, uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(addr); // Device address to start transmission
    Wire.write(reg);          // Register address to write to
    Wire.write(data);                // Data byte for that register
    if (Wire.endTransmission() != 0) { // Future additions go to consecutive registers and that one register write is complete, so we end transmission for now
        return false; // Confirm if command worked correctly (via 0)
    }          
    return true;
}

bool readRegisters(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(addr); 
    Wire.write(reg); //Register address to read from
    
    // Send the buffered bytes and a repeated start
    if (Wire.endTransmission(false) != 0) {  
        return false; // Confirm if command worked correctly (via 0)
    }

    uint8_t byteCount = Wire.requestFrom(addr, len); // Gets bytes into Rx buffer
    if (byteCount != len) {
        return false; // Confirm if we got all 14 bytes
    }

    for (uint8_t i = 0; i < len; i++) {
        if (!Wire.available()) {
            return false; // Extra safety to protect against edge cases like Rx buffer glitching
        }
        buf[i] = Wire.read(); // Copy bytes from RX buffer into caller buffer
    }
    return true;
}