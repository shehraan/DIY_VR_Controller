#pragma once
#include <math.h>

class MadgwickFilter {
private:
    float beta; // Algorithm gain
    float q0, q1, q2, q3; // Quaternion

public:
    MadgwickFilter();
    void begin(float confBeta);
    void setBeta(float newBeta);
    float getBeta() const { return beta; }
    
    // Pass in your dt directly from main to avoid clock issues
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);

    float getQuatW() { return q0; }
    float getQuatX() { return q1; }
    float getQuatY() { return q2; }
    float getQuatZ() { return q3; }

    // Helpers to quickly compare against your complementary filter
    float getRoll();
    float getPitch();
    float getYaw();
};