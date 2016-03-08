// =============================================================================
// Description: Provides access to an ITG3200 3-axis digital gyroscope via the
//              I2C bus
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "ITG3200.hpp"
#include <Timer.h>

ITG3200::ITG3200(Port port, uint32_t deviceAddress) : I2C(port, deviceAddress) {
    // Sets gyro at +-2000deg/sec and 98Hz low pass filter
    Write(k_digitalLowPassFilterRegister,
          DLPF_CFG_1 | DLPF_FS_SEL_0 | DLPF_FS_SEL_1);

    // Sets sample rate to 100Hz
    Write(k_samplePeriodDividerRegister, 0x09);

    Calibrate();

    m_loop.StartPeriodic(k_samplePeriod);
}

double ITG3200::GetAngleX() const {
    return m_xAngle;
}

double ITG3200::GetAngleY() const {
    return m_yAngle;
}

double ITG3200::GetAngleZ() const {
    return m_zAngle;
}

double ITG3200::GetRateX() {
    return (static_cast<double>(GetRawX()) - m_xZero) *
           k_degreesPerSecondPerLSB;
}

double ITG3200::GetRateY() {
    return (static_cast<double>(GetRawY()) - m_yZero) *
           k_degreesPerSecondPerLSB;
}

double ITG3200::GetRateZ() {
    return (static_cast<double>(GetRawZ()) - m_zZero) *
           k_degreesPerSecondPerLSB;
}

void ITG3200::Reset() {
    m_xAngle = 0.0;
    m_yAngle = 0.0;
    m_zAngle = 0.0;
}

void ITG3200::Calibrate() {
    uint16_t sumX = 0;
    uint16_t sumY = 0;
    uint16_t sumZ = 0;

    // 2 seconds * 100 Hz = 200 samples
    for (int time = 0; time < 100.0 * k_calibrationSampleTime; time++) {
        sumX += GetRawX();
        sumY += GetRawY();
        sumZ += GetRawZ();
        Wait(k_samplePeriod);
    }

    m_xZero = sumX / k_numCalibrationSamples;
    m_yZero = sumY / k_numCalibrationSamples;
    m_zZero = sumZ / k_numCalibrationSamples;
}

void ITG3200::Calculate() {
    m_xAngle = m_xAngle + GetRateX() * k_samplePeriod;
    m_yAngle = m_yAngle + GetRateY() * k_samplePeriod;
    m_zAngle = m_zAngle + GetRateZ() * k_samplePeriod;
}

int16_t ITG3200::GetRawX() {
    uint8_t data[2];

    Read(k_gyroXOutRegister, 2, data);
    return (data[0] << 8) | data[1];
}

int16_t ITG3200::GetRawY() {
    uint8_t data[2];

    Read(k_gyroYOutRegister, 2, data);
    return (data[0] << 8) | data[1];
}

int16_t ITG3200::GetRawZ() {
    uint8_t data[2];

    Read(k_gyroZOutRegister, 2, data);
    return (data[0] << 8) | data[1];
}
