// =============================================================================
// Description: Provides access to an ITG3200 3-axis digital gyroscope via the
//              I2C bus
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "ITG3200.hpp"

ITG3200::ITG3200(Port port, uint32_t deviceAddress) : I2C(port, deviceAddress) {
    // Sets gyro at +-2000deg/sec and 98Hz low pass filter
    Write(k_digitalLowPassFilterRegister,
          DLPF_CFG_1 | DLPF_FS_SEL_0 | DLPF_FS_SEL_1);

    // Sets sample rate to 100Hz
    Write(k_sampleRateDividerRegister, 0x09);
}

double ITG3200::GetRateX() {
    return (static_cast<double>(GetRawX()) - k_zeroX) *
        k_degreesPerSecondPerLSB;
}

double ITG3200::GetRateY() {
    return (static_cast<double>(GetRawY()) - k_zeroY) *
        k_degreesPerSecondPerLSB;
}

double ITG3200::GetRateZ() {
    return (static_cast<double>(GetRawZ()) - k_zeroZ) *
        k_degreesPerSecondPerLSB;
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
