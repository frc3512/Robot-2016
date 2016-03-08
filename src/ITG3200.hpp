// =============================================================================
// Description: Provides access to an ITG3200 3-axis digital gyroscope via the
//              I2C bus
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef ITG3200_HPP
#define ITG3200_HPP

#include <cstdint>

#include <I2C.h>

class ITG3200 : public I2C {
public:
    ITG3200(Port port, uint32_t deviceAddress = k_address);

    double GetRateX();
    double GetRateY();
    double GetRateZ();

private:
    static constexpr uint8_t k_address = 0x69;

    static constexpr uint8_t k_sampleRateDividerRegister = 0x15;
    static constexpr uint8_t k_digitalLowPassFilterRegister = 0x16;
    static constexpr uint8_t k_gyroXOutRegister = 0x1D;
    static constexpr uint8_t k_gyroYOutRegister = 0x1F;
    static constexpr uint8_t k_gyroZOutRegister = 0x21;

    // Digital Low Pass Filter (DLPF), Full Scale Register Bits
    // FS_SEL must be set to 3 for proper operation
    static constexpr uint8_t DLPF_CFG_0 = 1 << 0;
    static constexpr uint8_t DLPF_CFG_1 = 1 << 1;
    static constexpr uint8_t DLPF_CFG_2 = 1 << 2;
    static constexpr uint8_t DLPF_FS_SEL_0 = 1<<3;
    static constexpr uint8_t DLPF_FS_SEL_1 = 1<<4;

    // Zero points for gyro
    static constexpr double k_zeroX = 52.3;
    static constexpr double k_zeroY = -18.5;
    static constexpr double k_zeroZ = 0.0;

    // Sensitivity of gyro in least significant bits per degree/second
    static constexpr double k_degreesPerSecondPerLSB = 1.0 / 14.375;

    // Read raw X axis from gyro
    int16_t GetRawX();

    // Read raw Y axis from gyro
    int16_t GetRawY();

    // Read raw Z axis from gyro
    int16_t GetRawZ();
};

#endif // ITG3200_HPP
