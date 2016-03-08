// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <atomic>

#include <I2C.h>
#include <Notifier.h>

/**
 * Provides access to an ITG3200 3-axis digital gyroscope via the I2C bus
 */
class ITG3200 {
public:
    explicit ITG3200(frc::I2C::Port port, uint32_t deviceAddress = k_address);

    double GetAngleX() const;
    double GetAngleY() const;
    double GetAngleZ() const;

    double GetRateX();
    double GetRateY();
    double GetRateZ();

    void Reset();
    void Calibrate();

protected:
    frc::I2C m_i2c;

private:
    static constexpr uint8_t k_address = 0x68;

    static constexpr uint8_t k_samplePeriodDividerRegister = 0x15;
    static constexpr uint8_t k_digitalLowPassFilterRegister = 0x16;
    static constexpr uint8_t k_gyroXOutRegister = 0x1D;
    static constexpr uint8_t k_gyroYOutRegister = 0x1F;
    static constexpr uint8_t k_gyroZOutRegister = 0x21;

    // Digital Low Pass Filter (DLPF), Full Scale Register Bits
    // FS_SEL must be set to 3 for proper operation
    static constexpr uint8_t DLPF_CFG_0 = 1 << 0;
    static constexpr uint8_t DLPF_CFG_1 = 1 << 1;
    static constexpr uint8_t DLPF_CFG_2 = 1 << 2;
    static constexpr uint8_t DLPF_FS_SEL_0 = 1 << 3;
    static constexpr uint8_t DLPF_FS_SEL_1 = 1 << 4;

    static constexpr double k_calibrationSampleTime = 2.0;
    static constexpr double k_samplePeriod = 0.02;
    static constexpr int16_t k_numCalibrationSamples =
        k_calibrationSampleTime / k_samplePeriod;

    // Sensitivity of gyro in least significant bits per degree/second
    static constexpr double k_degreesPerSecondPerLSB = 1.0 / 14.375;

    std::atomic<double> m_xAngle{0.0};
    std::atomic<double> m_yAngle{0.0};
    std::atomic<double> m_zAngle{0.0};

    int16_t m_xZero = 0;
    int16_t m_yZero = 0;
    int16_t m_zZero = 0;

    frc::Notifier m_loop{&ITG3200::Calculate, this};

    // Used by notifier to update angle estimate
    void Calculate();

    // Read raw X axis from gyro
    int16_t GetRawX();

    // Read raw Y axis from gyro
    int16_t GetRawY();

    // Read raw Z axis from gyro
    int16_t GetRawZ();
};
