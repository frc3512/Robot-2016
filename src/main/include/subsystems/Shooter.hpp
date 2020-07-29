// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

#include <memory>

#include <frc/LinearFilter.h>
#include <frc/controller/ProfiledPIDController.h>

#include "Constants.hpp"
#include "StateMachine.hpp"

/**
 * Provides an interface for the robot's shooter
 */
class Shooter : public SubsystemBase {
public:
    Shooter();

    void ReloadPID();
    void ResetEncoders();

    void SetManualOverride(bool manual);
    bool GetManualOverride() const;

    double GetShooterHeight() const;
    double GetShooterHeightRaw() const;
    PIDState GetShooterHeightSetpoint() const;
    void SetShooterSpeed(double speed);

    void SetShooterHeight(double height, bool increment);

    double GetLeftRPM() const;
    double GetRightRPM() const;
    PIDState GetLeftSetpoint() const;
    PIDState GetRightSetpoint() const;
    void ManualChangeSetpoint(double delta);

    // Periodic
    void UpdateState();

private:
    // TODO: change this to false once the velocity feed forwards are non-zero
    bool m_manual = false;
    double m_manualShooterSpeed = 0.0;
    PIDState m_shooterHeight;

    frc2::Timer m_joystickTimer;

    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_leftShootGrbx{
        k_leftShooterID};
    frc::LinearFilter<units::rad_per_s> m_leftShootFilter{};

    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_rightShootGrbx{
        k_rightShooterID};
    frc::LinearFilter<units::rad_per_s> m_rightShootFilter{};

    frc::ProfiledPIDController m_shooterWheelController{
        k_wheelShooterP, k_wheelShooterI, k_wheelShooterD,
        frc::TrapezoidProfile<units::radians>::Constraints{k_wheelShooterV,
                                                           k_wheelShooterA}};

    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_shooterHeightGrbx{
        k_shooterHeightID};
    frc::ProfiledPIDController m_shooterHeightController{
        k_heightShooterP, k_heightShooterI, k_heightShooterD,
        frc::TrapezoidProfile<units::radians>::Constraints{k_heightShooterV,
                                                           k_heightShooterA}};

    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_rollBallGrbx{k_rollBallID};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_armIntakeGrbx{
        k_armIntakeID};

    StateMachine m_shootSM{"ShootSM"};
};
