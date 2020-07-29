// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

#include <memory>

#include <frc/SpeedControllerGroup.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/drive/DifferentialDrive.h>

#include "CANEncoder.hpp"
#include "Constants.hpp"
#include "StateMachine.hpp"

/**
 * Provides an interface for this year's drive train
 */
class Drivetrain {
public:
    Drivetrain();

    int32_t GetLeftRaw() const;
    int32_t GetRightRaw() const;

    /* Drives robot with given speed and turn values [-1..1].
     * This is a convenience function for use in Operator Control.
     */
    void Drive(double throttle, double turn, bool isQuickTurn = false);

    // Sets joystick deadband
    void SetDeadband(double band);

    // Reload PID constants
    void ReloadPID();

    // Set encoder distances to 0
    void ResetEncoders();

    // Directly set wheel speeds [0..1] (see GearBox::SetManual(double))
    void SetLeftManual(double value);
    void SetRightManual(double value);

    // Returns encoder distances
    units::inch_t GetLeftDisplacement();
    units::inch_t GetRightDisplacement();

    // Returns encoder rates
    units::feet_per_second_t GetLeftRate();
    units::feet_per_second_t GetRightRate();

    double DiffPIDGet();

    void EnablePID();
    void DisablePID();

    void SetGoal(units::foot_t goal);
    bool AtGoal() const;
    void ResetProfile();

private:
    double m_deadband = k_joystickDeadband;
    double m_sensitivity;

    // Cheesy Drive variables
    double m_oldTurn = 0.0;
    double m_quickStopAccumulator = 0.0;
    double m_negInertiaAccumulator = 0.0;

    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_leftDriveMaster{
        k_leftDriveMasterID};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_leftDriveSlave{
        k_leftDriveSlaveID};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_rightDriveMaster{
        k_rightDriveMasterID};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_rightDriveSlave{
        k_rightDriveSlaveID};

    CANEncoder m_leftEncoder{m_leftDriveMaster, true};
    CANEncoder m_rightEncoder{m_rightDriveMaster, true};

    frc::SpeedControllerGroup m_leftGrbx{m_leftDriveMaster, m_leftDriveSlave};
    frc::SpeedControllerGroup m_rightGrbx{m_rightDriveMaster,
                                          m_rightDriveSlave};

    frc::DifferentialDrive m_drive{m_leftGrbx, m_rightGrbx};

    frc::ProfiledPIDController<units::feet> m_leftController{
        k_diffDriveP, k_diffDriveI, k_diffDriveD,
        frc::TrapezoidProfile<units::feet>::Constraints{k_diffDriveV,
                                                        k_diffDriveA},
        20_ms};
    frc::ProfiledPIDController<units::feet> m_rightController{
        k_diffDriveP, k_diffDriveI, k_diffDriveD,
        frc::TrapezoidProfile<units::feet>::Constraints{k_diffDriveV,
                                                        k_diffDriveA},
        20_ms};
};
