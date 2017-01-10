// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

#include <memory>

#include "../Constants.hpp"
#include "../Differential.hpp"
#include "../MotionProfile/TrapezoidProfile.hpp"
#include "../SM/StateMachine.hpp"
#include "../Utility.hpp"
#include "../WPILib/PIDController.hpp"
#include "GearBox.hpp"
#include "SubsystemBase.hpp"

class GearBox;
class PIDController;

/**
 * Provides an interface for this year's drive train
 */
class DriveTrain : public SubsystemBase {
public:
    DriveTrain();

    int32_t GetLeftRaw() const;
    int32_t GetRightRaw() const;

    /* Drives robot with given speed and turn values [-1..1].
     * This is a convenience function for use in Operator Control.
     */
    void Drive(float throttle, float turn, bool isQuickTurn = false);

    // Sets joystick deadband
    void SetDeadband(float band);

    // Reload PID constants
    void ReloadPID();

    // Set encoder distances to 0
    void ResetEncoders();

    void DiffDrive(float output);

    // Directly set wheel speeds [0..1] (see GearBox::SetManual(float))
    void SetLeftManual(float value);
    void SetRightManual(float value);

    // Returns encoder distances
    double GetLeftDisplacement() const;
    double GetRightDisplacement() const;

    // Returns encoder rates
    double GetLeftRate() const;
    double GetRightRate() const;

    double DiffPIDGet();

    void EnablePID();
    void DisablePID();

    // Returns encoder PID loop setpoints
    PIDState GetLeftSetpoint() const;
    PIDState GetRightSetpoint() const;

    PIDState GetLeftGoal() const;

    void SetGoal(PIDState goal);
    bool AtGoal() const;
    void ResetProfile();

private:
    float m_deadband = k_joystickDeadband;
    float m_sensitivity;

    // Cheesy Drive variables
    float m_oldTurn = 0.f;
    float m_quickStopAccumulator = 0.f;
    float m_negInertiaAccumulator = 0.f;

    GearBox m_leftGrbx{-1, -1, -1, k_leftDriveMasterID, k_leftDriveSlaveID};
    GearBox m_rightGrbx{-1, -1, -1, k_rightDriveMasterID, k_rightDriveSlaveID};

    Differential m_diff{&m_leftGrbx, &m_rightGrbx};
    frc::PIDController m_diffPID{k_diffDriveP, k_diffDriveI, k_diffDriveD,
                                 k_diffDriveV, k_diffDriveA, &m_diff,
                                 &m_diff};
};
