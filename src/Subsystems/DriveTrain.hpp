// =============================================================================
// Description: Provides an interface for this year's drive train
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef DRIVE_TRAIN_HPP
#define DRIVE_TRAIN_HPP

#include "SubsystemBase.hpp"
#include "GearBox.hpp"
#include "../MotionProfile/TrapezoidProfile.hpp"
#include "../Utility.hpp"
#include <memory>

class GearBox;
class PIDController;

class DriveTrain : public SubsystemBase {
public:
    DriveTrain();

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

    // Set wheel setpoints (see GearBox::setSetpoint(float))
    void SetLeftSetpoint(PIDState setpt);
    void SetRightSetpoint(PIDState setpt);

    // Directly set wheel speeds [0..1] (see GearBox::setManual(float))
    void SetLeftManual(float value);
    void SetRightManual(float value);

    // Returns encoder distances
    double GetLeftDist() const;
    double GetRightDist() const;

    // Returns encoder rates
    double GetLeftRate() const;
    double GetRightRate() const;

    // Returns encoder PID loop setpoints
    PIDState GetLeftSetpoint() const;
    PIDState GetRightSetpoint() const;

    void SetGoal(PIDState goal);
    bool AtGoal() const;
    void ResetProfile();

    const static float maxWheelSpeed;

private:
    float m_deadband = 0.02f;
    float m_sensitivity;

    // Cheesy Drive variables
    float m_oldTurn = 0.f;
    float m_quickStopAccumulator = 0.f;
    float m_negInertiaAccumulator = 0.f;

    GearBox m_leftGrbx{-1, 1};
    GearBox m_rightGrbx{-1, 4};
    std::shared_ptr<PIDController> m_leftPID;
    std::unique_ptr<TrapezoidProfile> m_leftProfile;
    std::shared_ptr<PIDController> m_rightPID;
    std::unique_ptr<TrapezoidProfile> m_rightProfile;
};

#endif // DRIVE_TRAIN_HPP
