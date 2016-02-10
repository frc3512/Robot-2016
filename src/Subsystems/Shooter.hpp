// =============================================================================
// Description: Provides an interface for the robot's shooter
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef SHOOTER_HPP
#define SHOOTER_HPP

#include "SubsystemBase.hpp"
#include "../MotionProfile/TrapezoidProfile.hpp"
#include "../WPILib/CANTalon.h"
#include "../WPILib/PIDController.hpp"
#include "GearBox.hpp"
#include "DigitalInput.h"
#include "../StateMachine.hpp"
#include "Timer.h"
#include "PIDSource.h"
#include "PIDOutput.h"

class Shooter : public SubsystemBase {
public:
    Shooter();


    void ReloadPID();
    void ResetEncoders();

    void Shoot();
    void StartIntake();
    bool IsBallLoaded() const;
    void StopIntakeMotor();

    void ToggleManualOverride();
    bool GetManualOverride() const;

    void SetPIDShooterSpeed(double speed);
    void SetManualShooterSpeed(double speed);

    // For testing curving
    void SetLeftShooterSpeed(double speed);
    void SetRightShooterSpeed(double speed);

    float GetLeftRPM() const;
    float GetRightRPM() const;
    void ManualChangeSetpoint(double delta);

    void SetManualShooterHeight(double position);

    // Periodic
    void UpdateState();

private:
    bool m_manual = false;
    float m_latestLeftRPM = 0;
    float m_latestRightRPM = 0;

    // TODO: some CAN IDs conflict
    GearBox m_leftShootGrbx{-1, 10};
    std::shared_ptr<PIDController> m_leftShootPID;

    GearBox m_rightShootGrbx{-1, 6};
    std::shared_ptr<PIDController> m_rightShootPID;

    GearBox m_shooterHeightGrbx{-1, 7};
    std::shared_ptr<PIDController> m_shooterHeightPID;
    std::shared_ptr<TrapezoidProfile> m_shootHeightProfile;

    GearBox m_rollBallGrbx{-1, 5}; // TODO: fix ID

    DigitalInput m_intakeLimit{1};
    DigitalInput m_bottomLimit{2};

    StateMachine m_intakeSM{"START_INTAKE"};
};

#endif // ELEVATOR_HPP
