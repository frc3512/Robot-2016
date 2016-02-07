// =============================================================================
// Description: Provides an interface for the robot's shooter
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef SHOOTER_HPP
#define SHOOTER_HPP

#include "SubsystemBase.hpp"
#include "../MotionProfile/TrapezoidProfile.hpp"
#include "../WPILib/CANTalon.h"

#include "DigitalInput.h"
#include "../StateMachine.hpp"
#include "Timer.h"
#include "PIDSource.h"
#include "PIDOutput.h"

#include <thread>
#include <atomic>

class PIDController;

class Shooter : public SubsystemBase {
public:
    Shooter();
    ~Shooter();

    void ReloadPID();
    void ResetEncoders();

    void Shoot();
    void Intake();
    bool IsBallLoaded();
    void StopIntakeMotor();

    void SetManualShooterSpeed(double speed);

    // For testing curving
    void SetLeftShooterSpeed(double speed);
    void SetRightShooterSpeed(double speed);

    void SetLeftRPM(double wheelSpeed);
    void SetRightRPM(double wheelSpeed);
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
    CANTalon m_leftShooterMotor{10};
    CANTalon m_rightShooterMotor{5};
    CANTalon m_rollBallMotor{6}; // TODO: fix ID
    CANTalon m_shooterHeightMotor{7};

    DigitalInput m_intakeLimit{1};
    DigitalInput m_bottomLimit{2};

    //std::shared_ptr<PIDController> m_shooterHeightPID{m_shooterHeightPID, 0.0, 0.0};

    //Timer m_profileTimer;
    //std::atomic<bool> m_updateProfile{true};
    //std::unique_ptr <TrapezoidProfile> m_shootHeightProfile{ 0.f, 0.f, 0.f, 0.f, 0.f, &m_shooterHeightMotor, &m_shooterHeightMotor};

    /* Maximum velocity and time to maximum velocity constants to load from the
     * config file
     *
     *  double m_maxv_a;
     *  double m_ttmaxv_a;
     *  double m_maxv_b;
     *  double m_ttmaxv_b;
     */
};

#endif // ELEVATOR_HPP
