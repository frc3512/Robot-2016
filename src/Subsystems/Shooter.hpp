// =============================================================================
// Description: Provides an interface for the robot's shooter
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef SHOOTER_HPP
#define SHOOTER_HPP

#include "SubsystemBase.hpp"
#include "../WPILib/CANTalon.h"
#include "../StateMachine.hpp"

class Shooter : public SubsystemBase {
public:
    Shooter();

    void ReloadPID();
    void ResetEncoders();

    void Shoot();

    void SetManualShooterSpeed(double speed);

    // For testing curving
    void SetLeftShooterSpeed(double speed);
    void SetRightShooterSpeed(double speed);

    void SetLeftRPM(double wheelSpeed);
    void SetRightRPM(double wheelSpeed);
    float GetLeftRPM() const;
    float GetRightRPM() const;
    void ManualChangeSetpoint(double delta);

    void SetManualShooterPosition(double position);

    // Periodic
    void UpdateState();

private:
    bool m_manual = false;
    float m_latestLeftRPM = 0;
    float m_latestRightRPM = 0;

    // TODO: some CAN IDs conflict
    CANTalon m_leftShooterMotor{3};
    CANTalon m_rightShooterMotor{6};
    CANTalon m_kickBallMotor{9}; // TODO: fix ID
    CANTalon m_shooterPositionMotor{0};
    CANTalon m_shootElevationMotor{7};

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
