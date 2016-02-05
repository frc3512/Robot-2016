// =============================================================================
// Description: Provides an interface for the robot's shooter
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef SHOOTER_HPP
#define SHOOTER_HPP

#include "SubsystemBase.hpp"
#include <vector>
#include <thread>
#include <atomic>
#include <Timer.h>
#include <string>

#include <CANTalon.h>

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

    void SetRPMLeft(double wheelSpeed);
    void SetRPMRight(double wheelSpeed);
    float GetRPMLeft();
    float GetRPMRight();
    void ManualChangeSetpoint(double delta);

    void SetManualShooterPosition(double position);

    // Periodic
    void UpdateState();

private:
    bool m_manual = false;
    float m_latestRPMLeft = 0;
    float m_latestRPMRight = 0;

    CANTalon m_leftShooterMotor{3};
    CANTalon m_rightShooterMotor{6};
    CANTalon m_kickBallMotor{0};
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
