// =============================================================================
// File Name: Shooter.hpp
// Description: Provides an interface for the robot's shooter
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef SHOOTER_HPP
#define SHOOTER_HPP

#include "SubsystemBase.hpp"
#include "KalmanFilter.hpp"
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
    ~Shooter();

    void shoot();
    void setManualShooterSpeed( double speed );

    void setRPMLeft( double wheelSpeed );
    void setRPMRight( double wheelSpeed );
    float getRPMLeft();
    float getRPMRight();
    void updateLeftIntakeEncoderFilter( double Q , double R );
    void updateRightIntakeEncoderFilter( double Q , double R );
    void manualChangeSetpoint(double delta);

    void setManualShooterPosition(double position);

    // Periodic
    void updateState();

private:
    bool m_manual = false;
    float m_latestRPMLeft = 0;
    float m_latestRPMRight = 0;

    CANTalon m_leftShooterMotor{3};
    CANTalon m_rightShooterMotor{6};
    CANTalon m_kickBallMotor{7};
    CANTalon m_shooterPositionMotor{8};

    std::atomic <bool> m_updateProfile{true};
    KalmanFilter m_rpmFilterLeft{0 , 0};
    KalmanFilter m_rpmFilterRight{0 , 0};
    std::thread * m_profileUpdater;


    /* Maximum velocity and time to maximum velocity constants to load from the
     * config file

    double m_maxv_a;
    double m_ttmaxv_a;
    double m_maxv_b;
    double m_ttmaxv_b;
    */
};

#endif // ELEVATOR_HPP

