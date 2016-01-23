// =============================================================================
// File Name: Shooter.hpp
// Description: Provides an interface for the robot's shooter
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef SHOOTER_HPP
#define SHOOTER_HPP

class CANTalon;

#include "SubsystemBase.hpp"
#include <vector>
#include <thread>
#include <atomic>
#include <Timer.h>
#include <string>

#include "../StateMachine.hpp"

class Shooter : public SubsystemBase {
public:

    Shooter();
    ~Shooter();


    void reloadPID();
    void resetEncoders();

    void setRPM(float wheelSpeed);

    void getRPM();

    void manualChangeSetpoint(double delta);

    // Periodic
    void updateState();

private:
    bool m_manual = false;

    // Intake
    CANTalon m_shootWheelLeft{3};
    CANTalon m_shootWheelRight{6};


    /* Maximum velocity and time to maximum velocity constants to load from the
     * config file

    double m_maxv_a;
    double m_ttmaxv_a;
    double m_maxv_b;
    double m_ttmaxv_b;
    */
};

#endif // ELEVATOR_HPP

