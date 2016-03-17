// =============================================================================
// Description: Provides an interface for the robot's shooter
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef SHOOTER_HPP
#define SHOOTER_HPP

#include <Filters/LinearDigitalFilter.h>
#include <Relay.h>

#include "../Constants.hpp"
#include "../MotionProfile/TrapezoidProfile.hpp"
#include "../Events/JoystickEventGenerator.hpp"
#include "../Events/DigitalInputEventGenerator.hpp"
#include "../Events/TimerEventGenerator.hpp"
#include "../LeverPIDController.hpp"
#include "../SM/StateMachine.hpp"
#include "GearBox.hpp"
#include "SubsystemBase.hpp"

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

    float GetLeftRPM() const;
    float GetRightRPM() const;
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

    Timer m_joystickTimer;

    JoystickEventGenerator m_joystickEvent;
    DigitalInputEventGenerator m_dioEvent;
    TimerEventGenerator m_timerEvent{"ShootTimer", 2.0};

    std::shared_ptr<GearBox> m_leftShootGrbx;
    LinearDigitalFilter m_leftShootFilter{nullptr, {}, {}
    };

    std::shared_ptr<GearBox> m_rightShootGrbx;
    LinearDigitalFilter m_rightShootFilter{nullptr, {}, {}
    };

    GearBox m_shooterHeightGrbx{-1, -1, -1, k_shooterHeightID};
    std::shared_ptr<LeverPIDController> m_shooterHeightPID;

    GearBox m_rollBallGrbx{-1, -1, -1, k_rollBallID};
    GearBox m_armIntakeGrbx{-1, -1, -1, k_armIntakeID};

    StateMachine m_shootSM{"ShootSM"};

    friend class Robot;
};

#endif // ELEVATOR_HPP
