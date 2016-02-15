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
#include "../Events/JoystickEventGenerator.hpp"
#include "../Events/DigitalInputEventGenerator.hpp"
#include "../Events/TimerEventGenerator.hpp"
#include "../SM/StateMachine.hpp"
#include "../roboRIOID.hpp"
#include "GearBox.hpp"
#include <DigitalInput.h>
#include <Timer.h>
#include "LeverPIDController.hpp"
#include <PIDSource.h>
#include <PIDOutput.h>

class Shooter : public SubsystemBase {
public:
    Shooter();

    void ReloadPID();
    void ResetEncoders();

    void ToggleManualOverride();
    bool GetManualOverride() const;

    int32_t GetShootHeightValue() const;

    void SetPIDShooterSpeed(double speed);
    void SetManualShooterSpeed(double speed);

    // void SetShooterHeight(double height);

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

    JoystickEventGenerator m_joystickEvent;
    DigitalInputEventGenerator m_dioEvent;
    TimerEventGenerator m_timerEvent{"ShootTimer", 3.0};

    // TODO: some CAN IDs conflict
    GearBox m_leftShootGrbx{-1, k_leftShooterID};
    std::shared_ptr<PIDController> m_leftShootPID;

    GearBox m_rightShootGrbx{-1, k_rightShooterID};
    std::shared_ptr<PIDController> m_rightShootPID;

    GearBox m_shooterHeightGrbx{-1, k_shooterHeightID};
    std::shared_ptr<LeverPIDController> m_shooterHeightPID;
    std::shared_ptr<TrapezoidProfile> m_shootHeightProfile;

    GearBox m_rollBallGrbx{-1, k_rollBallID};

    DigitalInput m_bottomLimit{k_bottomLimitPin};

    StateMachine m_intakeSM{"IntakeSM"};
    StateMachine m_shootSM{"ShootSM"};
};

#endif // ELEVATOR_HPP
