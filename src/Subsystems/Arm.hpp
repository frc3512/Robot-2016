// =============================================================================
// Description: Provides an interface for the robot's arm
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef ARM_HPP
#define ARM_HPP

#include <DigitalInput.h>
#include <Timer.h>

#include "../Constants.hpp"
#include "../Events/DigitalInputEventGenerator.hpp"
#include "../Events/JoystickEventGenerator.hpp"
#include "../Events/TimerEventGenerator.hpp"
#include "../MotionProfile/TrapezoidProfile.hpp"
#include "GearBox.hpp"
#include "SubsystemBase.hpp"

class PIDController;
class TrapezoidProfile;

class Arm : public SubsystemBase {
public:
    Arm();

    void ReloadPID();
    void ResetEncoders();

    void SetCarryingHeight(double speed);
    void SetArmHeight(double height);
    bool GetManualOverride();
    bool AtGoal() const;
    void ToggleManualOverride();

    void SetManualCarriagePosition(int direction);

    void UpdateState();

private:
    bool m_manual = true;
    int m_armHeight = 0.0;

    JoystickEventGenerator m_joystickEvent;
    DigitalInputEventGenerator m_dioEvent;

    GearBox m_leftArmActuator{-1, k_leftArmLiftID};
    std::shared_ptr<PIDController> m_leftArmPID;
    std::shared_ptr<TrapezoidProfile> m_leftArmProfile;

    GearBox m_rightArmActuator{-1, k_rightArmLiftID};
    std::shared_ptr<PIDController> m_rightArmPID;
    std::shared_ptr<TrapezoidProfile> m_rightArmProfile;

    GearBox m_carriagePositionMotor{-1, k_carriagePositionID};
    std::shared_ptr<PIDController> m_carriagePositionPID;
    std::shared_ptr<TrapezoidProfile> m_carriagePositionProfile;

    DigitalInput m_bottomLimitSwitch{k_armBottomLimitPin};
    DigitalInput m_topLimitSwitch{k_armTopLimitPin};
    DigitalInput m_leftCarriageLimitSwitch{k_leftCarriageLimitPin};
    DigitalInput m_rightCarriageLimitSwitch{k_rightCarriageLimitPin};

    std::shared_ptr<TrapezoidProfile> m_rightArmHeightProfile;
    std::shared_ptr<TrapezoidProfile> m_leftArmHeightProfile;

    StateMachine m_armSM{"ArmSM"};
};

#endif // ARM_HPP
