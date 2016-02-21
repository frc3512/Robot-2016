// =============================================================================
// Description: Provides an interface for the robot's arm
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef ARM_HPP
#define ARM_HPP

#include <DigitalInput.h>
#include <Timer.h>

#include "../Constants.hpp"
#include "../Events/JoystickEventGenerator.hpp"
#include "../Events/DigitalInputEventGenerator.hpp"
#include "../Events/TimerEventGenerator.hpp"
#include "../MotionProfile/TrapezoidProfile.hpp"
#include "GearBox.hpp"
#include "SubsystemBase.hpp"
#include "Joystick.h"


class PIDController;
class TrapezoidProfile;

class Arm : public SubsystemBase {
public:
    Arm();

    void ReloadPID();
    void ResetEncoders();

    void SetCarryingHeight(double speed);
    void SetClimbHeight();
    bool GetManualOverride();

    void SetManualArmHeight(double height);
    void SetPIDArmHeight(double height);
    void SetManualCarriagePosition(double position);

    void UpdateState();

private:
    bool m_manual = true;
    double m_manualArmSpeed = 0.0;

    Joystick armStick{k_armStickPort};

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

    DigitalInput m_bottomLimitSwitch{k_bottomLeftLimitSwitchPin};
    DigitalInput m_topLimitSwitch{k_bottomRightLimitSwitchPin};
    DigitalInput m_leftCarriageLimitSwitch{k_leftCarriageLimitPin};
    DigitalInput m_rightCarriageLimitSwitch{k_rightCarriageLimitPin};

    std::shared_ptr<TrapezoidProfile> m_rightArmHeightProfile;
    std::shared_ptr<TrapezoidProfile> m_leftArmHeightProfile;

    StateMachine m_armSM{"ArmSM"};
};

#endif // ARM_HPP
