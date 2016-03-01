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
    void SetManualOverride(bool manual);
    bool GetManualOverride() const;
    bool AtGoal() const;

    void SetManualCarriagePosition(int direction);

    void SetManualWinchHeight(double speed);

    void UpdateState();

private:
    bool m_manual = true;
    int m_armHeight = 0.0;

    JoystickEventGenerator m_joystickEvent;
    DigitalInputEventGenerator m_dioEvent;

    GearBox m_leftArmActuator{-1, k_leftArmLiftID};
    std::shared_ptr<PIDController> m_leftArmPID;
    std::shared_ptr<TrapezoidProfile> m_leftArmProfile;

    GearBox m_carriagePositionMotor{-1, k_carriagePositionID};
    std::shared_ptr<PIDController> m_carriagePositionPID;
    std::shared_ptr<TrapezoidProfile> m_carriagePositionProfile;

    GearBox m_winchPositionMotor{-1, k_winchPositionID};
    std::shared_ptr<PIDController> m_winchPositionPID;
    std::shared_ptr<TrapezoidProfile> m_winchPositionProfile;

    DigitalInput m_bottomLeftLimitSwitch{k_armLeftBottomLimitPin};
    DigitalInput m_topLeftLimitSwitch{k_armLeftTopLimitPin};
    DigitalInput m_leftCarriageLimitSwitch{k_leftCarriageLimitPin};

    std::shared_ptr<TrapezoidProfile> m_leftArmHeightProfile;

    StateMachine m_armSM{"ArmSM"};
};

#endif // ARM_HPP
