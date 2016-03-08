// =============================================================================
// Description: Provides an interface for the robot's arm
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef ARM_HPP
#define ARM_HPP

#include <Timer.h>

#include "../Constants.hpp"
#include "../DigitalInputHandler.hpp"
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

    int32_t GetArmHeightValue() const;
    double GetArmSpeed() const;

    void SetManualCarriagePosition(int direction);

    void SetManualWinchHeight(double speed);

    void UpdateState();

private:
    bool m_manual = true;
    int m_armHeight = 0.0;

    JoystickEventGenerator m_joystickEvent;
    DigitalInputEventGenerator m_dioEvent;

    GearBox m_leftArmActuator{-1, k_armLeftBottomLimitChannel,
                              k_armLeftTopLimitChannel, k_leftArmLiftID};
    std::shared_ptr<PIDController> m_leftArmPID;
    std::shared_ptr<TrapezoidProfile> m_leftArmProfile;

    GearBox m_carriageGrbx{-1, -1, -1, k_carriageID};
    std::shared_ptr<PIDController> m_carriagePID;
    std::shared_ptr<TrapezoidProfile> m_carriageProfile;

    GearBox m_winchGrbx{-1, -1, -1, k_winchID};
    std::shared_ptr<PIDController> m_winchPID;
    std::shared_ptr<TrapezoidProfile> m_winchProfile;

    DigitalInput* m_leftCarriageLimit = nullptr;

    StateMachine m_armSM{"ArmSM"};
};

#endif // ARM_HPP
