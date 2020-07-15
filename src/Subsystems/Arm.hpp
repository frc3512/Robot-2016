// Copyright (c) 2016-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <memory>

#include <Timer.h>

#include "../Constants.hpp"
#include "../DigitalInputHandler.hpp"
#include "../Events/DigitalInputEventGenerator.hpp"
#include "../Events/JoystickEventGenerator.hpp"
#include "../Events/TimerEventGenerator.hpp"
#include "../MotionProfile/TrapezoidProfile.hpp"
#include "GearBox.hpp"
#include "SubsystemBase.hpp"

namespace frc {
class PIDController;
}  // namespace frc

class TrapezoidProfile;

/**
 * Provides an interface for the robot's arm
 */
class Arm : public SubsystemBase {
public:
    Arm();

    void ReloadPID();
    void ResetEncoders();

    void SetCarryingHeight(double speed);
    void SetArmHeight(double height);
    void SetManualOverride(bool manual);
    bool GetManualOverride() const;

    int32_t GetArmHeightRaw() const;
    double GetArmHeight() const;
    double GetArmSpeed() const;

    void SetManualCarriagePosition(int direction);

    void SetManualWinchHeight(double speed);

    void UpdateState();

private:
    bool m_manual = true;
    int m_armHeight = 0.0;

    JoystickEventGenerator m_joystickEvent;
    DigitalInputEventGenerator m_dioEvent;

    GearBox m_leftArmGrbx{-1, k_leftArmBottomLimitChannel,
                          k_leftArmTopLimitChannel, k_leftArmLiftID};
    std::shared_ptr<frc::PIDController> m_leftArmPID;

    GearBox m_carriageGrbx{-1, k_carriageLeftLimitChannel,
                           k_carriageRightLimitChannel, k_carriageID};
    std::shared_ptr<frc::PIDController> m_carriagePID;

    GearBox m_winchGrbx{-1, -1, -1, k_winchID};
    std::shared_ptr<frc::PIDController> m_winchPID;

    DigitalInput* m_carriageLeftLimit = nullptr;
    DigitalInput* m_carriageRightLimit = nullptr;

    StateMachine m_armSM{"ArmSM"};
};
