// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

#include <memory>

#include <frc2/Timer.h>

#include "CANDigitalInput.hpp"
#include "CANEncoder.hpp"
#include "Constants.hpp"

/**
 * Provides an interface for the robot's arm
 */
class Arm {
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

    CANDigitalInput m_dio;

    SpeedControllerGroup m_leftArmGrbx{-1, k_leftArmBottomLimitChannel,
                                       k_leftArmTopLimitChannel,
                                       k_leftArmLiftID};
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
