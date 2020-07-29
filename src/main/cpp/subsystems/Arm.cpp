// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Arm.hpp"

#include <iostream>
#include <string>
#include <utility>

Arm::Arm() {
    m_leftArmGrbx.SetLimitOnHigh(false);
    // m_leftArmGrbx.SetSoftPositionLimits(k_armMin , k_armMax);
    m_leftArmPID = std::make_shared<frc::PIDController>(
        0.0, 0.0, 0.0, 0.0, 0.0, &m_leftArmGrbx, &m_leftArmGrbx);

    m_carriagePID = std::make_shared<frc::PIDController>(
        0.0, 0.0, 0.0, 0.0, 0.0, &m_carriageGrbx, &m_carriageGrbx);

    m_winchPID = std::make_shared<frc::PIDController>(
        0.0, 0.0, 0.0, 0.0, 0.0, &m_winchGrbx, &m_winchGrbx);

    m_carriageLeftLimit = DigitalInputHandler::Get(k_carriageLeftLimitChannel);
    m_carriageRightLimit =
        DigitalInputHandler::Get(k_carriageRightLimitChannel);

    // Sets encoder type
    m_leftArmGrbx.GetMaster()->SetFeedbackDevice(
        CANTalon::CtreMagEncoder_Relative);
    m_leftArmGrbx.SetSensorDirection(true);
    m_leftArmGrbx.SetDistancePerPulse(k_armDpP);

    m_joystickEvent.RegisterButtonEvent("ZeroHeightButton", k_armStickPort,
                                        k_armZeroButton, true);
    /*m_joystickEvent.RegisterButtonEvent("CarryingHeightButton",
     * k_armStickPort,
     *                                   k_armCarryingButton,
     *                                   true);*/
    m_dioEvent.RegisterInputEvent("LeftArmZeroed", k_leftArmBottomLimitChannel,
                                  true, false, m_armSM);
    m_dioEvent.RegisterInputEvent("LeftArmUp", k_leftArmTopLimitChannel, true,
                                  false, m_armSM);

    // Idle
    auto state = std::make_unique<State>("Idle");
    state->Entry = [this] { m_leftArmGrbx.Set(0); };
    state->CheckTransition = [this](const std::string& event) {
        if (event == "CarryingHeightButton") {
            return "Idle";
        }
        if (event == "LeftArmZeroed") {
            m_leftArmGrbx.ResetEncoder();
            return "";
        } else if (event == "ZeroHeightButton") {
            return "ZeroHeight";
        } else {
            return "";
        }
    };
    m_armSM.AddState(std::move(state));

    // Start arm lift
    state = std::make_unique<State>("CarryingHeight");
    state->Run = [this] {
        if (GetManualOverride()) {
            m_leftArmGrbx.Set(1);
        }
    };
    state->CheckTransition = [this](const std::string& event) {
        if (m_leftArmPID->OnTarget()) {
            return "Idle";
        } else {
            return "";
        }
    };

    m_armSM.AddState(std::move(state));

    // Start arm lift
    state = std::make_unique<State>("ZeroHeight");
    state->Run = [this] { m_leftArmGrbx.Set(-1); };
    state->CheckTransition = [this](const std::string& event) {
        if (event == "LeftArmZeroed") {
            m_leftArmGrbx.ResetEncoder();
            return "Idle";
        } else {
            return "";
        }
    };

    m_armSM.AddState(std::move(state));
}

void Arm::SetManualOverride(bool manual) { m_manual = manual; }

bool Arm::GetManualOverride() const { return m_manual; }

void Arm::SetCarryingHeight(double speed) {
    if (GetManualOverride()) {
        m_leftArmPID->Disable();
        m_armHeight = speed;
    } else {
        m_leftArmPID->Enable();
        m_leftArmPID->SetSetpoint({0.0, speed / m_leftArmPID->GetV(), 0.0});
    }
}

void Arm::SetArmHeight(double height) {
    if (GetManualOverride()) {
        m_leftArmGrbx.Set(height);
    }
}

int32_t Arm::GetArmHeightRaw() const { return m_leftArmGrbx.Get(); }

double Arm::GetArmHeight() const { return m_leftArmGrbx.GetPosition(); }

double Arm::GetArmSpeed() const { return m_leftArmGrbx.GetSpeed(); }
void Arm::SetManualCarriagePosition(int direction) {
    m_carriageGrbx.Set(direction);
}

void Arm::ReloadPID() {}

void Arm::SetManualWinchHeight(double speed) { m_winchGrbx.Set(speed); }

void Arm::ResetEncoders() {}

void Arm::UpdateState() {
    m_armSM.Run();
    m_joystickEvent.Poll(m_armSM);
    m_joystickEvent.Update();
}
