// =============================================================================
// Description: Provides an interface for the robot's arm
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "../WPILib/PIDController.hpp"
#include "Arm.hpp"
#include <iostream>

Arm::Arm() {
    m_leftArmPID = std::make_shared<PIDController>(0.f,
                                                   0.f,
                                                   0.f,
                                                   0.f,
                                                   0.f,
                                                   &m_leftArmActuator,
                                                   &m_leftArmActuator);
    m_leftArmProfile = std::make_shared<TrapezoidProfile>(m_leftArmPID, 0.0,
                                                          0.0);

    m_rightArmPID = std::make_shared<PIDController>(0.f,
                                                    0.f,
                                                    0.f,
                                                    0.f,
                                                    0.f,
                                                    &m_rightArmActuator,
                                                    &m_rightArmActuator);
    m_rightArmProfile = std::make_shared<TrapezoidProfile>(m_rightArmPID, 0.0,
                                                           0.0);

    // Sets encoder type
    m_leftArmActuator.GetMaster()->SetFeedbackDevice(
        CANTalon::CtreMagEncoder_Relative);
    m_rightArmActuator.GetMaster()->SetFeedbackDevice(
        CANTalon::CtreMagEncoder_Relative);

    // Reversing encoder values
    m_leftArmActuator.SetSensorDirection(true);
    m_rightArmActuator.SetSensorDirection(true);

    m_joystickEvent.RegisterButtonEvent("ZeroHeightButton", k_armStickPort,
                                        k_armZeroButton,
                                        true); // TODO: change port number
    m_joystickEvent.RegisterButtonEvent("CarryingHeightButton", k_armStickPort,
                                        k_armCarryingButton,
                                        true); // TODO: change port number
    m_dioEvent.RegisterInputEvent("RightArmZeroed", k_armRightBottomLimitPin,
                                  true,
                                  false, m_armSM);
    m_dioEvent.RegisterInputEvent("RightArmUp", k_armRightTopLimitPin,
                                  true,
                                  false, m_armSM);
    m_dioEvent.RegisterInputEvent("LeftArmZeroed", k_armLeftBottomLimitPin,
                                  true,
                                  false, m_armSM);
    m_dioEvent.RegisterInputEvent("LeftArmUp", k_armLeftTopLimitPin,
                                  true,
                                  false, m_armSM);

    // Idle
    auto state = std::make_unique<State>("Idle");
    state->Entry = [this] {
        m_leftArmActuator.Set(0);
        m_rightArmActuator.Set(0);
    };
    state->CheckTransition = [this] (const std::string& event) {
                                 if (event == "CarryingHeightButton") {
                                     m_leftArmProfile->SetGoal({k_armHeight1,
                                                                0.0, 0.0});
                                     m_rightArmProfile->SetGoal({k_armHeight1,
                                                                 0.0,
                                                                 0.0});
                                     return "CarryingHeight";
                                 }
                                 else if (event == "ZeroHeightButton") {
                                     return "ZeroHeight";
                                 }
                                 else {
                                     return "";
                                 }
                             };
    m_armSM.AddState(std::move(state));

    // Start arm lift
    state = std::make_unique<State>("CarryingHeight");
    state->Run = [this] {
        if (GetManualOverride()) {
            m_leftArmActuator.Set(1);
            m_rightArmActuator.Set(1);
        }
    };
    state->CheckTransition = [this] (const std::string& event) {
                                 if (AtGoal()) {
                                     return "Idle";
                                 }
                                 else {
                                     return "";
                                 }
                             };

    m_armSM.AddState(std::move(state));

    // Start arm lift
    state = std::make_unique<State>("ZeroHeight");
    state->Run = [this] {
        m_leftArmActuator.Set(-1);
        m_rightArmActuator.Set(-1);
    };
    state->CheckTransition = [this] (const std::string& event) {
                                 if (event == "LeftArmZeroed") {
                                     m_leftArmActuator.ResetEncoder();
                                     m_rightArmActuator.ResetEncoder();
                                     return "Idle";
                                 }
                                 else if (event == "RightArmZeroed") {
                                     m_leftArmActuator.ResetEncoder();
                                     m_rightArmActuator.ResetEncoder();
                                     return "Idle";
                                 }
                                 else {
                                     return "";
                                 }
                             };

    m_armSM.AddState(std::move(state));
}

void Arm::SetManualOverride(bool manual) {
    m_manual = manual;
}

bool Arm::GetManualOverride() const {
    return m_manual;
}

void Arm::SetCarryingHeight(double speed) {
    if (GetManualOverride()) {
        m_leftArmPID->Disable();
        m_rightArmPID->Disable();

        m_armHeight = speed;
    }
    else {
        m_leftArmPID->Enable();
        m_rightArmPID->Enable();

        m_leftArmPID->SetSetpoint({ 0.0, speed / m_leftArmPID->GetV(), 0.0 }); // TODO: change all of these values
        m_rightArmPID->SetSetpoint({ 0.0, speed / m_leftArmPID->GetV(), 0.0 });
    }
}

void Arm::SetArmHeight(double height) {
    std::cout << "m_topLeftLimitSwitch:  " << m_topLeftLimitSwitch.Get()
              << std::endl;
    std::cout << "m_topRightLimitSwitch:  " << m_topRightLimitSwitch.Get()
              << std::endl;
    std::cout << "m_bottomLeftLimitSwitch:  " << m_bottomLeftLimitSwitch.Get()
              << std::endl;
    std::cout << "m_bottomRightLimitSwitch:  " << m_bottomRightLimitSwitch.Get()
              << std::endl;
    std::cout << "m_rightArmActuator:  " << m_rightArmActuator.Get()
              << std::endl;
    std::cout << "m_leftArmActuator:  " << m_leftArmActuator.Get() << std::endl;

    if (GetManualOverride()) {
        if (m_topLeftLimitSwitch.Get() == false
            || m_topRightLimitSwitch.Get() == false) {
            if (height < 0) {
                m_rightArmActuator.Set(height);
                m_leftArmActuator.Set(height);
            }
            else {
                m_rightArmActuator.Set(0);
                m_leftArmActuator.Set(0);
            }
        }
        else {
            m_rightArmActuator.Set(height);
            m_leftArmActuator.Set(height);
        }

        if (m_bottomLeftLimitSwitch.Get() == false
            || m_bottomRightLimitSwitch.Get() == false) {
            if (height > 0) {
                m_rightArmActuator.Set(height);
                m_leftArmActuator.Set(height);
            }
            else {
                m_rightArmActuator.Set(0);
                m_leftArmActuator.Set(0);
            }
        }
        else {
            m_rightArmActuator.Set(height);
            m_leftArmActuator.Set(height);
        }
    }
    else {
        m_leftArmProfile->SetGoal({ height, 0, 0 });
        m_rightArmProfile->SetGoal({ height, 0, 0 });
    }
}

void Arm::SetManualCarriagePosition(int direction) {
    if (direction == 90 && m_leftCarriageLimitSwitch.Get() == false) {
        m_carriagePositionMotor.Set(0.2);
    }
    else if (direction == 270 && m_rightCarriageLimitSwitch.Get() == false) {
        m_carriagePositionMotor.Set(-0.2);
    }
    else {
        m_carriagePositionMotor.Set(0.0);
    }
}
bool Arm::AtGoal() const {
    return m_leftArmProfile->AtGoal() && m_rightArmProfile->AtGoal();
}

void Arm::ReloadPID() {
}

void Arm::SetManualWinchHeight(double speed) {
    m_winchPositionMotor.Set(speed);
}

void Arm::ResetEncoders() {
}

void Arm::UpdateState() {
    m_armSM.Run();
    m_joystickEvent.Poll(m_armSM);
    m_joystickEvent.Update();
}
