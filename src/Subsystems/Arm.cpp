// =============================================================================
// Description: Provides an interface for the robot's arm
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "../WPILib/PIDController.hpp"
#include "Arm.hpp"

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

    m_leftArmActuator.GetMaster()->SetFeedbackDevice(
        CANTalon::CtreMagEncoder_Relative);
    m_rightArmActuator.GetMaster()->SetFeedbackDevice(
        CANTalon::CtreMagEncoder_Relative);

    m_leftArmActuator.SetPIDSourceType(PIDSourceType::kRate);
    m_rightArmActuator.SetPIDSourceType(PIDSourceType::kRate);

    // Sets encoder type
    m_leftArmActuator.GetMaster()->SetFeedbackDevice(
        CANTalon::CtreMagEncoder_Relative);
    m_rightArmActuator.GetMaster()->SetFeedbackDevice(
        CANTalon::CtreMagEncoder_Relative);

    m_joystickEvent.RegisterButtonEvent("PressedButton", k_armStickPort, 4,
                                        true); // TODO: change port number
    m_joystickEvent.RegisterButtonEvent("ReleasedButton", k_armStickPort, 4,
                                        false); // TODO: change port number
    m_joystickEvent.RegisterButtonEvent("PressedButton", k_armStickPort, 4,
                                        true); // TODO: change port number
    m_dioEvent.RegisterInputEvent("ArmBottomButton", k_intakeLimitPin,
                                  true,
                                  false, m_armSM);
    m_dioEvent.RegisterInputEvent("ArmUp", k_armBottomLimitPin,
                                  true,
                                  false, m_armSM);

    // Idle
    auto state = std::make_unique<State>("Idle");
    state->Entry = [this] {
        m_leftArmActuator.Set(0);
        m_rightArmActuator.Set(0);
    };
    state->CheckTransition = [this] (const std::string& event) {
                                 if (event == "PressedButton" &&
                                     m_leftArmActuator.GetPosition() != 1) {
                                     return "CarryingHeight";
                                 }
                                 else {
                                     return "Idle";
                                 }
                             };
    m_armSM.AddState(std::move(state));

    // Start arm lift
    state = std::make_unique<State>("CarryingHeight");
    state->Run = [this] {
        m_leftArmActuator.Set(1);
        m_rightArmActuator.Set(1);
    };
    state->CheckTransition = [this] (const std::string& event) {
                                 if (event == "armBottomButton") {
                                     return "Idle";
                                 }
                                 else if (event == "armTopButton") {
                                     return "Idle";
                                 }
                             };

    state->CheckTransition = [this] (const std::string& event) {
                                 if (event == "PressedButton" &&
                                     m_leftArmActuator.GetPosition() != 0) { // TODO: change "o"
                                     return "ZeroHeight";
                                 }
                                 else {
                                     return "Idle";
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
                                 if (event == "armBottomButton") {
                                     return "Idle";
                                 }
                                 else {
                                     return "";
                                 }
                             };

    m_armSM.AddState(std::move(state));
}

bool Arm::GetManualOverride() {
    return m_manual;
}

void Arm::SetCarryingHeight(double speed) {
    if (GetManualOverride()) {
        m_leftArmPID->Disable();
        m_rightArmPID->Disable();

        m_manualArmSpeed = speed;
    }
    else {
        m_leftArmPID->Enable();
        m_rightArmPID->Enable();

        m_leftArmPID->SetSetpoint({ 0.0, speed / m_leftArmPID->GetV(), 0.0 }); // TODO: change all of these values
        m_rightArmPID->SetSetpoint({ 0.0, speed / m_leftArmPID->GetV(), 0.0 });
    }
}

void Arm::SetClimbHeight() {
}

void Arm::SetManualArmHeight(double height) {
    m_leftArmActuator.Set(height);
    m_rightArmActuator.Set(height);
}

void Arm::SetManualCarriagePosition(double position) {
    if (armStick.GetPOV() == 90) {
        m_carriagePositionMotor.Set(0.2);
    }
    else if (armStick.GetPOV() == 270) {
        m_carriagePositionMotor.Set(-0.2);
    }
    if (m_leftCarriageLimitSwitch.Get()) {
        m_carriagePositionMotor.Set(0);
    }
    else if (m_rightCarriageLimitSwitch.Get()) {
        m_carriagePositionMotor.Set(0);
    }
}

void Arm::SetPIDArmHeight(double height) {
}

void Arm::ReloadPID() {
}

void Arm::ResetEncoders() {
}

void Arm::UpdateState() {
    m_armSM.Run();
    m_joystickEvent.Poll(m_armSM);
    m_joystickEvent.Update();
}
