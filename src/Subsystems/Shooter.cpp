// =============================================================================
// Description: Provides an interface for the robot's shooter
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "Shooter.hpp"

Shooter::Shooter() {
    m_leftShootPID =
        std::make_shared<PIDController>(0.f, 0.f, 0.f, 0.f, 0.f,
                                        &m_leftShootGrbx, &m_leftShootGrbx);
    m_rightShootPID =
        std::make_shared<PIDController>(0.f, 0.f, 0.f, 0.f, 0.f,
                                        &m_rightShootGrbx, &m_rightShootGrbx);
    m_shooterHeightPID =
        std::make_shared<LeverPIDController>(0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
                                             &m_shooterHeightGrbx,
                                             &m_shooterHeightGrbx);
    m_shootHeightProfile = std::make_shared<TrapezoidProfile>(
        m_shooterHeightPID, 0.0, 0.0);

    m_leftShootGrbx.SetInverted(true);
    m_leftShootGrbx.SetPIDSourceType(PIDSourceType::kRate);
    m_rightShootGrbx.SetPIDSourceType(PIDSourceType::kRate);

    m_joystickEvent.RegisterButtonEvent("PressedIntakeButton",
                                        k_shootStickPort, 1, true);
    m_joystickEvent.RegisterButtonEvent("PressedShooterButton",
                                        k_shootStickPort, 2, true);
    m_dioEvent.RegisterInputEvent("BallLoaded", k_intakeLimitPin, true, false,
                                  m_shootSM);
    m_dioEvent.RegisterInputEvent("ShooterZeroed", k_bottomLimitPin, true, false,
                                  m_shootSM);

    // Idle
    auto state = std::make_unique<State>("Idle");
    state->Entry = [this] {
        m_leftShootGrbx.Set(0);
        m_rightShootGrbx.Set(0);
        m_rollBallGrbx.Set(0);
    };
    state->CheckTransition = [] (const std::string& event) {
                                 if (event == "PressedShooterButton") {
                                     return "StartIntake";
                                 }
                                 else {
                                     return "";
                                 }
                             };
    m_intakeSM.AddState(std::move(state));

    // Start intake
    state = std::make_unique<State>("StartIntake");
    state->Run = [this] {
        m_rollBallGrbx.Set(-.5);
    };
    state->CheckTransition = [] (const std::string& event) {
                                 if (event == "BallLoaded") {
                                     return "Idle";
                                 }
                                 else {
                                     return "";
                                 }
                             };
    m_intakeSM.AddState(std::move(state));

    // Idle
    state = std::make_unique<State>("Idle");
    state->CheckTransition = [this] (const std::string& event) {
                                 if (event == "PressedIntakeButton") {
                                     return "StartIntake";
                                 }
                                 else if (event == "ShooterZeroed")  {
									 m_shooterHeightGrbx.ResetEncoder();
									 return "";
							     }
                                 else {
                                     return "";
                                 }
                             };
    m_shootSM.AddState(std::move(state));

    // Start intake and shooter
    state = std::make_unique<State>("StartIntakeAndShooter");
    state->Entry = [this] {
        m_timerEvent.Reset();
    };
    state->Run = [this] {
        m_leftShootGrbx.Set(-.5);
        m_rightShootGrbx.Set(-.5);
        m_rollBallGrbx.Set(.75);
    };
    state->CheckTransition = [this] (const std::string& event) {
                                 if (event == "ShootTimer") {
                                     return "Idle";
                                 }
                                 else if (event == "ShooterZeroed") {
                                	 m_shooterHeightGrbx.ResetEncoder();
                                	 return "";
                                 }
                                 else {
                                     return "";
                                 }
                             };
    state->Exit = [this] {
        // Stop intake motor
        m_rollBallGrbx.Set(0);
    };
    m_shootSM.AddState(std::move(state));
}

int32_t Shooter::GetShootHeightValue() const {
	return m_shooterHeightGrbx.Get();
}

void Shooter::ToggleManualOverride() {
    m_manual = !m_manual;
}

void Shooter::UpdateState() {
    m_intakeSM.Run();
    m_joystickEvent.Poll(m_intakeSM);
    m_timerEvent.Poll(m_intakeSM);

    m_shootSM.Run();
    m_joystickEvent.Poll(m_shootSM);
    m_timerEvent.Poll(m_intakeSM);
}

bool Shooter::GetManualOverride() const {
    return m_manual;
}

void Shooter::SetManualShooterHeight(double position) {
    m_shooterHeightGrbx.Set(position);
}

void Shooter::SetPIDShooterSpeed(double speed) {
    m_leftShootPID->Enable();
    m_rightShootPID->Enable();
}
/*
 *  void Shooter::SetShooterHeight(double height) {
 *   if (m_manual == false) {
 *        m_shootHeightProfile->SetGoal(height);
 *   }
 *  }
 */
void Shooter::SetManualShooterSpeed(double speed) {
    m_leftShootPID->Disable();
    m_rightShootPID->Disable();

    m_leftShootGrbx.Set(speed);
    m_rightShootGrbx.Set(speed);
}

/* Conversion table for calculating RPM
 * ||///////////////||/////////////////||///////////////||//////////////||
 * ||    S ticks    ||    1 rev        ||    1000 ms    ||    60 sec    ||
 * ||    -------    ||    ---------    ||    -------    ||    ------    ||
 * ||    100 ms     ||    360 ticks    ||    1 sec      ||    1 min     ||
 * ||///////////////||/////////////////||///////////////||//////////////||
 */
float Shooter::GetLeftRPM() const {
    // TODO: document magic number math
    //std::cout << "Left Motor Raw Output: " << m_leftShootGrbx.GetSpeed() <<
    //    std::endl;
    return m_leftShootGrbx.GetSpeed() * 5.0f * 1000.0f * 60.0f * 2.0f /
           (100.0f * 360.0f);
}

float Shooter::GetRightRPM() const {
    // TODO: document magic number math
    //std::cout << "Right Motor Raw Output: " << m_rightShootGrbx.GetSpeed() <<
    //    std::endl;
    return m_rightShootGrbx.GetSpeed() * 5.0f * 1000.0f * 60.0f * 2.0f /
           (100.0f * 360.0f);
}

void Shooter::ReloadPID() {
}

void Shooter::ResetEncoders() {
}
