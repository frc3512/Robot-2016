// =============================================================================
// Description: Provides an interface for the robot's shooter
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "Shooter.hpp"

Shooter::Shooter() {
    m_leftShootGrbx = std::make_shared<GearBox>(-1, k_leftShooterID);
    m_rightShootGrbx = std::make_shared<GearBox>(-1, k_rightShooterID);

    m_leftShootFilter = LinearDigitalFilter::MovingAverage(m_leftShootGrbx, 5);
    m_rightShootFilter =
        LinearDigitalFilter::MovingAverage(m_rightShootGrbx, 5);

    m_leftShootPID =
        std::make_shared<PIDController>(0.f, 0.f, 0.f, 0.f, 0.f,
                                        &m_leftShootFilter,
                                        m_leftShootGrbx.get());
    m_rightShootPID =
        std::make_shared<PIDController>(0.f, 0.f, 0.f, 0.f, 0.f,
                                        &m_rightShootFilter,
                                        m_rightShootGrbx.get());
    m_shooterHeightPID =
        std::make_shared<LeverPIDController>(0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
                                             &m_shooterHeightGrbx,
                                             &m_shooterHeightGrbx);
    m_shootHeightProfile = std::make_shared<TrapezoidProfile>(
        m_shooterHeightPID, 0.0, 0.0);

    m_leftShootGrbx->SetInverted(true);
    m_leftShootGrbx->SetPIDSourceType(PIDSourceType::kRate);
    m_rightShootGrbx->SetPIDSourceType(PIDSourceType::kRate);

    /* Distance per pulse unit conversions:
     * X ticks   1 revolution   60 sec
     * ------- * ------------ * ------
     *  1 sec     360 ticks     1 min
     *
     * Shooter has 1:1 gear ratio, so no conversion between driving and driven
     * RPMs needed
     */
    m_leftShootGrbx->SetDistancePerPulse(60.0 / 360.0);
    m_rightShootGrbx->SetDistancePerPulse(60.0 / 360.0);

    // Sets encoder type
    m_leftShootGrbx->GetMaster()->SetFeedbackDevice(
        CANTalon::CtreMagEncoder_Relative);
    m_rightShootGrbx->GetMaster()->SetFeedbackDevice(
        CANTalon::CtreMagEncoder_Relative);
    m_shooterHeightGrbx.GetMaster()->SetFeedbackDevice(
        CANTalon::CtreMagEncoder_Relative);


    m_joystickEvent.RegisterButtonEvent("PressedIntakeButton",
                                        k_shootStickPort, 2, true);
    m_joystickEvent.RegisterButtonEvent("ReleasedIntakeButton",
                                        k_shootStickPort, 2, false);
    m_joystickEvent.RegisterButtonEvent("PressedShooterButton",
                                        k_shootStickPort, 1, true);
    m_dioEvent.RegisterInputEvent("BallLoaded", k_intakeLimitPin, true, false,
                                  m_shootSM);
    m_dioEvent.RegisterInputEvent("ShooterZeroed",
                                  k_bottomLimitPin,
                                  true,
                                  false,
                                  m_shootSM);

    // Idle
    auto state = std::make_unique<State>("Idle");
    state->Entry = [this] {
        m_leftShootGrbx->Set(0);
        m_rightShootGrbx->Set(0);
        m_rollBallGrbx.Set(0);
    };
    state->CheckTransition = [this] (const std::string& event) {
                                 if (event == "PressedIntakeButton") {
                                     return "StartIntake";
                                 }
                                 else if (event == "PressedShooterButton") {
                                     return "SpinningUpMotors";
                                 }
                                 else if (event == "ShooterZeroed") {
                                     m_shooterHeightGrbx.ResetEncoder();
                                     return "";
                                 }
                                 else {
                                     return "";
                                 }
                             };
    m_shootSM.AddState(std::move(state));

    // Intake
    state = std::make_unique<State>("StartIntake");
    state->Run = [this] {
        m_rollBallGrbx.Set(-1);
        m_leftShootGrbx->Set(-1);
        m_rightShootGrbx->Set(-1);
    };
    state->CheckTransition = [] (const std::string& event) {
                                 if (event == "ReleasedIntakeButton") {
                                     return "Idle";
                                 }
                                 else if (event == "BallLoaded") {
                                     return "Idle";
                                 }

                                 else {
                                     return "";
                                 }
                             };
    m_shootSM.AddState(std::move(state));

    state = std::make_unique<State>("SpinningUpMotors");
    state->Entry = [this] {
        m_timerEvent.Reset();
    };
    state->Run = [this] {
        m_leftShootGrbx->Set(m_manualShooterSpeed);
        m_rightShootGrbx->Set(m_manualShooterSpeed);
    };
    state->CheckTransition = [this] (const std::string& event) {
                                 if (event == "ShootTimer") {
                                     return "Shoot";
                                 }
                                 else {
                                     return "";
                                 }
                             };

    // Shooter
    state = std::make_unique<State>("Shoot");
    state->Entry = [this] {
        m_timerEvent.Reset();
    };
    state->Run = [this] {
        m_rollBallGrbx.Set(1);
        m_leftShootGrbx->Set(m_manualShooterSpeed);
        m_rightShootGrbx->Set(m_manualShooterSpeed);
    };
    state->CheckTransition = [this] (const std::string& event) {
                                 if (event == "ShootTimer") {
                                     return "Idle";
                                 }
                                 else {
                                     return "";
                                 }
                             };
    m_shootSM.AddState(std::move(state));
    m_shootSM.EnableDebug(true);
}


int32_t Shooter::GetShootHeightValue() const {
    return m_shooterHeightGrbx.Get();
}

void Shooter::ToggleManualOverride() {
    m_manual = !m_manual;
}

void Shooter::UpdateState() {
    m_shootSM.Run();
    m_joystickEvent.Poll(m_shootSM);
    m_joystickEvent.Update();
    m_timerEvent.Poll(m_shootSM);
}

bool Shooter::GetManualOverride() const {
    return m_manual;
}

void Shooter::SetShooterHeight(double height) {
    if (GetManualOverride()) {
        m_shooterHeightGrbx.Set(height);
    }
    else {
        m_shootHeightProfile->SetGoal({height, 0.0, 0.0});
    }
}

void Shooter::SetShooterSpeed(double speed) {
    if (GetManualOverride()) {
        m_leftShootPID->Disable();
        m_rightShootPID->Disable();

        m_manualShooterSpeed = speed;
    }
    else {
        m_leftShootPID->Enable();
        m_rightShootPID->Enable();

        m_leftShootPID->SetSetpoint({0.0, speed / m_leftShootPID->GetV(), 0.0});
        m_rightShootPID->SetSetpoint({0.0, speed / m_leftShootPID->GetV(),
                                      0.0});
    }
}

float Shooter::GetLeftRPM() const {
    return m_leftShootGrbx->GetSpeed();
}

float Shooter::GetRightRPM() const {
    return m_rightShootGrbx->GetSpeed();
}

void Shooter::ReloadPID() {
}

void Shooter::ResetEncoders() {
}
