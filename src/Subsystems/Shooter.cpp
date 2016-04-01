// =============================================================================
// Description: Provides an interface for the robot's shooter
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "Shooter.hpp"

Shooter::Shooter() {
    m_leftShootGrbx = std::make_shared<GearBox>(-1, -1, -1, k_leftShooterID);
    m_rightShootGrbx = std::make_shared<GearBox>(-1, -1, -1, k_rightShooterID);

    m_leftShootFilter = LinearDigitalFilter::MovingAverage(m_leftShootGrbx, 5);
    m_rightShootFilter =
        LinearDigitalFilter::MovingAverage(m_rightShootGrbx, 5);

    m_shooterHeightPID =
        std::make_shared<LeverPIDController>(k_heightShooterP,
                                             k_heightShooterI,
                                             k_heightShooterD,
                                             k_heightShooterV,
                                             k_heightShooterA,
                                             k_heightShooterF,
                                             &m_shooterHeightGrbx,
                                             &m_shooterHeightGrbx,
                                             0.02);

    m_shooterHeightGrbx.SetSensorDirection(true);
    m_rightShootGrbx->SetSensorDirection(true);

    m_shooterHeightProfile = std::make_shared<TrapezoidProfile>(
        m_shooterHeightPID,
        k_shooterHeightMaxSpeed,
        k_shooterTimeMaxV);

    m_leftShootGrbx->SetPIDSourceType(PIDSourceType::kRate);
    m_rightShootGrbx->SetPIDSourceType(PIDSourceType::kRate);

    // m_leftShootGrbx->SetDistancePerPulse(k_shooterDpP);
    // m_rightShootGrbx->SetDistancePerPulse(k_shooterDpP);
    m_shooterHeightGrbx.SetDistancePerPulse(k_shooterHeightDpP);

    // Sets encoder type
    m_leftShootGrbx->SetFeedbackDevice(
        CANTalon::CtreMagEncoder_Relative);
    m_rightShootGrbx->SetFeedbackDevice(
        CANTalon::CtreMagEncoder_Relative);
    m_shooterHeightGrbx.SetFeedbackDevice(
        CANTalon::CtreMagEncoder_Relative);

    m_shooterHeightPID->Enable();

    m_joystickTimer.Start();

    m_joystickEvent.RegisterButtonEvent("PressedIntakeButton",
                                        k_shootStickPort, 2, true);
    m_joystickEvent.RegisterButtonEvent("ReleasedIntakeButton",
                                        k_shootStickPort, 2, false);
    m_joystickEvent.RegisterButtonEvent("PressedShooterButton",
                                        k_shootStickPort, 1, true);
    m_dioEvent.RegisterInputEvent("BallLoaded",
                                  k_shooterIntakeLimitChannel,
                                  true,
                                  false,
                                  m_shootSM);
    m_dioEvent.RegisterInputEvent("ShooterZeroed",
                                  k_shooterBottomLimitChannel,
                                  true,
                                  false,
                                  m_shootSM);

    // Idle
    auto state = std::make_unique<State>("Idle");
    state->Entry = [this] {
        m_leftShootGrbx->Set(0);
        m_rightShootGrbx->Set(0);
        m_rollBallGrbx.Set(0);
        m_armIntakeGrbx.Set(0);
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
        // m_rollBallGrbx.Set(-1);
        m_armIntakeGrbx.Set(-1);
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
        m_leftShootGrbx->Set(0.85);
        m_rightShootGrbx->Set(0.85);
    };
    state->CheckTransition = [this] (const std::string& event) {
                                 if (event == "ShootTimer") {
                                     return "Shoot";
                                 }
                                 else {
                                     return "";
                                 }
                             };
    m_shootSM.AddState(std::move(state));

    // Shooter
    state = std::make_unique<State>("Shoot");
    state->Entry = [this] {
        m_timerEvent.Reset();
    };
    state->Run = [this] {
        m_rollBallGrbx.Set(1);
        m_leftShootGrbx->Set(0.85);
        m_rightShootGrbx->Set(0.85);
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

double Shooter::GetShooterHeight() const {
    return m_shooterHeightGrbx.GetPosition();
}

double Shooter::GetShooterHeightRaw() const {
    return m_shooterHeightGrbx.Get();
}

PIDState Shooter::GetShooterHeightSetpoint() const {
    return m_shooterHeightPID->GetSetpoint();
}
void Shooter::SetManualOverride(bool manual) {
    m_manual = manual;
}

void Shooter::UpdateState() {
    m_shootSM.Run();
    m_joystickEvent.Poll(m_shootSM);
    m_joystickEvent.Update();
    m_timerEvent.Poll(m_shootSM);
    m_dioEvent.Poll(m_shootSM);
    m_dioEvent.Update();
}

bool Shooter::GetManualOverride() const {
    return m_manual;
}

void Shooter::SetShooterHeight(double height, bool increment) {
    if ((m_shooterHeight.displacement >= k_shooterHeightMax && height > 0) ||
        (m_shooterHeight.displacement <= k_shooterHeightMin && height < 0)) {
        /* If the current position is past the soft limit and motor is rotating
         * in that direction
         */
        m_shooterHeight.velocity = 0.0;
        if (m_shooterHeightPID->GetSetpoint() != m_shooterHeight) {
            m_shooterHeightPID->SetSetpoint(m_shooterHeight);
        }
    }
    else {
        if (increment == true) {
            m_shooterHeight.velocity = height * k_shooterHeightMaxSpeed;
            m_shooterHeight.displacement += m_shooterHeight.velocity *
                                            m_joystickTimer.Get();
            if (m_shooterHeightPID->GetSetpoint() != m_shooterHeight) {
                m_shooterHeightPID->SetSetpoint(m_shooterHeight);
            }
        }
        else {
            m_shooterHeightPID->SetSetpoint(PIDState(height, 0, 0));
            /*m_shooterHeightProfile->SetGoal(PIDState(height, 0, 0),
             *                               PIDState(GetShooterHeight(), 0, 0));*/
        }
    }

    m_joystickTimer.Reset();

    // m_shooterHeightGrbx.Set(height);
}

void Shooter::SetShooterSpeed(double speed) {
    m_manualShooterSpeed = speed;
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
