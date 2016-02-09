// =============================================================================
// Description: Provides an interface for the robot's shooter
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "Shooter.hpp"

Shooter::Shooter() {
    // m_shooterHeightPID = PIDController( 0.f, 0.f, 0.f, 0.f, 0.f, &m_shooterHeightMotor, &m_shooterHeightMotor);
    m_leftShootGrbx.SetInverted(true);
    m_leftShootGrbx.SetPIDSourceType(PIDSourceType::kRate);
    m_rightShootGrbx.SetPIDSourceType(PIDSourceType::kRate);

    auto state = std::make_unique<State>("IDLE");
    state->Entry = [this] {
    m_leftShootGrbx.Set(0);
    m_rightShootGrbx.Set(0);
    m_rollBallGrbx.Set(0);
     };

    m_intakeSM.AddState(std::move(state));
    m_intakeSM.SetState("IDLE");

    state = std::make_unique<State>("START_INTAKE");
    state->Entry = [this] {
        m_leftShootGrbx.Set(-.5);
        m_rightShootGrbx.Set(-.5);
        m_rollBallGrbx.Set(-.5);
    };
    state->Transition = [this] {
        if (IsBallLoaded()) {
            return "IDLE";
        }
        else {
            return "";
        }
    };

    m_intakeSM.AddState(std::move(state));

}

void Shooter::Shoot() {
    // Shoot only if motors spinning
    if (IsBallLoaded()) {
        m_rollBallGrbx.Set(.75);
    }
}

void Shooter::StopIntakeMotor() {
    m_rollBallGrbx.Set(0);
}
void Shooter::StartIntake() {
	if (m_intakeSM.GetState() == "IDLE") {
		m_intakeSM.SetState("START_INTAKE");
	}
}

bool Shooter::IsBallLoaded() const {
    return m_intakeLimit.Get();
}

bool Shooter::ToggleManualOverride() {
    m_manual = !m_manual;
}

bool Shooter::GetManualOverride() const {
    return m_manual;
}

void Shooter::SetManualShooterHeight(double position) {
    m_shooterHeightGrbx.Set(position);
}

void Shooter::SetPIDShooterSpeed(double speed) {
    m_leftShootPID.Enable();
    m_rightShootPID.Enable();
}

void Shooter::SetManualShooterSpeed(double speed) {
    m_leftShootPID.Disable();
    m_rightShootPID.Disable();

    m_leftShootGrbx.Set(speed);
    m_rightShootGrbx.Set(speed);
}

// TODO: FOR CURVING THE BOULDERS ONLY, REMOVE BEFORE FINAL RELEASE!
void Shooter::SetLeftShooterSpeed(double speed) {
    m_leftShootGrbx.Set(speed);
}

void Shooter::SetRightShooterSpeed(double speed) {
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
    std::cout << "Left Motor Raw Output: " << m_leftShootGrbx.GetSpeed() <<
        std::endl;
    return m_leftShootGrbx.GetSpeed() * 5.0f * 1000.0f * 60.0f * 2.0f /
           (100.0f * 360.0f);
}

float Shooter::GetRightRPM() const {
    // TODO: document magic number math
    std::cout << "Right Motor Raw Output: " << m_rightShootGrbx.GetSpeed() <<
        std::endl;
    return m_rightShootGrbx.GetSpeed() * 5.0f * 1000.0f * 60.0f * 2.0f /
           (100.0f * 360.0f);
}

void Shooter::ReloadPID() {
}

void Shooter::ResetEncoders() {
}
