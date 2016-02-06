// =============================================================================
// Description: Provides an interface for the robot's shooter
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "Shooter.hpp"

Shooter::Shooter() {
    m_leftShooterMotor.SetInverted(true);
}

void Shooter::Shoot() {
    // Shoot only if motors spinning
    if (m_latestLeftRPM != 0 && m_latestRightRPM != 0) {
        m_kickBallMotor.Enable();
    }
}

void Shooter::SetManualShooterPosition(double position) {
    m_shooterPositionMotor.SetControlMode(CANTalon::kPercentVbus);

    m_shooterPositionMotor.Set(position);
}
void Shooter::SetManualShooterSpeed(double speed) {
    m_leftShooterMotor.SetControlMode(CANTalon::kPercentVbus);
    m_rightShooterMotor.SetControlMode(CANTalon::kPercentVbus);

    m_rightShooterMotor.Set(speed);
}

// FOR CURVING THE BOULDERS ONLY, REMOVE BEFORE FINAL RELEASE!
void Shooter::SetLeftShooterSpeed(double speed) {
    m_leftShooterMotor.Set(-speed);
}
void Shooter::SetRightShooterSpeed(double speed) {
    m_rightShooterMotor.Set(speed);
}

float Shooter::GetLeftRPM() const {
    // TODO: document magic number math
    return m_leftShooterMotor.GetSpeed() *  5.0f * 1000.0f * 60.0f * 2.0f /
           (100.0f * 360.0f);
}

float Shooter::GetRightRPM() const {
    // TODO: document magic number math
    return m_rightShooterMotor.GetSpeed() *  5.0f * 1000.0f * 60.0f * 2.0f /
           (100.0f * 360.0f);
}

void Shooter::SetLeftRPM(double speed) {
    m_leftShooterMotor.Set(speed);
}

void Shooter::SetRightRPM(double speed) {
    m_rightShooterMotor.Set(speed);
}

void Shooter::ReloadPID() {
}

void Shooter::ResetEncoders() {
}
