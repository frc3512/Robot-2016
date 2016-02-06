// =============================================================================
// Description: Provides an interface for the robot's shooter
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "Shooter.hpp"

Shooter::Shooter() {
}

void Shooter::Shoot() {
    // Shoot only if motors spinning
    if (m_latestRPMLeft != 0 && m_latestRPMRight != 0) {
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

    m_leftShooterMotor.Set(-speed);
    m_rightShooterMotor.Set(speed);
}

// FOR CURVING THE BOULDERS ONLY, REMOVE BEFORE FINAL RELEASE!
void Shooter::SetLeftShooterSpeed(double speed) {
    m_leftShooterMotor.SetControlMode(CANTalon::kPercentVbus);
    m_leftShooterMotor.Set(-speed);
}
void Shooter::SetRightShooterSpeed(double speed) {
    m_rightShooterMotor.SetControlMode(CANTalon::kPercentVbus);
    m_rightShooterMotor.Set(speed);
}

float Shooter::GetRPMLeft() {
    float leftRPM;

    leftRPM = (m_leftShooterMotor.GetSpeed() *  5.0f * 1000.0f * 60.0f * 2.0f) /
              (100.0f * 360.0f);

    return leftRPM;
}

float Shooter::GetRPMRight() {
    float rightRPM;

    rightRPM =
        (m_rightShooterMotor.GetSpeed() *  5.0f * 1000.0f * 60.0f * 2.0f) /
        (100.0f * 360.0f);

    return rightRPM;
}

void Shooter::SetRPMLeft(double speed) {
    m_leftShooterMotor.Set(speed);
}

void Shooter::SetRPMRight(double speed) {
    m_rightShooterMotor.Set(speed);
}

void Shooter::ReloadPID() {
}

void Shooter::ResetEncoders() {
}
