// =============================================================================
// File Name: Shooter.cpp
// Description: Provides an interface for the robot's shooter
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "Shooter.hpp"
#include <CANTalon.h>

Shooter::Shooter() {
}

void Shooter::shoot() {
    // Shoot only if motors spinning
    if (m_latestRPMLeft != 0 && m_latestRPMRight != 0) {
        m_kickBallMotor.Enable();
    }
}

void Shooter::setManualShooterPosition(double position) {
	m_shooterPositionMotor.SetControlMode(CANTalon::kPercentVbus);

	m_shooterPositionMotor.Set(position);

}
void Shooter::setManualShooterSpeed(double speed) {
    m_leftShooterMotor.SetControlMode(CANTalon::kPercentVbus);
    m_rightShooterMotor.SetControlMode(CANTalon::kPercentVbus);

    m_leftShooterMotor.Set(-speed);
    m_rightShooterMotor.Set(speed);
}

// FOR CURVING THE BOULDERS ONLY, REMOVE BEFORE FINAL RELEASE!
void Shooter::setLeftShooterSpeed(double speed) {
    m_leftShooterMotor.SetControlMode(CANTalon::kPercentVbus);
    m_leftShooterMotor.Set(-speed);
}
void Shooter::setRightShooterSpeed(double speed) {
    m_rightShooterMotor.SetControlMode(CANTalon::kPercentVbus);
    m_rightShooterMotor.Set(speed);
}

float Shooter::getRPMLeft() {
    float leftRPM;

    leftRPM = (m_leftShooterMotor.GetSpeed() *  5.0f * 1000.0f * 60.0f * 2.0f) /
              (100.0f * 360.0f);

    return leftRPM;
}

float Shooter::getRPMRight() {
    float rightRPM;

    rightRPM =
        (m_rightShooterMotor.GetSpeed() *  5.0f * 1000.0f * 60.0f * 2.0f) /
        (100.0f * 360.0f);

    return rightRPM;
}

void Shooter::setRPMLeft(double speed) {
    m_leftShooterMotor.Set(speed);
}

void Shooter::setRPMRight(double speed) {
    m_rightShooterMotor.Set(speed);
}

void Shooter::reloadPID() {
}

void Shooter::resetEncoders() {
}

Shooter::~Shooter() {
}

