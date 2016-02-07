// =============================================================================
// Description: Provides an interface for the robot's shooter
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "Shooter.hpp"

Shooter::Shooter(){
	//m_shooterHeightPID = PIDController( 0.f, 0.f, 0.f, 0.f, 0.f, &m_shooterHeightMotor, &m_shooterHeightMotor);
	//m_shootHeightProfile = TrapezoidProfile(m_shooterHeightPID, 0.0, 0.0);
    m_leftShooterMotor.SetInverted(true);
}

void Shooter::Shoot() {
    // Shoot only if motors spinning
    if (IsBallLoaded()) {
        m_rollBallMotor.Set(.75);
    }
}

void Shooter::StopIntakeMotor() {
    m_rollBallMotor.Set(0);
}

void Shooter::Intake() {
    if (!IsBallLoaded()) {
        m_leftShooterMotor.Set(-.5);
        m_rightShooterMotor.Set(-.5);
        m_rollBallMotor.Set(-.5);
    }
}
bool Shooter::IsBallLoaded() {
    return m_intakeLimit.Get();
}
void Shooter::SetManualShooterHeight(double position) {
    m_shooterHeightMotor.SetControlMode(CANTalon::kPercentVbus);

    m_shooterHeightMotor.Set(position);
}
void Shooter::SetManualShooterSpeed(double speed) {
    m_leftShooterMotor.SetControlMode(CANTalon::kPercentVbus);
    m_rightShooterMotor.SetControlMode(CANTalon::kPercentVbus);

    m_leftShooterMotor.Set(speed);
    m_rightShooterMotor.Set(speed);
}

// FOR CURVING THE BOULDERS ONLY, REMOVE BEFORE FINAL RELEASE!
void Shooter::SetLeftShooterSpeed(double speed) {
    m_leftShooterMotor.Set(speed);
}
void Shooter::SetRightShooterSpeed(double speed) {
    m_rightShooterMotor.Set(speed);
}
/*
 * Conversion Table for calculating RPM, MAY OR MAY NOT BE ACCURATE
 ||///////////////||/////////////////||///////////////||//////////////||
 ||    S ticks    ||    1 rev       ||    1000 ms    ||    60 sec    ||
 ||    -------    ||    ---------    ||   -------    ||    ------    ||
 ||    100 ms    ||    360 ticks    ||    1 sec      ||    1 min     ||
 ||///////////////||/////////////////||///////////////||//////////////||
 */
float Shooter::GetLeftRPM() const {
    // TODO: document magic number math
    std::cout << "Left Motor Raw Output: " << m_leftShooterMotor.GetSpeed() <<
    std::endl;
    return m_leftShooterMotor.GetSpeed() *  5.0f * 1000.0f * 60.0f * 2.0f /
           (100.0f * 360.0f);
}

float Shooter::GetRightRPM() const {
    // TODO: document magic number math
    std::cout << "Right Motor Raw Output: " << m_rightShooterMotor.GetSpeed() <<
    std::endl;
    return m_rightShooterMotor.GetSpeed() *  5.0f * 1000.0f * 60.0f * 2.0f /
           (100.0f * 360.0f);
}

void Shooter::SetLeftRPM(double speed) {
    m_leftShooterMotor.Set(speed);
}

void Shooter::SetRightRPM(double speed) {
    m_rightShooterMotor.Set(speed);
}
Shooter::~Shooter() {

}
void Shooter::ReloadPID() {
}

void Shooter::ResetEncoders() {
}
