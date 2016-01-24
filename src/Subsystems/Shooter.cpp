// =============================================================================
// File Name: Shooter.cpp
// Description: Provides an interface for the robot's shooter
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "Shooter.hpp"
#include <CANTalon.h>

Shooter::Shooter() {

}

void Shooter::setManualShooterSpeed(double speed){
	m_leftShooterMotor.SetControlMode(CANTalon::kPercentVbus);
	m_rightShooterMotor.SetControlMode(CANTalon::kPercentVbus);

	m_leftShooterMotor.Set(speed);
	m_rightShooterMotor.Set(speed);


}

Shooter::~Shooter() {

}

void Shooter::reloadPID() {

}

void Shooter::resetEncoders() {

}
