// =============================================================================
// File Name: Shooter.cpp
// Description: Provides an interface for the robot's shooter
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "Shooter.hpp"
#include <CANTalon.h>

Shooter::Shooter() {
	m_profileUpdater = new std::thread([this] {
		float rpmLeft = 0;
		float rpmRight = 0;
		while (m_updateProfile) {
			/* Derivation of RPM:
			* speed = ms per tick of gear
			* 1 / 360 = revolutions per tick
			* freq * 1000 = seconds per tick of gear
			* freq * 1000 * 60 = minutes per tick
			*
			* shooterRPM = ( m_leftShooterMotor.GetSpeed() *  5.0f * 1000.0f * 60.0f * 2.0f ) / ( 100.0f * 360.0f )
			*/
			rpmLeft = ( m_leftShooterMotor.GetSpeed() *  5.0f * 1000.0f * 60.0f * 2.0f ) / ( 100.0f * 360.0f );
			rpmRight = ( m_rightShooterMotor.GetSpeed() *  5.0f * 1000.0f * 60.0f * 2.0f ) / ( 100.0f * 360.0f );
			m_rpmFilterLeft.update(rpmLeft);
			m_rpmFilterRight.update(rpmRight);
			m_latestRPMLeft = m_rpmFilterLeft.getEstimate();
			m_latestRPMRight = m_rpmFilterRight.getEstimate();
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	});

}

void Shooter::shoot() {
	// Shoot only if motors spinning
	if(m_latestRPMLeft != 0 && m_latestRPMRight != 0) {
		m_kickBallMotor.Enable();
	}
}

void Shooter::setManualShooterPosition(double position) {
	double cur_position = 0;
	while(true) {
		if(cur_position < position) {
			m_shooterPositionMotor.Enable();
			// position+=100;
		}
		else {
			break;
		}
	}
}

void Shooter::setManualShooterSpeed(double speed){
	m_leftShooterMotor.SetControlMode(CANTalon::kPercentVbus);
	m_rightShooterMotor.SetControlMode(CANTalon::kPercentVbus);

	m_leftShooterMotor.Set(-speed);
	m_rightShooterMotor.Set(speed);
}

float Shooter::getRPMLeft(){
	return m_latestRPMLeft;
}

float Shooter::getRPMRight(){
	return m_latestRPMRight;
}

void Shooter::setRPMLeft(double speed) {
	m_leftShooterMotor.Set(-speed);
}

void Shooter::setRPMRight(double speed) {
	m_rightShooterMotor.Set(speed);
}

Shooter::~Shooter() {

}

void Shooter::updateLeftIntakeEncoderFilter( double Q , double R ) {
	m_rpmFilterLeft.setQ(Q);
	m_rpmFilterLeft.setR(R);
}

void Shooter::updateRightIntakeEncoderFilter( double Q , double R ) {
	m_rpmFilterRight.setQ(Q);
	m_rpmFilterRight.setR(R);
}
