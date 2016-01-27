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
			rpmLeft = ( m_leftShooterMotor.GetSpeed() *  5.0f * 1000.0f * 60.0f * 2.0f ) / ( 100.0f * 360.0f );
			rpmRight = ( m_leftShooterMotor.GetSpeed() *  5.0f * 1000.0f * 60.0f * 2.0f ) / ( 100.0f * 360.0f );
			m_rpmFilter.update(getRPM());
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	});

}

void Shooter::setManualShooterSpeed(double speed){
	m_leftShooterMotor.SetControlMode(CANTalon::kPercentVbus);
	m_rightShooterMotor.SetControlMode(CANTalon::kPercentVbus);

	m_leftShooterMotor.Set(-speed);
	m_rightShooterMotor.Set(speed);


}

float Shooter::getRPM(){
    /* Derivation of RPM:
     * speed = ms per tick of gear
     * 1 / 360 = revolutions per tick
     * freq * 1000 = seconds per tick of gear
     * freq * 1000 * 60 = minutes per tick
     *
     * shooterRPM = ( m_leftShooterMotor.GetSpeed() *  5.0f * 1000.0f * 60.0f * 2.0f ) / ( 100.0f * 360.0f )
     */




}


Shooter::~Shooter() {

}

void Shooter::reloadPID() {

}

void Shooter::resetEncoders() {

}
