// =============================================================================
// Description: Provides an interface for the robot's arm
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "Arm.hpp"
#include "../MotionProfile/TrapezoidProfile.hpp"
#include "../WPILib/PIDController.hpp"

Arm::Arm() {
    m_leftArmPID = std::make_shared<PIDController>(0.f, 0.f, 0.f, 0.f, 0.f,
                                                   &m_leftArmActuator,
                                                   &m_leftArmActuator);
    m_leftArmProfile = std::make_shared<TrapezoidProfile>(m_leftArmPID,
                                                          0.0, 0.0);
}

void Arm::SetManualArmHeight(double height) {
    m_leftArmActuator.Set(height);
}

void Arm::SetManualCarriagePosition(double position) {
    m_carriagePositionMotor.Set(position);
}

void Arm::SetPIDArmHeight(double height) {
}

void Arm::ReloadPID() {
}

void Arm::ResetEncoders() {
}

void Arm::UpdateState() {
}
