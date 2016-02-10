/*
 * Arm.cpp
 *
 *  Created on: Feb 9, 2016
 *      Author: nad
 */

#include "Arm.hpp"

Arm::Arm() {
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
