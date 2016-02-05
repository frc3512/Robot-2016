// =============================================================================
// Description: Represents a gear box with up to 3 motors and an encoder
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "GearBoxBase.hpp"

GearBoxBase::GearBoxBase(int shifterChan,
                         int encA,
                         int encB,
                         int motor1,
                         int motor2,
                         int motor3) {
    (void) encA;
    (void) encB;

    if (shifterChan != -1) {
        m_shifter = std::make_unique<Solenoid>(shifterChan);
    }
    else {
        m_shifter = nullptr;
    }

    // Create motor controllers of specified template type
    m_motors.emplace_back(std::make_unique<CANTalon>(motor1));
    if (motor2 != -1) {
        m_motors.emplace_back(std::make_unique<CANTalon>(motor2));
    }
    if (motor3 != -1) {
        m_motors.emplace_back(std::make_unique<CANTalon>(motor3));
    }
}

void GearBoxBase::SetMotorReversed(bool reverse) {
    for (auto& motor : m_motors) {
        motor->SetInverted(reverse);
    }
}

bool GearBoxBase::IsMotorReversed() const {
    return m_motors[0]->GetInverted();
}

bool GearBoxBase::IsEncoderReversed() const {
    return m_isEncoderReversed;
}

void GearBoxBase::SetGear(bool gear) {
    if (m_shifter != nullptr) {
        m_shifter->Set(gear);
    }
}

bool GearBoxBase::GetGear() const {
    if (m_shifter != nullptr) {
        return m_shifter->Get();
    }
    else {
        return false;
    }
}
