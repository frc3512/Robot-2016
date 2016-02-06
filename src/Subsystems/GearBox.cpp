// =============================================================================
// Description: Represents a gear box with up to 3 motors and an encoder
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "GearBox.hpp"

GearBox::GearBox(int shifterChan, int motor1, int motor2, int motor3) {
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

    for (unsigned int i = 0; i < m_motors.size(); i++) {
        if (i == 0) {
            m_motors[i]->SetControlMode(CANTalon::kPercentVbus);
            m_motors[i]->SetFeedbackDevice(CANTalon::QuadEncoder);
            m_motors[i]->ConfigEncoderCodesPerRev(1);
            m_motors[i]->SetSensorDirection(m_isEncoderReversed);
            ResetEncoder();
            m_motors[i]->SelectProfileSlot(0);
            m_motors[i]->EnableControl();
        }
        else {
            // Use all other CANTalons as slaves
            m_motors[i]->SetControlMode(CANTalon::kFollower);

            // Set first CANTalon as master
            m_motors[i]->Set(motor1);
        }
    }
}

void GearBox::SetManual(float value) {
    m_motors[0]->Set(value);
}

float GearBox::GetPosition() const {
    return m_motors[0]->GetPosition() * m_distancePerPulse;
}

float GearBox::GetSpeed() const {
    return m_motors[0]->GetSpeed() * m_distancePerPulse;
}

void GearBox::SetDistancePerPulse(double distancePerPulse) {
    m_distancePerPulse = distancePerPulse;
}

void GearBox::ResetEncoder() {
    m_motors[0]->SetPosition(0);
}

void GearBox::SetInverted(bool reverse) {
    for (auto& motor : m_motors) {
        motor->SetInverted(reverse);
    }
}

bool GearBox::GetInverted() const {
    return m_motors[0]->GetInverted();
}

void GearBox::SetSensorDirection(bool reverse) {
    m_isEncoderReversed = reverse;
    m_motors[0]->SetSensorDirection(m_isEncoderReversed);
}

bool GearBox::IsEncoderReversed() const {
    return m_isEncoderReversed;
}

void GearBox::SetGear(bool gear) {
    if (m_shifter != nullptr) {
        m_shifter->Set(gear);
    }
}

bool GearBox::GetGear() const {
    if (m_shifter != nullptr) {
        return m_shifter->Get();
    }
    else {
        return false;
    }
}

CANTalon* GearBox::GetMaster() const {
    return m_motors[0].get();
}

void GearBox::PIDWrite(float output) {
    m_motors[0]->PIDWrite(output);
}

double GearBox::PIDGet() {
    if (GetPIDSourceType() == PIDSourceType::kRate) {
        return m_motors[0]->GetSpeed() * m_distancePerPulse;
    }
    else {
        return m_motors[0]->GetPosition() * m_distancePerPulse;
    }
}
