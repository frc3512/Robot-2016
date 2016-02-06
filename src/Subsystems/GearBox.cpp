// =============================================================================
// Description: Represents a gear box with up to 3 motors and an encoder
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "GearBox.hpp"

GearBox::GearBox(int shifterChan, int motor1, int motor2, int motor3) :
    GearBoxBase(shifterChan, -1, -1, motor1, motor2, motor3) {
    for (unsigned int i = 0; i < m_motors.size(); i++) {
        if (i == 0) {
            m_motors[i]->SetControlMode(CANTalon::kPercentVbus);
            m_motors[i]->SetFeedbackDevice(CANTalon::QuadEncoder);
            m_motors[i]->ConfigEncoderCodesPerRev(1);
            m_motors[i]->SetSensorDirection(m_isEncoderReversed);
            ResetEncoder();
            SetProfile(false);
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

void GearBox::SetSetpoint(PIDState setpoint) {
    m_motors[0]->SetControlMode(CANTalon::kPosition);

    m_setpoint = setpoint.displacement / m_distancePerPulse;
    m_motors[0]->Set(m_setpoint);
}

PIDState GearBox::GetSetpoint() const {
    return {m_setpoint* m_distancePerPulse, 0.0, 0.0};
}

void GearBox::SetManual(float value) {
    m_motors[0]->SetControlMode(CANTalon::kPercentVbus);
    m_motors[0]->Set(value);
}

float GearBox::Get(Grbx::PIDMode mode) const {
    if (mode == Grbx::Position) {
        return m_motors[0]->GetPosition() * m_distancePerPulse;
    }
    else if (mode == Grbx::Speed) {
        return m_motors[0]->GetEncVel() * m_distancePerPulse;
    }
    else if (mode == Grbx::Raw) {
        return m_motors[0]->Get();
    }

    return 0.f;
}

void GearBox::SetPID(float p, float i, float d) {
    m_motors[0]->SetPID(p, i, d);
}

void GearBox::SetV(float v) {
    m_motors[0]->SetV(v);
}

void GearBox::SetA(float a) {
    m_motors[0]->SetA(a);
}

void GearBox::SetDistancePerPulse(double distancePerPulse) {
    m_distancePerPulse = distancePerPulse;
}

void GearBox::ResetEncoder() {
    m_motors[0]->SetPosition(0);
}

void GearBox::SetEncoderReversed(bool reverse) {
    m_isEncoderReversed = reverse;
    m_motors[0]->SetSensorDirection(m_isEncoderReversed);
}

bool GearBox::OnTarget() const {
    return abs(m_motors[0]->GetClosedLoopError()) < 15;
}

void GearBox::ResetPID() {
    m_motors[0]->ClearIaccum();
}

void GearBox::SetControlMode(CANTalon::ControlMode ctrlMode) {
    m_motors[0]->SetControlMode(ctrlMode);
}

void GearBox::SetSoftPositionLimits(double forwardLimit, double backwardLimit) {
    m_motors[0]->ConfigSoftPositionLimits(forwardLimit, backwardLimit);
}

bool GearBox::IsFwdLimitSwitchClosed() const {
    return m_motors[0]->IsFwdLimitSwitchClosed();
}

bool GearBox::IsRevLimitSwitchClosed() const {
    return m_motors[0]->IsRevLimitSwitchClosed();
}

void GearBox::SetIZone(unsigned int value) {
    m_motors[0]->SetIzone(value);
}

void GearBox::SetCloseLoopRampRate(double value) {
    m_motors[0]->SetCloseLoopRampRate(value);
}

void GearBox::SetProfile(bool secondProfile) {
    m_motors[0]->SelectProfileSlot(secondProfile);
}

void GearBox::PIDWrite(float output) {
    m_motors[0]->PIDWrite(output);
}

double GearBox::PIDGet() {
    return m_motors[0]->PIDGet();
}
