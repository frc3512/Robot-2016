// =============================================================================
// Description: Used to control two gear boxes as a differential
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "Differential.hpp"

Differential::Differential(GearBox* leftGrbx, GearBox* rightGrbx) {
    m_leftGrbx = leftGrbx;
    m_rightGrbx = rightGrbx;
}

void Differential::SetForward(float value) {
    m_forwardValue = value;

    m_leftGrbx->Set(m_forwardValue + m_turnValue);
    m_rightGrbx->Set(m_forwardValue - m_turnValue);
}

void Differential::SetTurn(float value) {
    m_turnValue = value;

    m_leftGrbx->Set(m_forwardValue - m_turnValue);
    m_rightGrbx->Set(m_forwardValue + m_turnValue);
}

void Differential::PIDWrite(float output) {
    m_turnValue = output;

    m_leftGrbx->PIDWrite(m_forwardValue - m_turnValue);
    m_rightGrbx->PIDWrite(m_forwardValue + m_turnValue);
    std::cout << "LeftGrbx: " << m_forwardValue - m_turnValue <<
        " RightGrbx: " << m_forwardValue + m_turnValue << " PIDGet: " <<
        PIDGet() <<
        std::endl;
}

double Differential::PIDGet() {
    if (GetPIDSourceType() == PIDSourceType::kRate) {
        return m_rightGrbx->GetSpeed() - m_leftGrbx->GetSpeed();
    }
    else {
        return m_rightGrbx->GetPosition() - m_leftGrbx->GetPosition();
    }
}
