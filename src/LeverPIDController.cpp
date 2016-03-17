// =============================================================================
// Description: Gravity compensation feed forward for lever arm
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "LeverPIDController.hpp"

LeverPIDController::LeverPIDController(float p,
                                       float i,
                                       float d,
                                       float v,
                                       float a,
                                       float f,
                                       PIDSource* source,
                                       PIDOutput* output,
                                       float period) :
    PIDController(p, i, d, v, a,
                  source,
                  output,
                  period) {
    m_F = f;
}

double LeverPIDController::CalculateFeedForward() {
    return GetV() * GetSetpoint().velocity + GetA() *
           GetSetpoint().acceleration + GetF();
}

void LeverPIDController::SetF(double f) {
    m_F = f;
}

double LeverPIDController::GetF() const {
    return m_F;
}
