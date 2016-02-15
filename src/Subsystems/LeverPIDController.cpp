// =============================================================================
// Description: Gravity compenstation feed forward for lever arm
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
    PIDController(0.f, 0.f, 0.f, 0.f, 0.f,
                  source,
                  output,
                  period) {
    m_F = f;
    CalculateFeedForward();
}

double LeverPIDController::CalculateFeedForward() {
    return GetV() * GetSetpoint().velocity + GetA() * GetSetpoint().acceleration;
}

void LeverPIDController::SetF(double f) {
	m_F = f;
}
