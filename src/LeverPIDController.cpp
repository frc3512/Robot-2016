// Copyright (c) FRC Team 3512, Spartatroniks 2016. All Rights Reserved.

#include "LeverPIDController.hpp"

LeverPIDController::LeverPIDController(float p, float i, float d, float v,
                                       float a, float f, PIDSource* source,
                                       PIDOutput* output, float period)
    : PIDController(p, i, d, v, a, source, output, period) {
    m_F = f;
}

double LeverPIDController::CalculateFeedForward() {
    return GetV() * GetSetpoint().velocity +
           GetA() * GetSetpoint().acceleration +
           GetF() * std::cos((m_pidInput->PIDGet() - 10.0) / (60.0 - 10.0) *
                             M_PI / 2.0);
}

void LeverPIDController::SetF(double f) { m_F = f; }

double LeverPIDController::GetF() const { return m_F; }
