// Copyright (c) FRC Team 3512, Spartatroniks 2016. All Rights Reserved.

#pragma once

#include "WPILib/PIDController.hpp"

/**
 * Gravity compensation feed forward for lever arm
 */
class LeverPIDController : public PIDController {
public:
    LeverPIDController(float p, float i, float d, float v, float a, float f,
                       PIDSource* source, PIDOutput* output,
                       float period = 0.05);
    void SetF(double f);
    double GetF() const;
    double CalculateFeedForward();

private:
    double m_F;
};
