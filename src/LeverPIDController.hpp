// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

#include "WPILib/PIDController.hpp"

/**
 * Gravity compensation feed forward for lever arm
 */
class LeverPIDController : public frc::PIDController {
public:
    LeverPIDController(double p, double i, double d, double v, double a,
                       double f, frc::PIDSource* source, frc::PIDOutput* output,
                       double period = 0.05);
    void SetF(double f);
    double GetF() const;
    double CalculateFeedForward();

private:
    double m_F;
};
