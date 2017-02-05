// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

#include <PIDOutput.h>
#include <PIDSource.h>

#include "Subsystems/GearBox.hpp"

/**
 * Used to control two gear boxes as a differential
 */
class Differential : public frc::PIDOutput, public frc::PIDSource {
public:
    Differential(GearBox* leftGrbx, GearBox* rightGrbx);

    // Set forward speed of differential
    void SetForward(double value);

    // Set turning speed of differential
    void SetTurn(double value);

    // PIDOutput interface
    void PIDWrite(double output) override;

    // PIDSource interface
    double PIDGet() override;

private:
    double m_forwardValue = 0.0;
    double m_turnValue = 0.0;

    GearBox* m_leftGrbx;
    GearBox* m_rightGrbx;
};
