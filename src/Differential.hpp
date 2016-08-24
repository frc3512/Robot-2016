// Copyright (c) FRC Team 3512, Spartatroniks 2016. All Rights Reserved.

#ifndef DIFFERENTIAL_HPP
#define DIFFERENTIAL_HPP

#include <PIDOutput.h>
#include <PIDSource.h>

#include "Subsystems/GearBox.hpp"

/**
 * Used to control two gear boxes as a differential
 */
class Differential : public PIDOutput, public PIDSource {
public:
    Differential(GearBox* leftGrbx, GearBox* rightGrbx);

    // Set forward speed of differential
    void SetForward(float value);

    // Set turning speed of differential
    void SetTurn(float value);

    // PIDOutput interface
    void PIDWrite(float output) override;

    // PIDSource interface
    double PIDGet() override;

private:
    float m_forwardValue = 0.f;
    float m_turnValue = 0.f;

    GearBox* m_leftGrbx;
    GearBox* m_rightGrbx;
};

#endif  // DIFFERENTIAL_HPP
