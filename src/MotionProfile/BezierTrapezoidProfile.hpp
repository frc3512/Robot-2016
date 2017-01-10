// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

/* Implementation of trapezoid motion profile translated to C++; base Java code
 * courtesy of FRC Team 254; modifications for Bézier curve by FRC Team 3512.
 */

/* Constant acceleration until target (max) velocity is reached, sets
 * acceleration to zero for a calculated time, then decelerates at a constant
 * acceleration with a slope equal to the negative slope of the initial
 * acceleration.
 */

#pragma once

#include <memory>

#include "BezierCurve.hpp"
#include "TrapezoidProfile.hpp"

namespace frc {
class PIDController;
}

/**
 * Provides trapezoidal velocity control and follows a given Bézier curve
 */
class BezierTrapezoidProfile : public TrapezoidProfile {
public:
    BezierTrapezoidProfile(std::shared_ptr<frc::PIDController> pid, double maxV,
                           double timeToMaxV);

    /* Returns uncompensated setpoint for use in control of systems other than
     * the drive train
     */
    PIDState GetMidSetpoint() const;

    PIDState GetLeftSetpoint() const;
    PIDState GetRightSetpoint() const;

    /* goal is a Bézier curve for robot to follow
     * curSource is the current position
     */
    void SetCurveGoal(const BezierCurve& curve,
                      PIDState curSource = PIDState());

    // Sets distance between two sides of drive train
    void SetWidth(double width);

protected:
    /* curTime is current time
     *
     * returns updated uncompensated setpoint (see double getMidSetpoint())
     */
    PIDState UpdateSetpoint(double curTime);

private:
    // The robot follows this by turning in the motion profile
    BezierCurve m_curve;
    double m_width = 0.0;

    // Collection of setpoints
    PIDState m_leftSetpoint;
    PIDState m_rightSetpoint;

    /* t is time elapsed since start of motion
     * v is current setpoint velocity of middle of robot
     */
    double GetLeftVelocity(double t, double v) const;
    double GetRightVelocity(double t, double v) const;
};
