// =============================================================================
// Description: Provides trapezoidal velocity control
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

/* Implementation of trapezoid motion profile translated to C++; base Java code
 * courtesy of FRC Team 254
 */

/* Constant acceleration until target (max) velocity is reached, sets
 * acceleration to zero for a calculated time, then decelerates at a constant
 * acceleration with a slope equal to the negative slope of the initial
 * acceleration.
 */

#ifndef TRAPEZOID_PROFILE_HPP
#define TRAPEZOID_PROFILE_HPP

#include <memory>

#include "ProfileBase.hpp"

class PIDInterface;

class TrapezoidProfile : public ProfileBase {
public:
    TrapezoidProfile(std::shared_ptr<PIDInterface> pid, double maxV,
                     double timeToMaxV);
    virtual ~TrapezoidProfile() = default;

    /* goal is a distance to which to travel
     * curSource is the current position
     */
    virtual PIDState SetGoal(PIDState goal, PIDState curSource = PIDState());

    void SetMaxVelocity(double v);
    double GetMaxVelocity() const;
    void SetTimeToMaxV(double timeToMaxV);

protected:
    double m_acceleration;
    double m_velocity;
    double m_profileMaxVelocity;
    double m_timeFromMaxVelocity;
    double m_timeToMaxVelocity;
    double m_sign;

    // curTime is current time
    virtual PIDState UpdateSetpoint(double curTime);
};

#endif // TRAPEZOID_PROFILE_HPP
