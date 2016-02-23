// =============================================================================
// Description: Provides trapezoidal acceleration control
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

/* Constant acceleration until target (max) velocity is reached, sets
 * acceleration to zero for a calculated time, then decelerates at a constant
 * acceleration with a slope equal to the negative slope of the initial
 * acceleration.
 */

#ifndef SCURVE_PROFILE_HPP
#define SCURVE_PROFILE_HPP

#include <memory>

#include "ProfileBase.hpp"

class PIDController;

class SCurveProfile : public ProfileBase {
public:
    SCurveProfile(std::shared_ptr<PIDController> pid, double maxV, double maxA,
                  double timeToMaxA);

    /* goal is a distance to which to travel
     * curSource is the current position
     */
    virtual PIDState SetGoal(PIDState goal, PIDState curSource = PIDState());

    void SetMaxVelocity(double v);
    double GetMaxVelocity() const;
    void SetMaxAcceleration(double a);
    void SetTimeToMaxA(double timeToMaxA);

protected:
    double m_acceleration;
    double m_maxVelocity;
    double m_profileMaxVelocity;
    double m_timeToMaxA;

    double m_jerk;
    double m_t2;
    double m_t3;
    double m_t4;
    double m_t5;
    double m_t6;
    double m_t7;

    double m_sign;

    // curTime is current time
    virtual PIDState UpdateSetpoint(double curTime);
};

#endif // SCURVE_PROFILE_HPP
