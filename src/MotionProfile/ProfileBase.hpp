// =============================================================================
// Description: Base class for all types of motion profile controllers
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef PROFILE_BASE_HPP
#define PROFILE_BASE_HPP

#include "PIDState.hpp"
#include <mutex>

class ProfileBase {
public:
    ProfileBase();
    virtual ~ProfileBase() = default;

    virtual PIDState UpdateSetpoint(double curTime) = 0;

    // Should return initial setpoint for start of profile
    virtual PIDState SetGoal(double t, PIDState goal,
                                 PIDState curSource) = 0;
    virtual bool AtGoal();

    PIDState GetGoal() const;
    PIDState GetSetpoint() const;

    virtual void ResetProfile();

protected:
    // Use this to make updateSetpoint() and setGoal() thread-safe
    std::recursive_mutex m_varMutex;

    PIDState m_goal;
    PIDState m_sp; // SetPoint
    double m_lastTime;
    double m_timeTotal;
};

#endif // PROFILE_BASE_HPP
