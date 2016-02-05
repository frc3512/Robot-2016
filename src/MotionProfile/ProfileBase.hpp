// =============================================================================
// Description: Base class for all types of motion profile controllers
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef PROFILE_BASE_HPP
#define PROFILE_BASE_HPP

#include <mutex>

typedef enum {
    distance,
    velocity
} SetpointMode;

class ProfileBase {
public:
    ProfileBase();
    virtual ~ProfileBase() = default;

    virtual double UpdateSetpoint(double curTime) = 0;

    // Should return initial setpoint for start of profile
    virtual double SetGoal(double t, double goal, double curSource) = 0;
    virtual bool AtGoal();

    double GetGoal() const;
    double GetSetpoint() const;

    virtual void ResetProfile();

    // Tells algorithm whether to use distance or velocity as setpoint
    void SetMode(SetpointMode mode);
    SetpointMode GetMode() const;

protected:
    // Use this to make updateSetpoint() and setGoal() thread-safe
    std::recursive_mutex m_varMutex;

    double m_goal;
    double m_setpoint;
    double m_lastTime;
    double m_timeTotal;

    SetpointMode m_mode;
};

#endif // PROFILE_BASE_HPP
