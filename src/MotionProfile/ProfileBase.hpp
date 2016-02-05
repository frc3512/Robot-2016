// =============================================================================
// Description: Base class for all types of motion profile controllers
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef PROFILE_BASE_HPP
#define PROFILE_BASE_HPP

#include <mutex>

enum SetpointMode {
    displacement,
    velocity
};

struct ProfileState {
    ProfileState() = default;
    ProfileState(double displacement, double velocity, double acceleration) {
        this->displacement = displacement;
        this->velocity = velocity;
        this->acceleration = acceleration;
    }

    double displacement = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
};

class ProfileBase {
public:
    ProfileBase();
    virtual ~ProfileBase() = default;

    virtual ProfileState UpdateSetpoint(double curTime) = 0;

    // Should return initial setpoint for start of profile
    virtual ProfileState SetGoal(double t, ProfileState goal,
                                 ProfileState curSource) = 0;
    virtual bool AtGoal();

    ProfileState GetGoal() const;
    ProfileState GetSetpoint() const;

    virtual void ResetProfile();

    // Tells algorithm whether to use distance or velocity as setpoint
    void SetMode(SetpointMode mode);
    SetpointMode GetMode() const;

protected:
    // Use this to make updateSetpoint() and setGoal() thread-safe
    std::recursive_mutex m_varMutex;

    ProfileState m_goal;
    ProfileState m_sp; // SetPoint
    double m_lastTime;
    double m_timeTotal;

    SetpointMode m_mode;
};

#endif // PROFILE_BASE_HPP
