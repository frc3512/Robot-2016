// =============================================================================
// Description: Base class for all types of motion profile controllers
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef PROFILE_BASE_HPP
#define PROFILE_BASE_HPP

#include "../PIDState.hpp"
#include <Task.h>
#include <Timer.h>
#include <memory>
#include <mutex>

class PIDInterface;

class ProfileBase {
public:
    ProfileBase(std::shared_ptr<PIDInterface> pid);
    virtual ~ProfileBase();

    virtual PIDState UpdateSetpoint(double curTime) = 0;

    // Should return initial setpoint for start of profile
    virtual PIDState SetGoal(double t, PIDState goal,
                             PIDState curSource) = 0;
    virtual bool AtGoal();

    PIDState GetGoal() const;
    PIDState GetSetpoint() const;

    virtual void ResetProfile();

protected:
    void StartProfile();

    // Use this to make UpdateSetpoint() and SetGoal() thread-safe
    std::recursive_mutex m_varMutex;

    std::shared_ptr<PIDInterface> m_pid;

    Task m_task;
    Timer m_timer;

    // Set this to interrupt currently running profile for starting a new one
    std::atomic<bool> m_interrupt{false};

    PIDState m_goal;
    PIDState m_sp; // Current SetPoint
    double m_lastTime;
    double m_timeTotal;
};

#endif // PROFILE_BASE_HPP
