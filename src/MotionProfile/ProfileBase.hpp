// =============================================================================
// Description: Base class for all types of motion profile controllers
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef PROFILE_BASE_HPP
#define PROFILE_BASE_HPP

#include <limits>
#include <memory>

#include "../WPILib/PIDState.hpp"
#include <HAL/cpp/priority_mutex.h>
#include <Task.h>
#include <Timer.h>

class PIDController;

class ProfileBase {
public:
    ProfileBase(std::shared_ptr<PIDController> pid);
    virtual ~ProfileBase();

    virtual void SetGoal(PIDState goal, PIDState curSource) = 0;
    virtual bool AtGoal() const;

    PIDState GetGoal() const;
    PIDState GetSetpoint() const;

    void Stop();

protected:
    void Start();

    virtual PIDState UpdateSetpoint(double curTime) = 0;

    // Use this to make UpdateSetpoint() and SetGoal() thread-safe
    priority_mutex m_mutex;

    std::shared_ptr<PIDController> m_pid;

    Task m_task;
    Timer m_timer;

    // Set this to interrupt currently running profile for starting a new one
    std::atomic<bool> m_interrupt{false};

    PIDState m_goal;
    PIDState m_sp; // Current SetPoint
    double m_lastTime = 0.0;
    double m_timeTotal = std::numeric_limits<double>::infinity();
};

#endif // PROFILE_BASE_HPP
