// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

#include <limits>
#include <memory>
#include <thread>

#include <HAL/cpp/priority_mutex.h>
#include <Timer.h>

#include "../WPILib/PIDState.hpp"

namespace frc {
class PIDController;
}

/**
 * Base class for all types of motion profile controllers
 */
class ProfileBase {
public:
    explicit ProfileBase(std::shared_ptr<frc::PIDController> pid);
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

    std::thread m_thread;
    Timer m_timer;

    // Set this to interrupt currently running profile for starting a new one
    std::atomic<bool> m_interrupt{false};

    PIDState m_goal;
    PIDState m_sp;  // Current SetPoint
    double m_lastTime = 0.0;
    double m_timeTotal = std::numeric_limits<double>::infinity();
};
