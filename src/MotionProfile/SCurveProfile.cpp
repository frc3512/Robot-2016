// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include "SCurveProfile.hpp"

#include <cmath>
#include <utility>

#include "../WPILib/PIDController.hpp"

SCurveProfile::SCurveProfile(std::shared_ptr<frc::PIDController> pid,
                             double maxV, double maxA, double timeToMaxA)
    : ProfileBase(std::move(pid)) {
    SetMaxVelocity(maxV);
    SetMaxAcceleration(maxA);
    SetTimeToMaxA(timeToMaxA);
}

void SCurveProfile::SetGoal(PIDState goal, PIDState curSource) {
    std::lock_guard<priority_mutex> lock(m_mutex);

    // Subtract current source for profile calculations
    m_goal = goal - curSource;

    // Set setpoint to current distance since setpoint hasn't moved yet
    m_sp = curSource;

    if (m_goal.displacement < 0.0) {
        m_sign = -1.0;
    } else {
        m_sign = 1.0;
    }

    // If profile can't accelerate up to max velocity before decelerating
    bool shortProfile =
        m_maxVelocity * (m_timeToMaxA + m_maxVelocity / m_acceleration) >
        m_sign * m_goal.displacement;

    if (shortProfile) {
        m_profileMaxVelocity =
            m_acceleration *
            (std::sqrt(m_sign * m_goal.displacement / m_acceleration -
                       0.75 * std::pow(m_timeToMaxA, 2)) -
             0.5 * m_timeToMaxA);
    } else {
        m_profileMaxVelocity = m_maxVelocity;
    }

    // Find times at critical points
    m_t2 = m_profileMaxVelocity / m_acceleration;
    m_t3 = m_t2 + m_timeToMaxA;
    if (shortProfile) {
        m_t4 = m_t3;
    } else {
        m_t4 = m_sign * m_goal.displacement / m_profileMaxVelocity;
    }
    m_t5 = m_t4 + m_timeToMaxA;
    m_t6 = m_t4 + m_t2;
    m_t7 = m_t6 + m_timeToMaxA;
    m_timeTotal = m_t7;

    // Restore desired goal
    m_goal = goal;

    Start();
}

void SCurveProfile::SetMaxVelocity(double v) { m_maxVelocity = v; }

double SCurveProfile::GetMaxVelocity() const { return m_maxVelocity; }

void SCurveProfile::SetMaxAcceleration(double a) {
    m_acceleration = a;
    m_jerk = m_acceleration / m_timeToMaxA;
}

void SCurveProfile::SetTimeToMaxA(double timeToMaxA) {
    m_timeToMaxA = timeToMaxA;
    m_jerk = m_acceleration / m_timeToMaxA;
}

PIDState SCurveProfile::UpdateSetpoint(double curTime) {
    std::lock_guard<priority_mutex> lock(m_mutex);

    if (curTime < m_timeToMaxA) {
        // Ramp up acceleration
        m_sp.acceleration = m_jerk * curTime;
        m_sp.velocity = 0.5 * m_jerk * std::pow(curTime, 2);
    } else if (curTime < m_t2) {
        // Increase speed at max acceleration
        m_sp.acceleration = m_acceleration;
        m_sp.velocity = m_acceleration * (curTime - 0.5 * m_timeToMaxA);
    } else if (curTime < m_t3) {
        // Ramp down acceleration
        m_sp.acceleration = m_acceleration - m_jerk * (m_t2 - curTime);
        m_sp.velocity =
            m_acceleration * m_t2 - 0.5 * m_jerk * std::pow(m_t2 - curTime, 2);
    } else if (curTime < m_t4) {
        // Maintain max velocity
        m_sp.acceleration = 0.0;
        m_sp.velocity = m_profileMaxVelocity;
    } else if (curTime < m_t5) {
        // Ramp down acceleration
        m_sp.acceleration = -m_jerk * (curTime - m_t4);
        m_sp.velocity =
            m_profileMaxVelocity - 0.5 * m_jerk * std::pow(curTime - m_t4, 2);
    } else if (curTime < m_t6) {
        // Decrease speed at max acceleration
        m_sp.acceleration = m_acceleration;
        m_sp.velocity = m_acceleration * (m_t2 + m_t5 - curTime);
    } else if (curTime < m_t7) {
        // Ramp up acceleration
        m_sp.acceleration = m_jerk * (m_t6 - curTime);
        m_sp.velocity = 0.5 * m_jerk * std::pow(m_t6 - curTime, 2);
    }

    m_sp.displacement += m_sign * m_sp.velocity * (curTime - m_lastTime);

    m_lastTime = curTime;
    return m_sp;
}
