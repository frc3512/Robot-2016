// =============================================================================
// Description: Provides trapezoidal velocity control and follows a given Bézier
//             curve
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "BezierTrapezoidProfile.hpp"
#include "../WPILib/PIDInterface.hpp"
#include <cmath>

BezierTrapezoidProfile::BezierTrapezoidProfile(
    std::shared_ptr<PIDInterface> pid,
    double maxV,
    double timeToMaxV) :
    TrapezoidProfile(std::move(pid), maxV, timeToMaxV) {
    SetMaxVelocity(maxV);
    SetTimeToMaxV(timeToMaxV);
}

PIDState BezierTrapezoidProfile::GetLeftSetpoint() const {
    return m_leftSetpoint;
}

PIDState BezierTrapezoidProfile::GetRightSetpoint() const {
    return m_rightSetpoint;
}

PIDState BezierTrapezoidProfile::SetCurveGoal(const BezierCurve& curve,
                                              PIDState curSource) {
    m_curve = curve;

    PIDState state = {m_curve.GetArcLength(0, 1), 0.0, 0.0};
    return TrapezoidProfile::SetGoal(state, curSource);
}

void BezierTrapezoidProfile::ResetProfile() {
    TrapezoidProfile::ResetProfile();

    m_leftSetpoint = PIDState();
    m_rightSetpoint = PIDState();
}

void BezierTrapezoidProfile::SetWidth(double width) {
    m_width = width;
}

PIDState BezierTrapezoidProfile::UpdateSetpoint(double curTime) {
    // TODO: Verify correct acceleration is used along curve

    double period = curTime - m_lastTime;

    m_varMutex.lock();

    if (curTime < m_timeToMaxVelocity) {
        // Accelerate up
        m_sp.acceleration = m_acceleration;
        m_sp.velocity = m_acceleration * curTime;

        m_leftSetpoint.acceleration = m_acceleration;
        m_leftSetpoint.velocity = GetLeftVelocity(curTime,
                                                  m_acceleration * curTime);

        m_rightSetpoint.acceleration = m_acceleration;
        m_rightSetpoint.velocity = GetRightVelocity(curTime,
                                                    m_acceleration * curTime);
    }
    else if (curTime < m_timeFromMaxVelocity) {
        // Maintain max velocity
        m_sp.acceleration = 0.0;
        m_sp.velocity = m_profileMaxVelocity;

        m_leftSetpoint.acceleration = 0.0;
        m_leftSetpoint.velocity = GetLeftVelocity(curTime,
                                                  m_profileMaxVelocity);

        m_rightSetpoint.acceleration = 0.0;
        m_rightSetpoint.velocity = GetRightVelocity(curTime,
                                                    m_profileMaxVelocity);
    }
    else if (curTime < m_timeTotal) {
        // Accelerate down
        double decelTime = curTime - m_timeFromMaxVelocity;
        double v = m_profileMaxVelocity - m_acceleration * decelTime;

        m_sp.acceleration = -m_acceleration;
        m_sp.velocity = v;

        m_leftSetpoint.acceleration = -m_acceleration;
        m_leftSetpoint.velocity = GetLeftVelocity(curTime, v);

        m_rightSetpoint.acceleration = -m_acceleration;
        m_rightSetpoint.velocity = GetRightVelocity(curTime, v);
    }

    m_sp.acceleration *= m_sign;
    m_leftSetpoint.acceleration *= m_sign;
    m_rightSetpoint.acceleration *= m_sign;

    m_sp.velocity *= m_sign;
    m_leftSetpoint.velocity *= m_sign;
    m_rightSetpoint.velocity *= m_sign;

    m_sp.displacement += m_sp.velocity * period;
    m_leftSetpoint.displacement += m_leftSetpoint.velocity * period;
    m_rightSetpoint.displacement += m_rightSetpoint.velocity * period;

    m_varMutex.unlock();

    m_lastTime = curTime;

    StartProfile();

    return m_sp;
}

double BezierTrapezoidProfile::GetLeftVelocity(double t, double v) const {
    return (1.0 - m_curve.GetCurvature(t / m_timeTotal) * m_width / 2.0) * v;
}

double BezierTrapezoidProfile::GetRightVelocity(double t, double v) const {
    return (1.0 + m_curve.GetCurvature(t / m_timeTotal) * m_width / 2.0) * v;
}
