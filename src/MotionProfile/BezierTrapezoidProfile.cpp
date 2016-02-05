// =============================================================================
// Description: Provides trapezoidal velocity control and follows a given Bézier
//             curve
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "BezierTrapezoidProfile.hpp"
#include <cmath>

BezierTrapezoidProfile::BezierTrapezoidProfile(double maxV, double timeToMaxV) :
    TrapezoidProfile(maxV, timeToMaxV) {
    SetMaxVelocity(maxV);
    SetTimeToMaxV(timeToMaxV);
    SetMode(SetpointMode::distance);

    m_width = 0.0;
    m_leftSetpoint = 0.0;
    m_rightSetpoint = 0.0;
}

double BezierTrapezoidProfile::UpdateSetpoint(double curTime) {
    double period = curTime - m_lastTime;

    m_varMutex.lock();

    if (m_mode == SetpointMode::distance) {
        if (curTime < m_timeToMaxVelocity) {
            // Accelerate up
            m_setpoint += (m_acceleration * curTime) * period * m_sign;
            m_leftSetpoint +=
                GetLeftVelocity(curTime,
                                m_acceleration * curTime) * period * m_sign;
            m_rightSetpoint += GetRightVelocity(curTime,
                                                m_acceleration * curTime) *
                               period * m_sign;
        }
        else if (curTime < m_timeFromMaxVelocity) {
            // Maintain max velocity
            m_setpoint += (m_profileMaxVelocity * period * m_sign);
            m_leftSetpoint +=
                GetLeftVelocity(curTime,
                                m_profileMaxVelocity) * period * m_sign;
            m_rightSetpoint +=
                GetRightVelocity(curTime,
                                 m_profileMaxVelocity) * period * m_sign;
        }
        else if (curTime < m_timeTotal) {
            // Accelerate down
            double decelTime = curTime - m_timeFromMaxVelocity;
            double v = m_profileMaxVelocity - m_acceleration * decelTime;

            m_setpoint += v * period * m_sign;
            m_leftSetpoint += GetLeftVelocity(curTime, v) * period * m_sign;
            m_rightSetpoint += GetRightVelocity(curTime, v) * period * m_sign;
        }
    }
    else if (m_mode == SetpointMode::velocity) {
        if (curTime < m_timeToMaxVelocity) {
            // Accelerate up
            m_setpoint = (m_acceleration * curTime) * m_sign;
            m_leftSetpoint =
                GetLeftVelocity(curTime, m_acceleration * curTime) * m_sign;
            m_rightSetpoint =
                GetRightVelocity(curTime, m_acceleration * curTime) * m_sign;
        }
        else if (curTime < m_timeFromMaxVelocity) {
            // Maintain max velocity
            m_setpoint = m_profileMaxVelocity * m_sign;
            m_leftSetpoint =
                GetLeftVelocity(curTime, m_profileMaxVelocity) * m_sign;
            m_rightSetpoint =
                GetRightVelocity(curTime, m_profileMaxVelocity) * m_sign;
        }
        else if (curTime < m_timeTotal) {
            // Accelerate down
            double decelTime = curTime - m_timeFromMaxVelocity;
            double v = m_profileMaxVelocity + (-m_acceleration * decelTime);

            m_setpoint = v * m_sign;
            m_leftSetpoint = GetLeftVelocity(curTime, v) * m_sign;
            m_rightSetpoint = GetRightVelocity(curTime, v) * m_sign;
        }
    }

    m_varMutex.unlock();

    m_lastTime = curTime;
    return m_setpoint;
}

double BezierTrapezoidProfile::GetLeftSetpoint() const {
    return m_leftSetpoint;
}

double BezierTrapezoidProfile::GetRightSetpoint() const {
    return m_rightSetpoint;
}

double BezierTrapezoidProfile::SetCurveGoal(const BezierCurve& curve,
                                            double t) {
    m_curve = curve;

    return TrapezoidProfile::SetGoal(t, m_curve.GetArcLength(0, 1));
}

void BezierTrapezoidProfile::ResetProfile() {
    TrapezoidProfile::ResetProfile();

    m_leftSetpoint = 0.0;
    m_rightSetpoint = 0.0;
}

void BezierTrapezoidProfile::SetWidth(double width) {
    m_width = width;
}

double BezierTrapezoidProfile::GetLeftVelocity(double t, double v) const {
    return (1.0 - m_curve.GetCurvature(t / m_timeTotal) * m_width / 2.0) * v;
}

double BezierTrapezoidProfile::GetRightVelocity(double t, double v) const {
    return (1.0 + m_curve.GetCurvature(t / m_timeTotal) * m_width / 2.0) * v;
}
