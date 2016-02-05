// =============================================================================
// Description: Provides trapezoidal velocity control
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "TrapezoidProfile.hpp"
#include <cmath>

TrapezoidProfile::TrapezoidProfile(double maxV, double timeToMaxV) {
    SetMaxVelocity(maxV);
    SetTimeToMaxV(timeToMaxV);
}

PIDState TrapezoidProfile::UpdateSetpoint(double curTime) {
    std::lock_guard<std::recursive_mutex> lock(m_varMutex);

    if (curTime < m_timeToMaxVelocity) {
        // Accelerate up
        m_sp.acceleration = m_acceleration;
        m_sp.velocity = m_sp.acceleration * curTime;
    }
    else if (curTime < m_timeFromMaxVelocity) {
        // Maintain max velocity
        m_sp.acceleration = 0.0;
        m_sp.velocity = m_profileMaxVelocity;
    }
    else if (curTime < m_timeTotal) {
        // Accelerate down
        double decelTime = curTime - m_timeFromMaxVelocity;
        m_sp.acceleration = -m_acceleration;
        m_sp.velocity = m_profileMaxVelocity - m_sp.acceleration * decelTime;
    }

    m_sp.acceleration *= m_sign;
    m_sp.velocity *= m_sign;
    m_sp.displacement += m_sp.velocity * (curTime - m_lastTime);

    m_lastTime = curTime;
    return m_sp;
}

PIDState TrapezoidProfile::SetGoal(double t, PIDState goal,
                                   PIDState curSource) {
    std::lock_guard<std::recursive_mutex> lock(m_varMutex);

    m_sp = m_goal = goal;
    m_sp.displacement -= curSource.displacement;

    m_sign = (m_sp.displacement < 0) ? -1.0 : 1.0;
    m_timeToMaxVelocity = m_velocity / m_acceleration;

    /* d is distance traveled when accelerating to/from max velocity
     *       = 1/2 * (v0 + v) * t
     * t is m_timeToMaxVelocity
     * delta is distance traveled at max velocity
     * delta = totalDist - 2 * d
     *       = setpoint - 2 * ((v0 + v)/2 * t)
     * v0 = 0 therefore:
     * delta = setpoint - 2 * (v/2 * t)
     *       = setpoint - v * t
     *       = m_setpoint - m_velocity * m_timeToMaxVelocity
     *
     * t is time at maximum velocity
     * t = delta (from previous comment) / m_velocity (where m_velocity is maximum velocity)
     *   = (m_setpoint - m_velocity * m_timeToMaxVelocity) / m_velocity
     *   = m_setpoint/m_velocity - m_timeToMaxVelocity
     */
    double timeAtMaxV = m_sign * m_sp.displacement / m_velocity -
                        m_timeToMaxVelocity;

    /* if ( 1/2 * a * t^2 > m_setpoint / 2 ) // if distance travelled before
     *     reaching maximum speed is more than half of the total distance to
     *     travel
     * if ( a * t^2 > m_setpoint )
     * if ( a * (v/a)^2 > m_setpoint )
     * if ( a * v^2/a^2 > m_setpoint )
     * if ( v^2/a > m_setpoint )
     * if ( v * v/a > m_setpoint )
     * if ( v * m_timeToMaxVelocity > m_setpoint )
     */
    if (m_velocity * m_timeToMaxVelocity > m_sign * m_sp.displacement) {
        /* Solve for t:
         * 1/2 * a * t^2 = m_setpoint/2
         * a * t^2 = m_setpoint
         * t^2 = m_setpoint / a
         * t = sqrt( m_setpoint / a )
         */
        m_timeToMaxVelocity = std::sqrt(m_sign * m_sp.displacement /
                                        m_acceleration);
        m_timeFromMaxVelocity = m_timeToMaxVelocity;
        m_timeTotal = 2 * m_timeToMaxVelocity;
        m_profileMaxVelocity = m_acceleration * m_timeToMaxVelocity;
    }
    else {
        m_timeFromMaxVelocity = m_timeToMaxVelocity + timeAtMaxV;
        m_timeTotal = m_timeFromMaxVelocity + m_timeToMaxVelocity;
        m_profileMaxVelocity = m_velocity;
    }

    m_lastTime = t;

    // Set setpoint to current distance since setpoint hasn't moved yet
    m_sp = curSource;
    return curSource;
}

void TrapezoidProfile::SetMaxVelocity(double v) {
    m_velocity = v;
}

double TrapezoidProfile::GetMaxVelocity() const {
    return m_velocity;
}

void TrapezoidProfile::SetTimeToMaxV(double timeToMaxV) {
    m_acceleration = m_velocity / timeToMaxV;
}
