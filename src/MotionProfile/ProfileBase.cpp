// =============================================================================
// Description: Base class for all types of motion profile controllers
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "ProfileBase.hpp"

ProfileBase::ProfileBase() {
    ResetProfile();
}

bool ProfileBase::AtGoal() {
    return m_lastTime >= m_timeTotal;
}

PIDState ProfileBase::GetGoal() const {
    return m_goal;
}

PIDState ProfileBase::GetSetpoint() const {
    return m_sp;
}

void ProfileBase::ResetProfile() {
    m_goal = PIDState();
    m_sp = PIDState();

    m_lastTime = 0.0;
    m_timeTotal = 0.0;
}
