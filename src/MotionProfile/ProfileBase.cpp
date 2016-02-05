// =============================================================================
// Description: Base class for all types of motion profile controllers
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "ProfileBase.hpp"

ProfileBase::ProfileBase() {
    SetMode(SetpointMode::distance);
    ResetProfile();
}

bool ProfileBase::AtGoal() {
    return m_lastTime >= m_timeTotal;
}

double ProfileBase::GetGoal() const {
    return m_goal;
}

double ProfileBase::GetSetpoint() const {
    return m_setpoint;
}

void ProfileBase::ResetProfile() {
    m_goal = 0.0;
    m_setpoint = 0.0;
    m_lastTime = 0.0;
    m_timeTotal = 0.0;
}

void ProfileBase::SetMode(SetpointMode mode) {
    m_mode = mode;
}

SetpointMode ProfileBase::GetMode() const {
    return m_mode;
}
