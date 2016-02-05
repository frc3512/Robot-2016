// =============================================================================
// Description: Base class for all types of motion profile controllers
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "ProfileBase.hpp"

ProfileBase::ProfileBase() {
    SetMode(SetpointMode::displacement);
    ResetProfile();
}

bool ProfileBase::AtGoal() {
    return m_lastTime >= m_timeTotal;
}

ProfileState ProfileBase::GetGoal() const {
    return m_goal;
}

ProfileState ProfileBase::GetSetpoint() const {
    return m_sp;
}

void ProfileBase::ResetProfile() {
    m_goal = ProfileState();
    m_sp = ProfileState();

    m_lastTime = 0.0;
    m_timeTotal = 0.0;
}

void ProfileBase::SetMode(SetpointMode mode) {
    m_mode = mode;
}

SetpointMode ProfileBase::GetMode() const {
    return m_mode;
}
