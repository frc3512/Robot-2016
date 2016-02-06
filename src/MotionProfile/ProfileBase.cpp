// =============================================================================
// Description: Base class for all types of motion profile controllers
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "ProfileBase.hpp"
#include "../WPILib/PIDInterface.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

ProfileBase::ProfileBase(std::shared_ptr<PIDInterface> pid) {
    m_pid = pid;
    ResetProfile();
    m_timer.Start();
}

ProfileBase::~ProfileBase() {
    m_interrupt = true;
    m_task.join();
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
    m_pid->SetSetpoint(m_sp);

    m_lastTime = 0.0;
    m_timeTotal = 0.0;
}

void ProfileBase::StartProfile() {
    // Stop currently running profile
    m_interrupt = true;
    m_task.join();

    m_timer.Reset();

    m_task = Task("ProfileBase", [this] {
        m_interrupt = false;

        while (!AtGoal()) {
            m_pid->SetSetpoint(UpdateSetpoint(m_timer.Get()));

            std::this_thread::sleep_for(10ms);
        }
    });
}
