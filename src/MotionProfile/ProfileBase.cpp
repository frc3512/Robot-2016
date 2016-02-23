// =============================================================================
// Description: Base class for all types of motion profile controllers
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include <chrono>
#include <thread>

#include "../WPILib/PIDInterface.hpp"
#include "ProfileBase.hpp"

using namespace std::chrono_literals;

ProfileBase::ProfileBase(std::shared_ptr<PIDInterface> pid) {
    m_pid = pid;
    ResetProfile();
    m_timer.Start();
}

ProfileBase::~ProfileBase() {
    StopProfile();
}

bool ProfileBase::AtGoal() {
    /* Checking also whether the goal was reached allows the profile to stop
     * early for non-zero goal velocities and accelerations
     */
    return m_lastTime >= m_timeTotal ||
           (m_goal.displacement - m_sp.displacement < 0.001 &&
            m_goal.velocity - m_sp.velocity < 0.001 &&
            m_goal.acceleration - m_sp.acceleration < 0.001);
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

void ProfileBase::StopProfile() {
    m_interrupt = true;
    m_task.join();
}

void ProfileBase::StartProfile() {
    // Stop currently running profile
    StopProfile();

    m_timer.Reset();

    m_task = Task("ProfileBase", [this] {
        m_interrupt = false;

        while (!AtGoal()) {
            m_pid->SetSetpoint(UpdateSetpoint(m_timer.Get()));

            std::this_thread::sleep_for(10ms);
        }
    });
}
