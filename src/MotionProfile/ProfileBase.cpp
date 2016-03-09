// =============================================================================
// Description: Base class for all types of motion profile controllers
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include <cmath>
#include <chrono>
#include <thread>

#include "../WPILib/PIDController.hpp"
#include "ProfileBase.hpp"

using namespace std::chrono_literals;

ProfileBase::ProfileBase(std::shared_ptr<PIDController> pid) {
    m_pid = pid;
    ResetProfile();
    m_timer.Start();
}

ProfileBase::~ProfileBase() {
    StopProfile();
}

bool ProfileBase::AtGoal() {
	std::cout << "LAST TIME: " << m_lastTime << "TIME TOTAL: " << m_timeTotal << " INTERUPTS: " << m_interrupt << std::endl;
	if (m_interrupt || m_lastTime >= m_timeTotal) {
        return true;
    }

    /* Checking also whether the goal was reached allows the profile to stop
     * early for non-zero goal velocities and accelerations
     */
    if (m_pid->GetPIDSourceType() == PIDSourceType::kRate) {
        return std::fabs(m_goal.velocity - m_sp.velocity) < 0.001 &&
               std::fabs(m_goal.acceleration - m_sp.acceleration) < 0.001;
    }
    else {
        return std::fabs(m_goal.displacement - m_sp.displacement) < 0.001 &&
               std::fabs(m_goal.velocity - m_sp.velocity) < 0.001 &&
               std::fabs(m_goal.acceleration - m_sp.acceleration) < 0.001;
    }
    return false;
}

PIDState ProfileBase::GetGoal() const {
    return m_goal;
}

PIDState ProfileBase::GetSetpoint() const {
    return m_sp;
}

void ProfileBase::ResetProfile() {
	StopProfile();
	m_goal = PIDState();
    m_sp = PIDState();
    m_pid->SetSetpoint(m_sp);

    m_lastTime = 0.0;
    m_timeTotal = 0.0;
}

void ProfileBase::StopProfile() {
    if (!m_interrupt && m_task.joinable()) {
        m_interrupt = true;
        m_task.join();
        m_interrupt = false;
    }

    // If PID is enabled, disable it
    if (m_pid->IsEnabled()) {
        m_pid->Disable();
    }
}

void ProfileBase::StartProfile() {
    // Stop currently running profile
    StopProfile();

    m_timer.Reset();

    // If PID is disabled, enable it
    if (!m_pid->IsEnabled()) {
        m_pid->Enable();
    }

    m_task = Task("ProfileBase", [this] {
        m_interrupt = false;

        while (!AtGoal()) {
            m_pid->SetSetpoint(UpdateSetpoint(m_timer.Get()));

            std::this_thread::sleep_for(10ms);
        }
    });
}
