// Copyright (c) FRC Team 3512, Spartatroniks 2016. All Rights Reserved.

#include <chrono>
#include <cmath>
#include <thread>

#include "../WPILib/PIDController.hpp"
#include "ProfileBase.hpp"

using namespace std::chrono_literals;

ProfileBase::ProfileBase(std::shared_ptr<PIDController> pid) {
    m_pid = pid;
    m_timer.Start();
}

ProfileBase::~ProfileBase() { Stop(); }

bool ProfileBase::AtGoal() const {
    if (m_interrupt || m_lastTime >= m_timeTotal) {
        return true;
    }

    /* Checking also whether the goal was reached allows the profile to stop
     * early for non-zero goal velocities and accelerations
     */
    if (m_pid->GetPIDSourceType() == PIDSourceType::kRate) {
        return std::fabs(m_goal.velocity - m_sp.velocity) < 0.001 &&
               std::fabs(m_goal.acceleration - m_sp.acceleration) < 0.001;
    } else {
        return m_goal == m_sp;
    }
}

PIDState ProfileBase::GetGoal() const { return m_goal; }

PIDState ProfileBase::GetSetpoint() const { return m_sp; }

void ProfileBase::Stop() {
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

void ProfileBase::Start() {
    // Stop currently running profile
    Stop();

    m_lastTime = 0.0;
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
