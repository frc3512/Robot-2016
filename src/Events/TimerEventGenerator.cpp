// =============================================================================
// Description: Pass event to a handler if a timer expired
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "TimerEventGenerator.hpp"

TimerEventGenerator::TimerEventGenerator(std::string eventName,
                                         double period,
                                         bool oneShot) {
    m_period = period;
    m_eventName = eventName;
    m_oneShot = oneShot;

    m_timer.Start();
}

void TimerEventGenerator::Poll(EventAcceptor& acceptor) {
    if (m_timer.HasPeriodPassed(m_period)) {
        acceptor.HandleEvent(m_eventName);
        if (m_oneShot) {
            m_timer.Stop();
            m_timer.Reset();
        }
    }
}

void TimerEventGenerator::Reset() {
    m_timer.Start();
    m_timer.Reset();
}

bool TimerEventGenerator::IsOneShot() const {
    return m_oneShot;
}

double TimerEventGenerator::GetTimePassedSinceLastEvent() const {
    return m_timer.Get();
}
