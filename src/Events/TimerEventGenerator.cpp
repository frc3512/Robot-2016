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
        /* This is done before calling HandleEvent() so the current state's Exit
         * or the next state's Entry can start it again if desired.
         */
        if (m_oneShot) {
            m_timer.Stop();
            m_timer.Reset();
        }

        // Force a deep copy to keep the original event name intact
        std::string temp = m_eventName;
        acceptor.HandleEvent(temp);
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
