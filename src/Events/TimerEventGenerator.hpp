// Copyright (c) FRC Team 3512, Spartatroniks 2016. All Rights Reserved.

#ifndef TIMER_EVENT_GENERATOR_HPP
#define TIMER_EVENT_GENERATOR_HPP

#include <string>

#include <Timer.h>

#include "EventGenerator.hpp"

/**
 * Pass event to a handler if a timer expired
 */
class TimerEventGenerator : public EventGenerator {
public:
    TimerEventGenerator(std::string eventName, double period,
                        bool oneShot = true);

    void Poll(EventAcceptor& acceptor) override;

    void Reset();
    bool IsOneShot() const;
    double GetTimePassedSinceLastEvent() const;

private:
    double m_period;
    std::string m_eventName;
    bool m_oneShot;
    Timer m_timer;
};

#endif  // TIMER_EVENT_GENERATOR_HPP
