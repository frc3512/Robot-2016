// Copyright (c) FRC Team 3512, Spartatroniks 2016. All Rights Reserved.

#ifndef EVENT_GENERATOR_HPP
#define EVENT_GENERATOR_HPP

#include <string>

#include "EventAcceptor.hpp"

/**
 * Receives events
 */
class EventGenerator {
public:
    virtual ~EventGenerator() = default;

    virtual void Poll(EventAcceptor& acceptor) = 0;
};

#endif  // EVENT_GENERATOR_HPP
