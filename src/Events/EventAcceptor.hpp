// Copyright (c) FRC Team 3512, Spartatroniks 2016. All Rights Reserved.

#ifndef EVENT_ACCEPTOR_HPP
#define EVENT_ACCEPTOR_HPP

#include <string>

/**
 * Receives events
 */
class EventAcceptor {
public:
    virtual ~EventAcceptor() = default;

    virtual std::string HandleEvent(std::string& event) = 0;
};

#endif  // EVENT_ACCEPTOR_HPP
