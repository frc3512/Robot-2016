// =============================================================================
// Description: Receives events
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef EVENT_GENERATOR_HPP
#define EVENT_GENERATOR_HPP

#include "EventAcceptor.hpp"
#include <string>

class EventGenerator {
public:
    virtual ~EventGenerator() = default;

    virtual void Poll(EventAcceptor& acceptor) = 0;
};

#endif // EVENT_GENERATOR_HPP
