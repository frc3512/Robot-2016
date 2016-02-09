// =============================================================================
// Description: Receives events
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef EVENT_ACCEPTOR_HPP
#define EVENT_ACCEPTOR_HPP

#include <string>

class EventAcceptor {
public:
    virtual ~EventAcceptor() = default;

    virtual std::string HandleEvent(std::string& event) = 0;
};

#endif // EVENT_ACCEPTOR_HPP
