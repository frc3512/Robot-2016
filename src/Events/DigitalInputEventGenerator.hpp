// =============================================================================
// Description: Pass event to a handler if a digital input changed state
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef DIGITAL_INPUT_EVENT_GENERATOR_HPP
#define DIGITAL_INPUT_EVENT_GENERATOR_HPP

#include <cstdint>
#include <vector>

#include "../DigitalInputHandler.hpp"
#include "../SM/StateMachine.hpp"
#include "EventGenerator.hpp"

struct InterruptParam {
    InterruptParam(EventAcceptor* object, std::string eventName);

    EventAcceptor* acceptor;
    std::string eventName;
};

class DigitalInputEventGenerator : public EventGenerator {
public:
    /* Registers a digital input to be checked for either a rising or falling
     * edge.
     *
     * @param channel digital input channel. Only channels 0-9 are valid.
     * @param onRisingEdge 'true' causes event to be triggered when digital
     *                     input changes from low to high.
     * @param onFallingEdge 'true' causes event to be triggered when digital
     *                      input changes from high to low.
     * @param acceptor The argument passed to the interrupt handler.
     */
    void RegisterInputEvent(std::string eventName, uint32_t channel,
                            bool onRisingEdge, bool onFallingEdge,
                            EventAcceptor& acceptor);

    void Poll(EventAcceptor& acceptor) override;

    // This should be called after finished calling Poll() on event acceptors
    void Update();

private:
    struct DigitalInputEvent {
        std::string name;
        DigitalInput* input;
        bool onRisingEdge;
        bool onFallingEdge;
    };

    std::vector<DigitalInputEvent> m_events;

    std::vector<bool> m_oldStates;
    std::vector<bool> m_newStates;
};

#endif // DIGITAL_INPUT_EVENT_GENERATOR_HPP
