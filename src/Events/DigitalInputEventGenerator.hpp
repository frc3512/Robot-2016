// =============================================================================
// Description: Pass event to a handler if a digital input changed state
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef DIGITAL_INPUT_EVENT_GENERATOR_HPP
#define DIGITAL_INPUT_EVENT_GENERATOR_HPP

#include "EventGenerator.hpp"
#include "../SM/StateMachine.hpp"
#include <DigitalInput.h>
#include <map>
#include <memory>
#include <vector>
#include <cstdint>

class DigitalInputEventGenerator : public EventGenerator {
public:
    virtual ~DigitalInputEventGenerator();

    /* Registers a joystick button to be checked for either a rising or trailing
     * edge.
     *
     * @param onRisingEdge 'true' causes event to be triggered when digital
     *                     input changes from low to high.
     * @param onTrailingEdge 'true' causes event to be triggered when digital
     *                       input changes from high to low.
     * @param acceptor The argument passed to the interrupt handler.
     */
    void RegisterInputEvent(std::string eventName, uint32_t channel,
                            bool onRisingEdge, bool onFallingEdge,
                            EventAcceptor& acceptor);

    void Poll(EventAcceptor& acceptor) override;

private:
    static std::map<EventAcceptor*, std::string> m_eventAcceptorMap;
    static std::vector<std::unique_ptr<DigitalInput>> m_inputs;

    static void Handler(uint32_t interruptAssertedMask, void* param);
};

#endif // DIGITAL_INPUT_EVENT_GENERATOR_HPP
