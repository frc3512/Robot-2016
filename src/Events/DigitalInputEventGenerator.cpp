// =============================================================================
// Description: Pass event to a handler if a digital input changed state
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "DigitalInputEventGenerator.hpp"

std::map<EventAcceptor*,
         std::string> DigitalInputEventGenerator::m_eventAcceptorMap;
std::vector<std::unique_ptr<DigitalInput>> DigitalInputEventGenerator::m_inputs;

DigitalInputEventGenerator::~DigitalInputEventGenerator() {
    for (auto& input : m_inputs) {
        input->CancelInterrupts();
    }
}

void DigitalInputEventGenerator::RegisterInputEvent(std::string eventName,
                                                    uint32_t channel,
                                                    bool onRisingEdge,
                                                    bool onFallingEdge,
                                                    EventAcceptor& acceptor) {
    m_eventAcceptorMap.emplace(&acceptor, std::move(eventName));
    m_inputs.emplace_back(std::make_unique<DigitalInput>(channel));
    m_inputs.back()->RequestInterrupts(Handler, &acceptor);
    m_inputs.back()->SetUpSourceEdge(onRisingEdge, onFallingEdge);
    m_inputs.back()->EnableInterrupts();
}

void DigitalInputEventGenerator::Poll(EventAcceptor& acceptor) {
}

void DigitalInputEventGenerator::Handler(uint32_t interruptAssertedMask,
                                         void* param) {
    auto object = static_cast<EventAcceptor*>(param);

    // Force a deep copy to keep the original event name intact
    std::string temp = m_eventAcceptorMap[object];
    object->HandleEvent(temp);
}
