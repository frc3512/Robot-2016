// =============================================================================
// Description: Pass event to a handler if a digital input changed state
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "DigitalInputEventGenerator.hpp"

std::vector<InterruptParam> DigitalInputEventGenerator::m_params;
std::vector<std::unique_ptr<DigitalInput>> DigitalInputEventGenerator::m_inputs;

InterruptParam::InterruptParam(EventAcceptor* object, std::string eventName) {
    acceptor = object;
    this->eventName = eventName;
}

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
    m_params.emplace_back(&acceptor, std::move(eventName));
    m_inputs.emplace_back(std::make_unique<DigitalInput>(channel));

    // We are using the pointer as storage for an index of m_params
    static_assert(sizeof(void*) == sizeof(size_t),
                  "Storing integer into void* changes precision");
    size_t index = m_params.size() - 1;
    void* temp;
    std::memcpy(&temp, &index, sizeof(index));
    m_inputs.back()->RequestInterrupts(Handler, temp);
    m_inputs.back()->SetUpSourceEdge(onRisingEdge, onFallingEdge);
    m_inputs.back()->EnableInterrupts();
}

void DigitalInputEventGenerator::Poll(EventAcceptor& acceptor) {
}

void DigitalInputEventGenerator::Handler(uint32_t interruptAssertedMask,
                                         void* param) {
    static_assert(sizeof(void*) == sizeof(size_t),
                  "Storing integer into void* changes precision");
    size_t index;
    std::memcpy(&index, &param, sizeof(param));

    // Force a deep copy to keep the original event name intact
    std::string temp = m_params[index].eventName;
    m_params[index].acceptor->HandleEvent(temp);
}
