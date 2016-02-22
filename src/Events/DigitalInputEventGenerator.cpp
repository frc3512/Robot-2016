// =============================================================================
// Description: Pass event to a handler if a digital input changed state
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "DigitalInputEventGenerator.hpp"

std::vector<std::unique_ptr<DigitalInput>> DigitalInputEventGenerator::m_inputs(
    10);

void DigitalInputEventGenerator::RegisterInputEvent(std::string eventName,
                                                    uint32_t channel,
                                                    bool onRisingEdge,
                                                    bool onFallingEdge,
                                                    EventAcceptor& acceptor) {
    // Lazily construct digital input
    if (m_inputs[channel] == nullptr) {
        m_inputs[channel] = std::make_unique<DigitalInput>(channel);
    }

    m_events.push_back({std::move(eventName), m_inputs[channel].get(),
                        onRisingEdge, onFallingEdge});
    m_oldStates.emplace_back(false);
    m_newStates.emplace_back(false);
}

void DigitalInputEventGenerator::Poll(EventAcceptor& acceptor) {
    // Update joystick button states
    for (uint32_t i = 0; i < m_newStates.size(); i++) {
        m_newStates[i] = m_events[i].input->Get();
    }

    for (unsigned int i = 0; i < m_events.size(); i++) {
        // If checking for rising edge
        if (m_events[i].onRisingEdge) {
            // If button wasn't pressed before and is now
            if (m_oldStates[i] == false && m_newStates[i] == true) {
                // Force a deep copy to keep the original event name intact
                std::string temp = m_events[i].name;
                acceptor.HandleEvent(temp);
            }
        }

        // If checking for falling edge
        if (m_events[i].onFallingEdge) {
            // If button was pressed before and isn't now
            if (m_oldStates[i] == true && m_newStates[i] == false) {
                // Force a deep copy to keep the original event name intact
                std::string temp = m_events[i].name;
                acceptor.HandleEvent(temp);
            }
        }
    }
}

void DigitalInputEventGenerator::Update() {
    // Update old states for next check
    for (uint32_t i = 0; i < m_oldStates.size(); i++) {
        m_oldStates[i] = m_newStates[i];
    }
}
