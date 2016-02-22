// =============================================================================
// Description: Pass event to a handler if a joystick button was pressed
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "JoystickEventGenerator.hpp"

JoystickEventGenerator::JoystickEventGenerator() {
    for (auto& state : m_oldStates) {
        state = 0;
    }
    for (auto& state : m_newStates) {
        state = 0;
    }
}

void JoystickEventGenerator::RegisterButtonEvent(std::string eventName,
                                                 uint32_t port,
                                                 uint32_t button,
                                                 bool onRisingEdge) {
    m_events.push_back({std::move(eventName), port, button, onRisingEdge});
}

void JoystickEventGenerator::Poll(EventAcceptor& acceptor) {
    // Update joystick button states
    for (uint32_t i = 0; i < m_newStates.size(); i++) {
        m_newStates[i] = DriverStation::GetInstance().GetStickButtons(i);
    }

    for (auto& event : m_events) {
        // If checking for rising edge
        if (event.onRisingEdge) {
            // If button wasn't pressed before and is now
            if (GetButtonState(m_oldStates[event.port],
                               event.button) == false &&
                GetButtonState(m_newStates[event.port],
                               event.button) == true) {
                // Force a deep copy to keep the original event name intact
                std::string temp = event.name;
                acceptor.HandleEvent(temp);
            }
        }
        else {
            // If button was pressed before and isn't now
            if (GetButtonState(m_oldStates[event.port],
                               event.button) == true &&
                GetButtonState(m_newStates[event.port],
                               event.button) == false) {
                // Force a deep copy to keep the original event name intact
                std::string temp = event.name;
                acceptor.HandleEvent(temp);
            }
        }
    }
}

void JoystickEventGenerator::Update() {
    // Update old states for next check
    for (uint32_t i = 0; i < m_oldStates.size(); i++) {
        m_oldStates[i] = m_newStates[i];
    }
}

bool JoystickEventGenerator::GetButtonState(uint32_t buttonStates,
                                            uint32_t button) {
    return ((1 << (button - 1)) & buttonStates) != 0;
}
