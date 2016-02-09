// =============================================================================
// Description: Pass event to a handler if a joystick button was pressed
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "JoystickEventGenerator.hpp"

void JoystickEventGenerator::RegisterButtonEvent(std::string eventName,
                                                 uint32_t port,
                                                 uint32_t button,
                                                 bool risingEdge) {
    m_events.push_back({std::move(eventName), port, button, risingEdge});
}

void JoystickEventGenerator::Poll(EventAcceptor& acceptor) {
    for (auto& event : m_events) {
        m_newStates[event.port] =
            DriverStation::GetInstance().GetStickButtons(event.port);

        // If checking for rising edge
        if (event.risingEdge) {
            // If button wasn't pressed before and is now
            if (GetButtonState(m_oldStates[event.port],
                               event.button) == false &&
                GetButtonState(m_newStates[event.port],
                               event.button) == true) {
                acceptor.HandleEvent(event.name);
            }
        }
        else {
            // If button was pressed before and isn't now
            if (GetButtonState(m_oldStates[event.port],
                               event.button) == true &&
                GetButtonState(m_newStates[event.port],
                               event.button) == false) {
                acceptor.HandleEvent(event.name);
            }
        }

        // Update old states for next check
        m_oldStates[event.port] = m_newStates[event.port];
    }
}

bool JoystickEventGenerator::GetButtonState(uint32_t buttonStates,
                                            uint32_t button) {
    return ((1 << (button - 1)) & buttonStates) != 0;
}
