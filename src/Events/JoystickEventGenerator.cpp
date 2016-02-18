// =============================================================================
// Description: Pass event to a handler if a joystick button was pressed
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "JoystickEventGenerator.hpp"
#include <bitset>

void JoystickEventGenerator::RegisterButtonEvent(std::string eventName,
                                                 uint32_t port,
                                                 uint32_t button,
                                                 bool risingEdge) {
    m_events.push_back({std::move(eventName), port, button, risingEdge});
}

void JoystickEventGenerator::Poll(EventAcceptor& acceptor) {
    // Update joystick button states
    for (uint32_t i = 0; i < m_newStates.size(); i++) {
        m_newStates[i] = DriverStation::GetInstance().GetStickButtons(i);
    }

    for (auto& event : m_events) {
        std::bitset<32> oldStates(m_oldStates[event.port]);
        std::bitset<32> newStates(m_newStates[event.port]);
        std::cout << "Old States: " << oldStates << " New States: " <<
            newStates << std::endl;
        std::cout << "Old Button State: " <<
            GetButtonState(m_oldStates[event.port], event.button) << std::endl;
        std::cout << "New Button State: " <<
            GetButtonState(m_newStates[event.port], event.button) << std::endl;
        // If checking for rising edge
        if (event.risingEdge) {
            // If button wasn't pressed before and is now
            if (GetButtonState(m_oldStates[event.port],
                               event.button) == false &&
                GetButtonState(m_newStates[event.port],
                               event.button) == true) {
                std::cout << "Pressed Event Generated" << std::endl;
                acceptor.HandleEvent(event.name);
            }
        }
        else {
            // If button was pressed before and isn't now
            if (GetButtonState(m_oldStates[event.port],
                               event.button) == true &&
                GetButtonState(m_newStates[event.port],
                               event.button) == false) {
                std::cout << "Released Event Generated" << std::endl;
                acceptor.HandleEvent(event.name);
            }
        }
        std::cout << std::endl;

        // Update old states for next check
        for (uint32_t i = 0; i < m_oldStates.size(); i++) {
            m_oldStates[i] = m_newStates[i];
        }
    }
}

bool JoystickEventGenerator::GetButtonState(uint32_t buttonStates,
                                            uint32_t button) {
    return ((1 << (button - 1)) & buttonStates) != 0;
}
