// =============================================================================
// Description: Pass event to a handler if a joystick button was pressed
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef JOYSTICK_EVENT_GENERATOR_HPP
#define JOYSTICK_EVENT_GENERATOR_HPP

#include "EventGenerator.hpp"
#include "../SM/StateMachine.hpp"
#include <DriverStation.h>
#include <string>
#include <vector>
#include <cstdint>

class JoystickEventGenerator : public EventGenerator {
public:
    /* Registers a joystick button to be checked for either a rising or trailing
     * edge.
     *
     * @param risingEdge 'true' causes event to be triggered when button is
     *                   pressed. 'false' causes event to be triggered when
     *                   button is released.
     */
    void RegisterButtonEvent(std::string eventName, uint32_t port,
                             uint32_t button, bool risingEdge);

    void Poll(EventAcceptor& acceptor) override;

private:
    struct JoystickEvent {
        std::string name;
        uint32_t port;
        uint32_t button;
        bool risingEdge;
    };

    // tuple of port, button, risingEdge, eventName
    std::vector<JoystickEvent> m_events;

    std::vector<uint32_t> m_oldStates{DriverStation::kJoystickPorts, 0};
    std::vector<uint32_t> m_newStates{DriverStation::kJoystickPorts, 0};

    bool GetButtonState(uint32_t buttonStates, uint32_t button);
};

#endif // JOYSTICK_EVENT_GENERATOR_HPP
