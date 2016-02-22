// =============================================================================
// Description: Pass event to a handler if a joystick button was pressed
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef JOYSTICK_EVENT_GENERATOR_HPP
#define JOYSTICK_EVENT_GENERATOR_HPP

#include <cstdint>
#include <array>
#include <string>
#include <vector>

#include <DriverStation.h>

#include "../SM/StateMachine.hpp"
#include "EventGenerator.hpp"

class JoystickEventGenerator : public EventGenerator {
public:
    JoystickEventGenerator();

    /* Registers a joystick button to be checked for either a rising or falling
     * edge.
     *
     * @param onRisingEdge 'true' causes event to be triggered when button is
     *                     pressed. 'false' causes event to be triggered when
     *                     button is released.
     */
    void RegisterButtonEvent(std::string eventName, uint32_t port,
                             uint32_t button, bool onRisingEdge);

    void Poll(EventAcceptor& acceptor) override;

    // This should be called after finished calling Poll() on event acceptors
    void Update();

private:
    struct JoystickEvent {
        std::string name;
        uint32_t port;
        uint32_t button;
        bool onRisingEdge;
    };

    std::vector<JoystickEvent> m_events;

    std::array<uint32_t, DriverStation::kJoystickPorts> m_oldStates;
    std::array<uint32_t, DriverStation::kJoystickPorts> m_newStates;

    bool GetButtonState(uint32_t buttonStates, uint32_t button);
};

#endif // JOYSTICK_EVENT_GENERATOR_HPP
