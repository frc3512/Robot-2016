// =============================================================================
// Description: Helps user determine if joystick button was just pressed, just
//              released, or held
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "ButtonTracker.hpp"

#include <DriverStation.h>

ButtonTracker::ButtonTracker(uint32_t port) {
    m_port = port;
}

void ButtonTracker::Update() {
    // "new" values are now "old"
    m_oldStates = m_newStates;

    // Get newer values
    m_newStates = DriverStation::GetInstance().GetStickButtons(m_port);
}

bool ButtonTracker::PressedButton(uint32_t button) {
    return GetButtonState(m_oldStates, button) == false &&
           GetButtonState(m_newStates, button) == true;
}

bool ButtonTracker::ReleasedButton(uint32_t button) {
    return GetButtonState(m_oldStates, button) == true &&
           GetButtonState(m_newStates, button) == false;
}

bool ButtonTracker::HeldButton(uint32_t button) {
    return GetButtonState(m_oldStates, button) == true &&
           GetButtonState(m_newStates, button) == true;
}

bool ButtonTracker::GetButtonState(short& buttonStates, uint32_t& button) {
    return ((1 << (button - 1)) & buttonStates) != 0;
}
