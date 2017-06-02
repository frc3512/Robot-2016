// Copyright (c) 2016-2017 FRC Team 3512. All Rights Reserved.

#include "ButtonTracker.hpp"

#include <DriverStation.h>

ButtonTracker::ButtonTracker(uint32_t port) { m_port = port; }

void ButtonTracker::Update() {
    // "new" values are now "old"
    m_oldStates = m_newStates;

    // Get newer values
    m_newStates = frc::DriverStation::GetInstance().GetStickButtons(m_port);
}

bool ButtonTracker::PressedButton(uint32_t button) const {
    return GetButtonState(m_oldStates, button) == true &&
           GetButtonState(m_newStates, button) == false;
}

bool ButtonTracker::ReleasedButton(uint32_t button) const {
    return GetButtonState(m_oldStates, button) == false &&
           GetButtonState(m_newStates, button) == true;
}

bool ButtonTracker::HeldButton(uint32_t button) const {
    return GetButtonState(m_oldStates, button) == false &&
           GetButtonState(m_newStates, button) == false;
}

bool ButtonTracker::GetButtonState(uint16_t buttonStates, uint32_t button) {
    return ((1 << (button - 1)) & buttonStates) != 0;
}
