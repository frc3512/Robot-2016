// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

#include <stdint.h>

/**
 * This class allows you to check if a button was pressed or released without
 * having to wait in one spot of code until that happens.
 *
 * It is useful for situations in which you need to toggle a variable and just
 * checking for it with Joystick::GetRawButton(uint32_t) would cause it to
 * toggle
 * in every iteration of a loop.
 *
 * USAGE
 * 1) Call UpdateButtons() at beginning of loop to get new button statuses from
 *    the Driver Station
 * 2) Call PressedButton(), ReleasedButton(), or HeldButton() to poll for
 *    whether the button was pressed or released since last loop iteration
 *
 * None of these functions block.
 */
class ButtonTracker {
public:
    explicit ButtonTracker(uint32_t port);

    // Gets new button statuses for joystick from Driver Station
    void Update();

    // Returns 'true' if button wasn't pressed but is now
    bool PressedButton(uint32_t button) const;

    // Returns 'true' if button was pressed but isn't now
    bool ReleasedButton(uint32_t button) const;

    // Return 'true' if button was pressed and is now
    bool HeldButton(uint32_t button) const;

protected:
    uint32_t m_port;

private:
    static bool GetButtonState(uint16_t buttonStates, uint32_t button);

    uint16_t m_oldStates = 0;
    uint16_t m_newStates = 0;
};
