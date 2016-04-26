// =============================================================================
// Description: Helps user determine if joystick button was just pressed, just
//              released, or held
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef BUTTON_TRACKER_HPP
#define BUTTON_TRACKER_HPP

#include <cstdint>

/* This class allows you to check if a button was pressed or released without
 * having to wait in one spot of code until that happens.
 *
 * It is useful for situations in which you need to toggle a variable and just
 * checking for it with Joystick::GetRawButton(uint32_t) would cause it to toggle
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
    static bool GetButtonState(short buttonStates, uint32_t button);

    short m_oldStates = 0;
    short m_newStates = 0;
};

#endif // BUTTON_TRACKER_HPP
