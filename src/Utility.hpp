// =============================================================================
// Description: Provides generic utility functions
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef UTILITY_HPP
#define UTILITY_HPP

// Zeroes value if it's inside deadband range, and rescales values outside of it
float ApplyDeadband(float value, float deadband);

// Limits 'value' to within +- 'limit' (limit should be positive)
template <class T>
T Limit(T value, T limit) {
    if (value > limit) {
        return limit;
    }
    else if (value < -limit) {
        return -limit;
    }
    else {
        return value;
    }
}

/* Rescales joystick value from [1..-1] to [0..rangeMax] (this includes flipping
 * the range)
 */
float JoystickRescale(float value, float rangeMax);

#endif // UTILITY_HPP
