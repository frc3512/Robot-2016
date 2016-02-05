// =============================================================================
// Description: Provides generic utility functions
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef UTILITY_HPP
#define UTILITY_HPP

// Zeroes value if it's inside deadband range, and rescales values outside of it
float applyDeadband(float value, float deadband);

// Limits 'value' to within +- 'limit' (limit should be positive)
template <class T>
T limit(T value, T limit) {
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

/* Rescales joystick value from [1..-1] to [0..1] (this includes flipping the
 * range)
 */
float joystickRescale(float value);

#endif // UTILITY_HPP
