// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

// Provides generic utility functions

// Zeroes value if it's inside deadband range, and rescales values outside of it
float ApplyDeadband(float value, float deadband);

// Limits 'value' to within +- 'limit' (limit should be positive)
template <class T>
T Limit(T value, T limit) {
    if (value > limit) {
        return limit;
    } else if (value < -limit) {
        return -limit;
    } else {
        return value;
    }
}

/* Rescales joystick value from [1..-1] to [0..rangeMax] (this includes flipping
 * the range)
 */
float JoystickRescale(float value, float rangeMax);
