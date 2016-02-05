// =============================================================================
// Description: Provides generic utility functions
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "Utility.hpp"
#include <cmath>

float applyDeadband(float value, float deadband) {
    if (std::fabs(value) > deadband) {
        if (value > 0) {
            return (value - deadband) / (1 - deadband);
        }
        else {
            return (value + deadband) / (1 - deadband);
        }
    }
    else {
        return 0.f;
    }
}

float joystickRescale(float value) {
    return (1.f - value) / 2.f;
}
