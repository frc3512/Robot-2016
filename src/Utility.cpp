// =============================================================================
// Description: Provides generic utility functions
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include <cmath>
#include "Utility.hpp"

float ApplyDeadband(float value, float deadband) {
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

float JoystickRescale(float value, float rangeMax) {
    return (1.f - value) * rangeMax / 2.f;
}
