// Copyright (c) 2016-2017 FRC Team 3512. All Rights Reserved.

#include "Utility.hpp"

#include <cmath>

double ApplyDeadband(double value, double deadband) {
    if (std::fabs(value) > deadband) {
        if (value > 0) {
            return (value - deadband) / (1 - deadband);
        } else {
            return (value + deadband) / (1 - deadband);
        }
    } else {
        return 0.0;
    }
}

double JoystickRescale(double value, double rangeMax) {
    return (1.0 - value) * rangeMax / 2.0;
}
