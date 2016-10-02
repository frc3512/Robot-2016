// Copyright (c) FRC Team 3512, Spartatroniks 2016. All Rights Reserved.

#pragma once

#include <cmath>

/**
 * Data type for PID controller state
 */
struct PIDState {
    PIDState() = default;
    PIDState(double displacement, double velocity, double acceleration) {
        this->displacement = displacement;
        this->velocity = velocity;
        this->acceleration = acceleration;
    }

    PIDState& operator+=(const PIDState& rhs) {
        displacement += rhs.displacement;
        velocity += rhs.velocity;
        acceleration += rhs.acceleration;

        return *this;
    }

    PIDState operator+(const PIDState& rhs) {
        PIDState result = *this;
        result += rhs;
        return result;
    }

    PIDState& operator-=(const PIDState& rhs) {
        displacement -= rhs.displacement;
        velocity -= rhs.velocity;
        acceleration -= rhs.acceleration;

        return *this;
    }

    PIDState operator-(const PIDState& rhs) {
        PIDState result = *this;
        result -= rhs;
        return result;
    }

    bool operator==(const PIDState& rhs) const {
        return std::fabs(displacement - rhs.displacement) < 0.001 &&
               std::fabs(velocity - rhs.velocity) < 0.001 &&
               std::fabs(acceleration - rhs.acceleration) < 0.001;
    }

    bool operator!=(const PIDState& rhs) const { return !(*this == rhs); }

    double displacement = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
};
