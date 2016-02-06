// =============================================================================
// Description: Data type for PID controller state
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef PID_STATE_HPP
#define PID_STATE_HPP

struct PIDState {
    PIDState() = default;
    PIDState(double displacement, double velocity, double acceleration) {
        this->displacement = displacement;
        this->velocity = velocity;
        this->acceleration = acceleration;
    }

    double displacement = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
};

#endif // PID_STATE_HPP
