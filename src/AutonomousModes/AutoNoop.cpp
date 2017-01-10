// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include "../Robot.hpp"

using namespace std::chrono_literals;

// Does nothing
void Robot::AutoNoop() {
    while (IsAutonomous() && IsEnabled()) {
        DS_PrintOut();

        std::this_thread::sleep_for(10ms);
    }
}
