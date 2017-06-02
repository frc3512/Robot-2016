// Copyright (c) 2016-2017 FRC Team 3512. All Rights Reserved.

#include "../DigitalInputHandler.hpp"
#include "../Robot.hpp"

using namespace std::chrono_literals;

// Rough terrain autonomous
void Robot::AutoRoughTerrain() {
    Timer timer;
    timer.Start();
    shooter.SetShooterHeight(60, false);

    while (!timer.HasPeriodPassed(10.0) && IsAutonomous() && IsEnabled()) {
        DS_PrintOut();

        std::this_thread::sleep_for(10ms);
    }
    while (!timer.HasPeriodPassed(1.0) && IsAutonomous() && IsEnabled()) {
        DS_PrintOut();
        robotDrive.Drive(1, 0, false);

        std::this_thread::sleep_for(10ms);
    }

    robotDrive.Drive(0.0, 0.0, false);

    while (IsAutonomous() && IsEnabled()) {
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }
}
