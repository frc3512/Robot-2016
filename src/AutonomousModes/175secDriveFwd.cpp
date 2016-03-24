// =============================================================================
// File Name: AutoDriveForward.cpp
// Description: Drives forward
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "../Robot.hpp"

void Robot::Sec175AutoDriveFwd() {
    Timer timer;
    timer.Start();


    shooter.SetShooterHeight(52, false);

    while (!timer.HasPeriodPassed(1.75) && IsAutonomous() && IsEnabled()) {
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
