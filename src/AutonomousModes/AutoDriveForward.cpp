// =============================================================================
// File Name: AutoDriveForward.cpp
// Description: Drives forward
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "../Robot.hpp"

void Robot::AutoDriveForward() {
    Timer timer;
    timer.Start();

    while (!timer.HasPeriodPassed(2.5) && IsAutonomous() && IsEnabled()) { // TODO: Figure out correct amount of seconds
        DS_PrintOut();
        robotDrive.Drive(-0.7, 0.0, false); // TODO: Determine if fast enough
        std::this_thread::sleep_for(10ms);
    }
    robotDrive.Drive(0.0, 0.0, false);
}
