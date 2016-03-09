// =============================================================================
// File Name: AutoDriveForward.cpp
// Description: Drives forward
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "../Robot.hpp"

void Robot::AutoDriveForward() {
    Timer timer;
    timer.Start();

    robotDrive.ResetEncoders();
    robotDrive.EnablePID();
    shooter.SetManualOverride(false);
    shooter.SetShooterHeight(30);
    robotDrive.DiffDrive(0.1);

    while (!timer.HasPeriodPassed(2.0) && IsAutonomous() && IsEnabled()) {
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }

    shooter.SetManualOverride(true);
    robotDrive.DisablePID();
    robotDrive.Drive(0.0, 0.0, false);

    while (IsAutonomous() && IsEnabled()) {
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }
}
