// =============================================================================
// File Name: AutoDriveForward.cpp
// Description: Drives forward
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "../Robot.hpp"
#include "../DigitalInputHandler.hpp"

void Robot::Sec150AutoDriveFwd() {
    Timer timer;
    timer.Start();
    shooter.SetShooterHeight(60, false);

    while (!timer.HasPeriodPassed(10.0) && IsAutonomous() && IsEnabled()) {
        DS_PrintOut();
        if (timer.Get() < 1.0 &&
            DigitalInputHandler::Get(k_leftArmBottomLimitChannel)->Get()) {
            arm.SetArmHeight(1.0);
        }
        else {
            arm.SetArmHeight(0.0);
        }
        std::this_thread::sleep_for(10ms);
    }
    while (!timer.HasPeriodPassed(1.50) && IsAutonomous() && IsEnabled()) {
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
