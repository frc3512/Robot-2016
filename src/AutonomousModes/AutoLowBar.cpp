// Copyright (c) FRC Team 3512, Spartatroniks 2016. All Rights Reserved.

#include "../DigitalInputHandler.hpp"
#include "../Robot.hpp"

using namespace std::chrono_literals;

// Low bar autonomous
void Robot::AutoLowBar() {
    Timer timer;
    timer.Start();
    // shooter.SetShooterHeight(60, false);

    while (!timer.HasPeriodPassed(2.0) && IsAutonomous() && IsEnabled()) {
        DS_PrintOut();
        if (DigitalInputHandler::Get(k_leftArmBottomLimitChannel)->Get()) {
            arm.SetArmHeight(1.0);
        } else {
            arm.SetArmHeight(0.0);
        }
        std::this_thread::sleep_for(10ms);
    }
    while (!timer.HasPeriodPassed(2.50) && IsAutonomous() && IsEnabled()) {
        DS_PrintOut();
        robotDrive.Drive(-0.5, 0, false);

        std::this_thread::sleep_for(10ms);
    }

    robotDrive.Drive(0.0, 0.0, false);

    while (IsAutonomous() && IsEnabled()) {
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }
}
