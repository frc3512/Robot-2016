// =============================================================================
// Description: Drives forward
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "../Robot.hpp"
#include <iostream>

void Robot::AutoMotionProfile() {
    robotDrive.SetLeftManual(0.f);
    robotDrive.SetRightManual(0.f);
    robotDrive.ResetProfile();

    robotDrive.ResetEncoders();

    autoTimer.Reset();

    // Move robot forward
    robotDrive.SetGoal(PIDState(24.0, 0.0, 0.0));
    std::cout << "PROFILE AUTON" << std::endl;
    while (IsAutonomous() && IsEnabled() && !robotDrive.AtGoal()) {
        DS_PrintOut();
        std::cout << "SETPOINT: " << robotDrive.GetLeftSetpoint().velocity << " SPEED: " << robotDrive.GetLeftRate() << std::endl;
        std::cout << "GOAL: " << robotDrive.GetLeftGoal().displacement << std::endl;
        std::this_thread::sleep_for(10ms);
    }
    std::cout << "AFTER PROFILE WHILE" << std::endl;
    // Stop moving
    robotDrive.SetLeftManual(0.f);
    robotDrive.SetRightManual(0.f);

    while (IsAutonomous() && IsEnabled()) {
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }
}
