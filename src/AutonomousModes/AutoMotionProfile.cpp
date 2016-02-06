// =============================================================================
// Description: Drives forward
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "../Robot.hpp"

void Robot::AutoMotionProfile() {
    robotDrive.SetLeftManual(0.f);
    robotDrive.SetRightManual(0.f);
    robotDrive.SetLeftSetpoint(PIDState());
    robotDrive.SetRightSetpoint(PIDState());
    robotDrive.ResetProfile();

    robotDrive.ResetEncoders();

    autoTimer.Reset();

    // Move robot forward
    robotDrive.SetGoal(PIDState(150.0, 0.0, 0.0));
    while (IsAutonomous() && IsEnabled() && !robotDrive.AtGoal()) {
        DS_PrintOut();

        std::this_thread::sleep_for(10ms);
    }

    // Stop moving
    robotDrive.SetLeftManual(0.f);
    robotDrive.SetRightManual(0.f);

    while (IsAutonomous() && IsEnabled()) {
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }
}
