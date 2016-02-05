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

    robotDrive.SetControlMode(CANTalon::kPosition);

    BezierCurve curve;
    curve.AddPoint(0.0, 0.0);
    curve.AddPoint(0.0, 50.0);
    curve.AddPoint(0.0, 100.0);
    curve.AddPoint(0.0, 150.0);

    robotDrive.ResetEncoders();

    autoTimer.Reset();

    // Move robot forward
    robotDrive.SetCurveGoal(curve, autoTimer.Get());
    while (IsAutonomous() && IsEnabled() && !robotDrive.AtGoal()) {
        DS_PrintOut();

        robotDrive.UpdateSetpoint(autoTimer.Get());
        robotDrive.SetLeftSetpoint(
            robotDrive.BezierTrapezoidProfile::GetLeftSetpoint());
        robotDrive.SetRightSetpoint(
            robotDrive.BezierTrapezoidProfile::GetRightSetpoint());

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
