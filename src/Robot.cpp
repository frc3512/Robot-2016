// =============================================================================
// Description: Implements the main robot class
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "Robot.hpp"

#include <chrono>
using namespace std::chrono_literals;

#include "Utility.hpp"

Robot::Robot() {
    dsDisplay.AddAutoMethod("Noop Auton", &Robot::AutoNoop, this);
    dsDisplay.AddAutoMethod("2 Sec Drive Forward",
                            &Robot::Sec2AutoDriveFwd,
                            this);
    dsDisplay.AddAutoMethod("3 Sec Drive Forward",
                            &Robot::Sec3AutoDriveFwd,
                            this);
    dsDisplay.AddAutoMethod("3 Sec / 75% Sp Drive Forward",
                            &Robot::Sec3Sp75AutoDriveFwd,
                            this);


    pidGraph.SetSendInterval(5ms);

    displayTimer.Start();
}

void Robot::OperatorControl() {
    while (IsEnabled() && IsOperatorControl()) {
        // Enables QuickTurn if button is pressed
        robotDrive.Drive(-driveStick1.GetY(), driveStick2.GetX(),
                         driveStick2.GetRawButton(2));


        shooter.SetShooterSpeed(JoystickRescale(shootStick.GetThrottle(), 1.f));
        shooter.SetShooterHeight(ApplyDeadband(shootStick.GetY(),
                                               k_joystickDeadband));                    // TODO: Change back to GetY and shootStick

        if (shootButtons.PressedButton(3)) {
            shooter.SetShooterHeight(18.0);
        }

        if (shootButtons.PressedButton(4)) {
            shooter.SetShooterHeight(50.0);
        }

        if (armStick.GetPOV() == 0) {
            arm.SetManualWinchHeight(1);
        }
        else if (armStick.GetPOV() == 180) {
            arm.SetManualWinchHeight(-1);
        }
        else {
            arm.SetManualWinchHeight(0);
        }
        arm.SetArmHeight(-armStick.GetY());


        if (armStick.GetPOV() == 90) {
            arm.SetManualCarriagePosition(armStick.GetPOV() * 0.1);
        }
        else if (armStick.GetPOV() == 270) {
            arm.SetManualCarriagePosition(armStick.GetPOV() * 0.1);
        }
        else {
            arm.SetManualCarriagePosition(armStick.GetPOV() * 0.1);
        }

        shootButtons.Update();

        shooter.UpdateState();

        DS_PrintOut();

        std::this_thread::sleep_for(10ms);
    }
}

void Robot::Autonomous() {
    autoTimer.Reset();
    autoTimer.Start();

    robotDrive.ResetEncoders();
    dsDisplay.ExecAutonomous();
}

void Robot::Disabled() {
    while (IsDisabled()) {
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }

    robotDrive.ReloadPID();
}

void Robot::Test() {
    shooter.SetManualOverride(false);
    while (IsEnabled() && IsTest()) {
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }
}

void Robot::DS_PrintOut() {
    if (pidGraph.HasIntervalPassed()) {
        pidGraph.GraphData(
            shooter.GetShooterHeightSetpoint().displacement, "ShtHei SP");
        pidGraph.GraphData(shooter.GetShooterHeight(), "Sht Height POS");
        pidGraph.GraphData(
            shooter.m_shooterHeightGrbx.GetSpeed(), "Sht Hght Spd");
        // pidGraph.GraphData(robotDrive.GetLeftDisplacement(), "Left PV (DR)");
        // pidGraph.GraphData(robotDrive.GetRightDisplacement(), "Right PV (DR)");
        // pidGraph.GraphData(robotDrive.DiffPIDGet(), "Diff PID (DR)");

        pidGraph.ResetInterval();
    }

    if (displayTimer.HasPeriodPassed(0.5)) {
        // Send things to DS display
        dsDisplay.Clear();

        dsDisplay.AddData("ENCODER_LEFT", robotDrive.GetLeftDisplacement());
        dsDisplay.AddData("ENCODER_RIGHT", robotDrive.GetRightDisplacement());

        dsDisplay.SendToDS();
    }
    dsDisplay.ReceiveFromDS();

    std::cout << "Throttle Value: " << JoystickRescale(
        armStick.GetThrottle(),
        1.f) << " Shooter Angle: " << shooter.GetShooterHeight() << std::endl;
}

START_ROBOT_CLASS(Robot);
