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
    dsDisplay.AddAutoMethod("1.75 Sec Drive Forward",
                            &Robot::Sec175AutoDriveFwd,
                            this);
    dsDisplay.AddAutoMethod("3 Sec Drive Forward",
                            &Robot::Sec3AutoDriveFwd,
                            this);
    dsDisplay.AddAutoMethod("3 Sec / 75% Sp Drive Forward",
                            &Robot::Sec3Sp75AutoDriveFwd,
                            this);

    // camera->StartAutomaticCapture();

    pidGraph.SetSendInterval(5ms);

    displayTimer.Start();
}

void Robot::OperatorControl() {
    while (IsEnabled() && IsOperatorControl()) {
        // Enables QuickTurn if button is pressed
        // If trigger is pressed, move at half speed
        if (driveStick1.GetTrigger()) {
            robotDrive.Drive(-driveStick1.GetY() * 0.5,
                             driveStick2.GetX() * 0.5,
                             driveStick2.GetRawButton(2));
        }
        else {
            robotDrive.Drive(-driveStick1.GetY(), driveStick2.GetX(),
                             driveStick2.GetRawButton(2));
        }

        shooter.SetShooterSpeed(JoystickRescale(shootStick.GetThrottle(), 1.f));
        shooter.SetShooterHeight(ApplyDeadband(shootStick.GetY(),
                                               k_joystickDeadband), true);

        if (shootButtons.PressedButton(3)) {
            shooter.SetShooterHeight(18.0, false);
        }

        if (shootButtons.PressedButton(4)) {
            shooter.SetShooterHeight(52.0, false);
        }

        if (shootButtons.PressedButton(5)) {
            shooter.ResetEncoders();
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

        if (armButtons.PressedButton(3)) {
            gyro.Calibrate();
        }

        shootButtons.Update();
        armButtons.Update();

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
        shooter.UpdateState();
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);

        if (armButtons.PressedButton(3)) {
            gyro.Calibrate();
        }

        armButtons.Update();
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

        // pidGraph.GraphData(gyro.GetRateZ(), "Rate Of Z");

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

    std::cout << " Shooter Angle: " << shooter.GetShooterHeight() << std::endl;
    std::cout << " Shooter Angle PID " << shooter.m_shooterHeightPID->Get() <<
    std::endl;
    std::cout << gyro.GetRateZ() << std::endl;
}

START_ROBOT_CLASS(Robot);
