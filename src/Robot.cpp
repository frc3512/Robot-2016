// =============================================================================
// Description: Implements the main robot class
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "Robot.hpp"
#include "Utility.hpp"
#include <cmath>
#include <iostream>

Robot::Robot() {
    dsDisplay.AddAutoMethod("Noop Auton", &Robot::AutoNoop, this);
    dsDisplay.AddAutoMethod("Profile Auton", &Robot::AutoMotionProfile, this);

    pidGraph.SetSendInterval(5ms);

    displayTimer.Start();
}

void Robot::OperatorControl() {
    while (IsEnabled() && IsOperatorControl()) {
        if (driveStick2.GetRawButton(2)) {
            robotDrive.Drive(driveStick1.GetY(), driveStick2.GetX(), true);
        }
        else {
            robotDrive.Drive(driveStick1.GetY(), driveStick2.GetX());
        }


        if (shootButton.PressedButton(3)) {
            shooter.ToggleManualOverride();
        }
        if (shooter.GetManualOverride()) {
            shooter.SetManualShooterSpeed(JoystickRescale(
                                              shootStick.GetThrottle()));
        }
        else {
            shooter.SetPIDShooterSpeed(JoystickRescale(
                                           shootStick.GetThrottle()));
        }
        // FOR CURVING THE BOULDERS ONLY, REMOVE BEFORE FINAL RELEASE!
        /*else {
         *   shooter.SetLeftShooterSpeed(JoystickRescale(
         *                                   driveStick2.GetThrottle()));
         *   shooter.SetRightShooterSpeed(JoystickRescale(
         *                                    shootStick.GetThrottle()));
         *  }
         */
        std::cout << "Drive Stick Throttle: " << driveStick2.GetThrottle() <<
            " | " << "Shoot Stick Throttle: " << shootStick.GetThrottle() <<
            std::endl;
        std::cout << "Left RPM: " << shooter.GetLeftRPM() << " | " <<
            "Right RPM: " << shooter.GetRightRPM() << std::endl;


        if (shootButton.PressedButton(1)) {
            std::cout << "Trigger Pressed: " << shootStick.GetTrigger() <<
                std::endl;
            shootTimer.Reset();
            shootTimer.Start();
            shooter.Shoot();
        }

        if (shootTimer.HasPeriodPassed(3.0)) {
            shootTimer.Stop();
            shooter.StopIntakeMotor();
        }


        if (shootStick.GetRawButton(2)) {
            shooter.Intake();
        }

        // if (shootStick.GetRawButton(4)) {
        shooter.SetManualShooterHeight(shootStick.GetY());
        // }

        drive1Buttons.UpdateButtons();
        drive2Buttons.UpdateButtons();
        shootButton.UpdateButtons();

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
    while (IsEnabled() && IsTest()) {
    }
}

void Robot::DS_PrintOut() {
    if (pidGraph.HasIntervalPassed()) {
        pidGraph.GraphData(robotDrive.GetLeftDisplacement(), "Left PV (DR)");
        pidGraph.GraphData(
            robotDrive.GetLeftSetpoint().displacement, "Left SP (DR)");
        pidGraph.GraphData(robotDrive.GetRightDisplacement(), "Right PV (DR)");
        pidGraph.GraphData(
            robotDrive.GetRightSetpoint().displacement, "Right SP (DR)");

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
}

START_ROBOT_CLASS(Robot);
