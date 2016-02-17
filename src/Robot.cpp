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
        // Enables QuickTurn if button is pressed
        robotDrive.Drive(-driveStick1.GetY(), driveStick2.GetX(),
                         driveStick2.GetRawButton(2));

        if (shootButtons.PressedButton(3)) {
            shooter.ToggleManualOverride();
        }

        if (shooter.GetManualOverride()) {
            shooter.SetManualShooterSpeed(JoystickRescale(
                                              shootStick.GetThrottle(), 1.f));
        }
        /*
         *  else {
         *   shooter.SetPIDShooterSpeed(JoystickRescale(
         *                                  shootStick.GetThrottle(), 1.f));
         *  }
         */

        arm.SetManualArmHeight(armStick.GetY());
        arm.SetManualCarriagePosition(armStick.GetPOV());

        shooter.SetManualShooterHeight(shootStick.GetY());

        std::cout << "SHOOTER HEIGHT: " <<shooter.GetShootHeightValue() << std::endl;
        std::cout << "LEFT SHOOTER WHEEL: "<<shooter.GetLeftRPM() << std::endl;
        std::cout << "RIGHT SHOOTER WHEEL: "<<shooter.GetRightRPM() << std::endl;

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
    while (IsEnabled() && IsTest()) {
        std::cout << "PRACTICE MODE" << std::endl;
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
