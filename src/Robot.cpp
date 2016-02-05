// =============================================================================
// File Name: Robot.cpp
// Description: Implements the main robot class
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "Robot.hpp"
#include "Utility.hpp"
#include <cmath>
#include <iostream>

Robot::Robot() {
    pidGraph.setSendInterval(5ms);

    displayTimer.Start();
}

void Robot::OperatorControl() {
    while (IsEnabled() && IsOperatorControl()) {
        if (driveStick2.GetRawButton(2)) {
            robotDrive.drive(driveStick1.GetY(), driveStick2.GetX(), true);
        }
        else {
            robotDrive.drive(driveStick1.GetY(), driveStick2.GetX());
        }

        // FOR CURVING THE BOULDERS ONLY, REMOVE BEFORE FINAL RELEASE!
        if (shootStick.GetRawButton(2)) {
            shooter.setManualShooterSpeed(joystickRescale(shootStick.GetThrottle()));
        }
        else {
            shooter.setLeftShooterSpeed(joystickRescale(driveStick2.GetThrottle()));
            shooter.setRightShooterSpeed(joystickRescale(shootStick.GetThrottle()));
        }
        std::cout << "Drive Stick Throttle: " << driveStick2.GetThrottle() <<
            " | " << "Shoot Stick Throttle: " << shootStick.GetThrottle() <<
            std::endl;
        std::cout << "Left RPM: " << shooter.getRPMLeft() << " | " <<
            "Right RPM: " << shooter.getRPMRight() << std::endl;

        if (shootButton.pressedButton(2)) {
            shooter.shoot();
        }
        // Update the elevator automatic stacking state
        if (shootStick.GetRawButton(4)) {
            shooter.setManualShooterPosition(shootStick.GetY());
        }
        // Moves shooter up and down
        drive1Buttons.updateButtons();
        drive2Buttons.updateButtons();
        shootButton.updateButtons();

        DS_PrintOut();

        std::this_thread::sleep_for(10ms);
    }
}

void Robot::Autonomous() {
    autoTimer.Reset();
    autoTimer.Start();

    robotDrive.resetEncoders();
    dsDisplay.execAutonomous();
}

void Robot::Disabled() {
    while (IsDisabled()) {
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }

    robotDrive.reloadPID();
}

void Robot::Test() {
    while (IsEnabled() && IsTest()) {
    }
}

void Robot::DS_PrintOut() {
    if (pidGraph.hasIntervalPassed()) {
        pidGraph.graphData(robotDrive.getLeftDist(), "Left PV (DR)");
        pidGraph.graphData(robotDrive.getLeftSetpoint(), "Left SP (DR)");
        pidGraph.graphData(robotDrive.getRightDist(), "Right PV (DR)");
        pidGraph.graphData(robotDrive.getRightSetpoint(), "Right SP (DR)");

        pidGraph.resetInterval();
    }

    if (displayTimer.HasPeriodPassed(0.5)) {
        // Send things to DS display
        dsDisplay.clear();

        dsDisplay.addData("ENCODER_LEFT", robotDrive.getLeftDist());
        dsDisplay.addData("ENCODER_RIGHT", robotDrive.getRightDist());

        dsDisplay.sendToDS();
    }

    dsDisplay.receiveFromDS();
}

START_ROBOT_CLASS(Robot);
