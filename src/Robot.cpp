// =============================================================================
// File Name: Robot.cpp
// Description: Implements the main robot class
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "Robot.hpp"
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
            shooter.setManualShooterSpeed(1 -(shootStick.GetThrottle()) / 2);
        }
        else {
            shooter.setLeftShooterSpeed(1 -(driveStick2.GetThrottle()) / 2);
            shooter.setRightShooterSpeed(1 -(shootStick.GetThrottle()) / 2);
        }
        std::cout<<"Drive Stick Throttle: "<< driveStick2.GetThrottle() <<" | "<<"Shoot Stick Throttle: " <<shootStick.GetThrottle()<<std::endl;
        std::cout<<"Left RPM: "<<shooter.getRPMLeft()<<" | "<<"Right RPM: "<<shooter.getRPMRight()<<std::endl;

        if (shootButton.pressedButton(2)) {
            shooter.shoot();
        }
        // Update the elevator automatic stacking state

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
	while(IsEnabled() && IsTest()) {

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

float Robot::applyDeadband(float value, float deadband) {
    if (fabs(value) > deadband) {
        if (value > 0) {
            return (value - deadband) / (1 - deadband);
        }
        else {
            return (value + deadband) / (1 - deadband);
        }
    }
    else {
        return 0.f;
    }
}

START_ROBOT_CLASS(Robot);

