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
    dsDisplay.AddAutoMethod("2 Sec Drive Forward", &Robot::Sec2AutoDriveFwd, this);
    dsDisplay.AddAutoMethod("3 Sec Drive Forward", &Robot::Sec3AutoDriveFwd, this);
    dsDisplay.AddAutoMethod("3 Sec / 75% Sp Drive Forward", &Robot::Sec3Sp75AutoDriveFwd, this);


    pidGraph.SetSendInterval(5ms);

    displayTimer.Start();
}

void Robot::OperatorControl() {
    while (IsEnabled() && IsOperatorControl()) {
        // Enables QuickTurn if button is pressed
        robotDrive.Drive(-driveStick1.GetY(), driveStick2.GetX(),
                         driveStick2.GetRawButton(2));

//        if (shootButtons.PressedButton(3)) {
//            shooter.SetManualOverride(!shooter.GetManualOverride());
//        }

        // shooter.SetShooterSpeed(JoystickRescale(shootStick.GetThrottle(), 1.f));
        // shooter.SetShooterHeight(shootStick.GetY()); // TODO: Change back to GetY and shootStick

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

        /*
         *  std::cout << "SHOOTER HEIGHT: " << shooter.GetShootHeightValue() <<
         *   std::endl;
         *  std::cout << "HEIGHT THROTTLE: " << JoystickRescale(armStick.GetThrottle(), 1.f) << std::endl;
         *
         *
         *  std::cout << "LEFT SHOOTER WHEEL: " << shooter.GetLeftRPM() <<
         *   std::endl;
         *  std::cout << "RIGHT SHOOTER WHEEL: " << shooter.GetRightRPM() <<
         *   std::endl;
         *  std::cout << "LEFT DRIVE: " << robotDrive.GetLeftDisplacement() <<
         *   std::endl;
         *  std::cout << "RIGHT DRIVE: " << robotDrive.GetRightDisplacement() <<
         *   std::endl;
         */
        //shootButtons.Update(); // TODO: UNCOMMENT

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
        std::cout << "PRACTICE MODE" << std::endl;

        // shooter.SetShooterSpeed((shootStick.GetThrottle() + 1.0) / 2.0);


        // shooter.SetShooterSpeed(0.5);
        // shooter.SetShooterHeight(shootStick.GetY());


        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }
}

void Robot::DS_PrintOut() {
    if (pidGraph.HasIntervalPassed()) {
        pidGraph.GraphData(shooter.GetLeftRPM(), "Left RPM");
        pidGraph.GraphData(shooter.GetRightRPM(), "Right RPM");

        pidGraph.GraphData(
            shooter.GetShooterHeightSetpoint().displacement, "ShtHei SP");
        pidGraph.GraphData(shooter.GetShooterHeightValue(), "Sht Height POS");
        // pidGraph.GraphData(
        //    robotDrive.GetLeftSetpoint().displacement, "Left SP (DR)");
        pidGraph.GraphData(robotDrive.GetLeftDisplacement(), "Left PV (DR)");
        // pidGraph.GraphData(robotDrive.GetLeftSetpoint().displacement,
        //                   "Left SP (DR)");
        pidGraph.GraphData(robotDrive.GetRightDisplacement(), "Right PV (DR)");
        // pidGraph.GraphData(robotDrive.GetRightSetpoint().displacement,
        //                   "Right SP (DR)");
        pidGraph.GraphData(robotDrive.DiffPIDGet(), "Diff PID (DR)");

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
    std::cout << "ARM HEIGHT: " << arm.GetArmHeightValue() << std::endl;
}

START_ROBOT_CLASS(Robot);
