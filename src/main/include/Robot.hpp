// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>

#include "Constants.hpp"
#include "subsystems/Arm.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Shooter.hpp"

/**
 * Implements the main robot class
 */
class Robot : public TimedRobot {
public:
    Robot();

    void OperatorControl();

    void Autonomous();

    void Disabled();

    void Test();

    void AutoMotionProfile();

    void AutoDriveForward();

    void AutoRoughTerrain();

    void AutoLowBar();

    void AutoPortcullis();

    void DS_PrintOut();

private:
    DriveTrain robotDrive;
    Shooter shooter;
    Arm arm;

    frc::Joystick driveStick1{k_driveStick1Port};
    frc::Joystick driveStick2{k_driveStick2Port};
    frc::Joystick shootStick{k_shootStickPort};
    frc::Joystick armStick{k_armStickPort};

    ButtonTracker shootButtons{k_shootStickPort};

    frc::Timer autoTimer;
    frc::Timer displayTimer;

    // The LiveGrapher host
    GraphHost pidGraph{3513};
};
