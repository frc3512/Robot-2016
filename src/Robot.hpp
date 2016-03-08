// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

#include <CameraServer.h>
#include <Joystick.h>
#include <SampleRobot.h>

#include "ButtonTracker.hpp"
#include "Constants.hpp"
#include "DSDisplay.hpp"
#include "ITG3200.hpp"
#include "LiveGrapher/GraphHost.hpp"
#include "Subsystems/Arm.hpp"
#include "Subsystems/DriveTrain.hpp"
#include "Subsystems/Shooter.hpp"

/**
 * Implements the main robot class
 */
class Robot : public SampleRobot {
public:
    Robot();
    void OperatorControl();
    void Autonomous();
    void Disabled();
    void Test();

    void AutoNoop();
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
    ButtonTracker armButtons{k_armStickPort};

    frc::Timer autoTimer;
    frc::Timer displayTimer;

    // Used for sending data to the Driver Station
    DSDisplay& dsDisplay{DSDisplay::GetInstance(k_dsPort)};

    // The LiveGrapher host
    GraphHost pidGraph{3513};

    // Camera
    // frc::CameraServer* camera = frc::CameraServer::GetInstance();

    ITG3200 gyro{I2C::kOnboard};
};
