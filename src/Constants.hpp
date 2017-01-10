// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

// Includes definition for Talons and etc that connect to the RoboRIO

/* Used to flip directions of some motors on the practice robot with respect to
 * the competition one
 */
#define PRACTICE_ROBOT

/* Order of subsystem constants:
 * > Motor IDs
 * > Limit switches
 * > Distance per pulse
 * > PID
 * > Other (i.e. miscellaneous constants)
 */

// DS port
constexpr double k_dsPort = 1130;

/*
 * Joystick and buttons
 */

// Joystick ports
constexpr int k_driveStick1Port = 0;
constexpr int k_driveStick2Port = 1;
constexpr int k_shootStickPort = 2;
constexpr int k_armStickPort = 3;

// Joystick axis deadband range
constexpr double k_joystickDeadband = 0.02;

// Arm buttons
constexpr int k_armZeroButton = 11;
constexpr int k_armCarryingButton = 12;

/*
 * DriveTrain
 */

// DriveTrain GearBox ID
constexpr int k_leftDriveMasterID = 8;
constexpr int k_leftDriveSlaveID = 10;
constexpr int k_rightDriveMasterID = 4;
constexpr int k_rightDriveSlaveID = 6;

// DriveTrain distance per pulse
constexpr double k_driveDpP = 36.0 / 575.0;  // in/pulse

// Differential DriveTrain PID
constexpr double k_diffDriveMaxSpeed = 15.600;  // in/sec
constexpr double k_diffDriveP = 0.015;
constexpr double k_diffDriveI = 0.007;
constexpr double k_diffDriveD = 0.0;
constexpr double k_diffDriveA = 0.0;
constexpr double k_diffDriveV = 0.0;

// CheesyDrive constants
constexpr double k_lowGearSensitive = 0.75;
constexpr double k_turnNonLinearity = 1.0;
constexpr double k_inertiaDampen = 2.5;
constexpr double k_inertiaHighTurn = 3.0;
constexpr double k_inertiaLowTurn = 3.0;

/*
 * Arm
 */

// Arm GearBox ID
constexpr int k_leftArmLiftID = 7;

// Arm carriage and winch ID's
constexpr int k_carriageID = 20;  // Remove
constexpr int k_winchID = 9;
constexpr int k_armIntakeID = 11;  // TODO: CHANGE

// Arm limit switch channels
constexpr int k_leftArmBottomLimitChannel = 0;
constexpr int k_leftArmTopLimitChannel = -1;

// Arm carriage limit switch channels
constexpr int k_carriageLeftLimitChannel = 8;
constexpr int k_carriageRightLimitChannel = 7;

// Arm distance per pulse
constexpr double k_armDpP = 30.0 / 133.0;  // degrees/pulse

// Arm height constants
constexpr int k_armHeight1 = 1;
constexpr int k_armHeight2 = 2;

// Arm height
constexpr int k_armMin = 0;
constexpr int k_armMax = 80;

/*
 * Shooter
 */

// Shooter GearBox ID's
constexpr int k_leftShooterID = 5;
constexpr int k_rightShooterID = 3;
constexpr int k_shooterHeightID = 2;
constexpr int k_rollBallID = 1;  // TODO: was 11

// Shooter limit switch channels
constexpr int k_shooterIntakeLimitChannel = 2;
constexpr int k_shooterBottomLimitChannel = 1;

/* Shooter distance per pulse unit conversions:
 * X ticks   1 revolution   60 sec
 * ------- * ------------ * ------
 *  1 sec     360 ticks     1 min
 *
 * Shooter has 1:1 gear ratio, so no conversion between driving and driven RPMs
 * needed
 */
constexpr double k_shooterDpP = 60.0 / 360.0;  // RPM/(pulse/sec)

// Shooter speed
constexpr double k_shooterWheelMaxSpeed = 800.0;        // RPM
constexpr double k_shooterHeightMaxSpeed = 24.0 / 0.2;  // Degrees per second

// Left shooter wheel PID
constexpr double k_leftShooterP = 0.001;
constexpr double k_leftShooterI = 0.0;
constexpr double k_leftShooterD = 0.0;
constexpr double k_leftShooterV = 1.0 / k_shooterWheelMaxSpeed;
constexpr double k_leftShooterA = 0.0;

// Right shooter wheel PID
constexpr double k_rightShooterP = 0.001;
constexpr double k_rightShooterI = 0.0;
constexpr double k_rightShooterD = 0.0;
constexpr double k_rightShooterV = 1.0 / k_shooterWheelMaxSpeed;
constexpr double k_rightShooterA = 0.0;

// Shooter height PID
constexpr double k_heightShooterP = 0.008;
constexpr double k_heightShooterI = 0.0008;
constexpr double k_heightShooterD = 0.0375;
constexpr double k_heightShooterV = 1.0 / k_shooterHeightMaxSpeed;
constexpr double k_heightShooterA = 0.0;
constexpr double k_heightShooterF = 0.155;

constexpr double k_shooterTimeMaxV = 3.0;

constexpr double k_shooterHeightDpP = 360;
constexpr double k_shooterHeightMax = 65.0;  // Degrees
constexpr double k_shooterHeightMin = 0.0;   // Degrees
