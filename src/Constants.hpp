// =============================================================================
// Description: Includes definition for Talons and etc that connect to the
//              RoboRIO
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// DS port
constexpr double k_dsPort = 1130;

// Joystick and button port
constexpr int k_driveStick1Port = 0;
constexpr int k_driveStick2Port = 1;
constexpr int k_shootStickPort = 2;
constexpr int k_armStickPort = 3;

// Shooter gearbox ID
constexpr int k_leftShooterID = 5;
constexpr int k_rightShooterID = 3;
constexpr int k_shooterHeightID = 1;
constexpr int k_rollBallRelay = 0;

// Shooter limit switch pin
constexpr int k_intakeLimitPin = 1;
constexpr int k_bottomLimitPin = 2;
constexpr int k_armBottomLimitPin = 3;
constexpr int k_armTopLimitPin = 4;
constexpr int k_rightCarriageLimitPin = 5;
constexpr int k_leftCarriageLimitPin = 6;

constexpr int k_armHeight1 = 1;
constexpr int k_armHeight2 = 2;

constexpr int k_armZeroButton = 11;
constexpr int k_armCarryingButton = 12;

// Drivetrain gearbox ID
constexpr int k_leftDriveMasterID = 10;
constexpr int k_leftDriveSlaveID = 8;
constexpr int k_rightDriveMasterID = 6;
constexpr int k_rightDriveSlaveID = 4;

// Left drivetrain PID values
constexpr double k_leftDriveP = 5;
constexpr double k_leftDriveI = 0;
constexpr double k_leftDriveD = 2;
constexpr double k_leftDriveV = 0;
constexpr double k_leftDriveA = 0;

// Right drivetrain PID values
constexpr double k_rightDriveP = 8;
constexpr double k_rightDriveI = 0;
constexpr double k_rightDriveD = 3;
constexpr double k_rightDriveV = 0;
constexpr double k_rightDriveA = 0;

// Shooter PID values
constexpr double k_shooterWheelMaxSpeed = 800;

// Arm gearbox ID
constexpr int k_leftArmLiftID = 9;
constexpr int k_rightArmLiftID = 7;
constexpr int k_winchArmID = 1;
constexpr int k_carriagePositionID = 20;

// Arm limit switch pin
constexpr int k_bottomLeftLimitSwitchPin = 3;     // TODO: Confirm these port numbers
constexpr int k_bottomRightLimitSwitchPin = 4;     // TODO: Confirm these port numbers

// CheesyDrive constants
constexpr double k_lowGearSensitive = 0.75;
constexpr double k_turnNonLinearity = 1.0;
constexpr double k_inertiaDampen = 2.5;
constexpr double k_inertiaHighTurn = 3.0;
constexpr double k_inertiaLowTurn = 3.0;

#endif // CONSTANTS_HPP
