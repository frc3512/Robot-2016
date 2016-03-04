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
constexpr int k_shooterIntakeLimitPin = 2;
constexpr int k_shooterBottomLimitPin = 1;

// Arm limit switch pin
constexpr int k_armBottomLimitPin = 3;
constexpr int k_armTopLimitPin = 4;
constexpr int k_rightCarriageLimitPin = 7;
constexpr int k_leftCarriageLimitPin = 8;

// Drivetrain gearbox ID
constexpr int k_leftDriveMasterID = 8;
constexpr int k_leftDriveSlaveID = 10;
constexpr int k_rightDriveMasterID = 4;
constexpr int k_rightDriveSlaveID = 6;

// CheesyDrive constants
constexpr double k_lowGearSensitive = 0.75;
constexpr double k_turnNonLinearity = 1.0;
constexpr double k_inertiaDampen = 2.5;
constexpr double k_inertiaHighTurn = 3.0;
constexpr double k_inertiaLowTurn = 3.0;

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

// Arm Carriage and Winch ID's
constexpr int k_carriagePositionID = 2;    // TODO: Change ID
constexpr int k_winchPositionID = 9;

// Shooter constants
constexpr double k_shooterWheelMaxSpeed = 800.0;

// Left shooter wheel PID
constexpr double k_leftShooterP = 0.0;
constexpr double k_leftShooterI = 0.0;
constexpr double k_leftShooterD = 0.0;
constexpr double k_leftShooterV = 1.0 / k_shooterWheelMaxSpeed;
constexpr double k_leftShooterA = 0.0;

// Right shooter wheel PID
constexpr double k_rightShooterP = 0.0;
constexpr double k_rightShooterI = 0.0;
constexpr double k_rightShooterD = 0.0;
constexpr double k_rightShooterV = 1.0 / k_shooterWheelMaxSpeed;
constexpr double k_rightShooterA = 0.0;

// Shooter height PID
constexpr double k_heightShooterP = 0.0;
constexpr double k_heightShooterI = 0.0;
constexpr double k_heightShooterD = 0.0;
constexpr double k_heightShooterV = 0.0;
constexpr double k_heightShooterA = 0.0;
constexpr double k_heightShooterF = 0.125;

// Arm gearbox ID
constexpr int k_leftArmLiftID = 7;

// Arm Limit Switch Pin
constexpr int k_armLeftBottomLimitPin = 3;
constexpr int k_armLeftTopLimitPin = 5;

// Arm height constants
constexpr int k_armHeight1 = 1;
constexpr int k_armHeight2 = 2;

// Arm buttons
constexpr int k_armZeroButton = 11;
constexpr int k_armCarryingButton = 12;

#endif // CONSTANTS_HPP
