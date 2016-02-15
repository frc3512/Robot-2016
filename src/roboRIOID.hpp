// =============================================================================
// Description: Includes definition for Talons and etc that connect to the
//              roboRIO
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef ROBORIOID_HPP
#define ROBORIOID_HPP

// Joystick and button port
constexpr int k_driveStick1Port = 0;
constexpr int k_driveStick2Port = 1;
constexpr int k_shootStickPort = 2;
constexpr int k_armStickPort = 3;

// Shooter gearbox ID
constexpr int k_leftShooterID = 3;
constexpr int k_rightShooterID = 5;
constexpr int k_shooterHeightID = 2;
constexpr int k_rollBallID = 1;     // TODO: fix ID

// Shooter limit switch pin
constexpr int k_intakeLimitPin = 1;
constexpr int k_bottomLimitPin = 2;

// Drivetrain gearbox ID
constexpr int k_leftDriveMasterID = 8;
constexpr int k_leftDriveSlaveID = 10;
constexpr int k_rightDriveMasterID = 4;
constexpr int k_rightDriveSlaveID = 6;

// Arm gearbox ID
constexpr int k_leftArmLiftID = 11;     // TODO: Change ID
constexpr int k_rightArmLiftID = 12;     // TODO: Change ID
constexpr int k_carriagePositionID = 25;     // TODO: Change ID

// Arm limit switch pin
constexpr int k_bottomLeftLimitSwitchPin = 3;     // TODO: Confirm these port numbers
constexpr int k_bottomRightLimitSwitchPin = 4;     // TODO: Confirm these port numbers

#endif // ROBORIOID_HPP
