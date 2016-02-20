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
constexpr int k_leftShooterID = 3;
constexpr int k_rightShooterID = 5;
constexpr int k_shooterHeightID = 2;
constexpr int k_rollBallID = 1;     // TODO: fix ID

// Shooter limit switch pin
constexpr int k_intakeLimitPin = 1;
constexpr int k_bottomLimitPin = 2;
constexpr int k_armBottomLimitPin = 3;
constexpr int k_armTopLimitPin = 4;
constexpr int k_rightCarriageLimitPin = 5;
constexpr int k_leftCarriageLimitPin = 6;

const int k_armHeight1 = 1;
const int k_armHeight2 = 2;

// Drivetrain gearbox ID
constexpr int k_leftDriveMasterID = 8;
constexpr int k_leftDriveSlaveID = 10;
constexpr int k_rightDriveMasterID = 4;
constexpr int k_rightDriveSlaveID = 6;

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

// Arm gearbox ID
constexpr int k_leftArmLiftID = 11;     // TODO: Change ID
constexpr int k_rightArmLiftID = 12;     // TODO: Change ID
constexpr int k_carriagePositionID = 25;     // TODO: Change ID

// Arm limit switch pin
constexpr int k_bottomLeftLimitSwitchPin = 3;     // TODO: Confirm these port numbers
constexpr int k_bottomRightLimitSwitchPin = 4;     // TODO: Confirm these port numbers

// Other settings
constexpr double k_lowGearSensitive = 0.75;
constexpr double k_turnNonLinearity = 1.0;
constexpr double k_inertia = 5.0;
constexpr double k_inertiaDampen = 2.5;
constexpr double k_inertiaHighTurn = 5.0;
constexpr double k_inertiaLowTurn = 3.0;

#endif // CONSTANTS_HPP
