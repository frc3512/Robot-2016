// =============================================================================
// Description: Includes definition for Talons and etc that connect to the
//              roboRIO
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef ROBORIOID_HPP
#define ROBORIOID_HPP

    //DS port
    constexpr double k_dsPort = 1130;

    // Joystick and button port
    constexpr int k_driveStick1Port = 0;
    constexpr int k_driveStick2Port = 1;
    constexpr int k_shootStickPort = 2;
    constexpr int k_armStickPort = 3;

    // Shooter gearbox ID
    constexpr int k_leftShooterID = 14;
    constexpr int k_rightShooterID = 6;
    constexpr int k_shooterHeightID = 7;
    constexpr int k_rollBallID = 5; // TODO: fix ID

    // Shooter limit switch pin
    constexpr int k_intakeLimitPin = 1;
    constexpr int k_bottomLimitPin = 2;

    // Drivetrain gearbox ID
    constexpr int k_leftDriveMasterID = 1;
    constexpr int k_leftDriveSlaveID = 3;
    constexpr int k_rightDriveMasterID = 4;
    constexpr int k_rightDriveSlaveID = 9;

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
    constexpr int k_leftArmLiftID = 27; // TODO: Change ID
    constexpr int k_rightArmLiftID = 26; // TODO: Change ID
    constexpr int k_carriagePositionID = 25; // TODO: Change ID

    // Arm limit switch pin
    constexpr int k_bottomLeftLimitSwitchPin = 3; // TODO: Confirm these port numbers
    constexpr int k_bottomRightLimitSwitchPin = 4; // TODO: Confirm these port numbers

    // Other settings
    constexpr double k_lowGearSensitive = 0.75;
	constexpr double k_turnNonLinearity = 1.0;
	constexpr double k_inertia = 5.0;
	constexpr double k_inertiaDampen = 2.5;
	constexpr double k_inertiaHighTurn = 5.0;
	constexpr double k_inertiaLowTurn = 3.0;

#endif // ROBORIOID_HPP
