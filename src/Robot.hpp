// =============================================================================
// Description: Implements the main robot class
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <SampleRobot.h>
#include <Joystick.h>

#include "ButtonTracker.hpp"
#include "Constants.hpp"
#include "DSDisplay.hpp"
#include "LiveGrapherHost/GraphHost.hpp"
#include "Subsystems/Shooter.hpp"
#include "Subsystems/DriveTrain.hpp"
#include "Subsystems/Arm.hpp"

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

    void DS_PrintOut();

private:
    DriveTrain robotDrive;
    Shooter shooter;
    Arm arm;

    Joystick driveStick1{k_driveStick1Port};
    Joystick driveStick2{k_driveStick2Port};
    Joystick shootStick{k_shootStickPort};
    Joystick armStick{k_armStickPort};

    ButtonTracker shootButtons{k_shootStickPort};

    Timer autoTimer;
    Timer displayTimer;

    // Used for sending data to the Driver Station
    DSDisplay& dsDisplay{DSDisplay::GetInstance(k_dsPort)};

    // The LiveGrapher host
    GraphHost pidGraph{3513};
};

#endif // ROBOT_HPP
