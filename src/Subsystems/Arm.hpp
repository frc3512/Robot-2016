// =============================================================================
// Description: Provides an interface for the robot's arm
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef ARM_HPP
#define ARM_HPP

#include <DigitalInput.h>
#include <Timer.h>

#include "../Constants.hpp"
#include "../MotionProfile/TrapezoidProfile.hpp"
#include "GearBox.hpp"
#include "SubsystemBase.hpp"

class PIDController;
class TrapezoidProfile;

class Arm : public SubsystemBase {
public:
    Arm();

    void ReloadPID();
    void ResetEncoders();

    void SetManualArmHeight(double height);
    void SetPIDArmHeight(double height);
    void SetManualCarriagePosition(double position);

    void UpdateState();

private:
    GearBox m_leftArmActuator{-1, k_leftArmLiftID};
    std::shared_ptr<PIDController> m_leftArmPID;
    std::shared_ptr<TrapezoidProfile> m_leftArmProfile;

    GearBox m_rightArmActuator{-1, k_rightArmLiftID};
    std::shared_ptr<PIDController> m_rightArmPID;
    std::shared_ptr<TrapezoidProfile> m_rightArmProfile;

    GearBox m_carriagePositionMotor{-1, k_carriagePositionID};
    std::shared_ptr<PIDController> m_carriagePositionPID;
    std::shared_ptr<TrapezoidProfile> m_carriagePositionProfile;

    DigitalInput m_bottomLeftLimitSwitch{k_bottomLeftLimitSwitchPin};
    DigitalInput m_bottomRightLimitSwitch{k_bottomRightLimitSwitchPin};

    std::shared_ptr<TrapezoidProfile> m_rightArmHeightProfile;
    std::shared_ptr<TrapezoidProfile> m_leftArmHeightProfile;
};

#endif // ARM_HPP
