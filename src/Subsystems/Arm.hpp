// =============================================================================
// Description: Provides an interface for the robot's arm
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef ARM_HPP
#define ARM_HPP

#include "SubsystemBase.hpp"
#include "GearBox.hpp"
#include "../StateMachine.hpp"
#include "../WPILib/CANTalon.h"
#include <DigitalInput.h>
#include <Timer.h>

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
    GearBox m_leftArmActuator{-1, 8};  // TODO: Change ID
    std::shared_ptr<PIDController> m_leftArmPID;
    std::shared_ptr<TrapezoidProfile> m_leftArmProfile;

    GearBox m_rightArmActuator{-1, 9};  // TODO: Change ID
    std::shared_ptr<PIDController> m_rightArmPID;
    std::shared_ptr<TrapezoidProfile> m_rightArmProfile;

    GearBox m_carriagePositionMotor{-1, 10};  // TODO Change ID
    std::shared_ptr<PIDController> m_carriagePositionPID;
    std::shared_ptr<TrapezoidProfile> m_carriagePositionProfile;

    DigitalInput m_bottomLeftLimitSwitch{1};
    DigitalInput m_bottomRightLimitSwitch{2};
};

#endif // ARM_HPP
