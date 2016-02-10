/*
 * Arm.hpp
 *
 *  Created on: Feb 9, 2016
 *      Author: nad
 */

#ifndef SRC_SUBSYSTEMS_ARM_HPP_
#define SRC_SUBSYSTEMS_ARM_HPP_

#include "SubsystemBase.hpp"
#include "../StateMachine.hpp"
#include "../WPILib/PIDController.hpp"
#include "Timer.h"
#include "../WPILib/CANTalon.h"
#include "DigitalInput.h"

#include <thread>
#include <atomic>

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
    CANTalon m_leftArmActuator{8};  // TODO: Change ID
    CANTalon m_rightArmActuator{9};  // TODO: Change ID
    CANTalon m_carriagePositionMotor{10};  // TODO Change ID

    DigitalInput m_bottomLeftLimitSwitch{1};
    DigitalInput m_bottomRightLimitSwitch{2};
};




#endif /* SRC_SUBSYSTEMS_ARM_HPP_ */
