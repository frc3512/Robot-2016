// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include "GearBox.hpp"

#include <iostream>
#include <memory>

#include "../DigitalInputHandler.hpp"

GearBox::GearBox(int shifterChan, int forwardLimitPin, int reverseLimitPin,
                 int motor1, int motor2, int motor3) {
    if (shifterChan != -1) {
        m_shifter = std::make_unique<frc::Solenoid>(shifterChan);
    }

    if (forwardLimitPin != -1) {
        m_forwardLimit = DigitalInputHandler::Get(forwardLimitPin);
    }

    if (reverseLimitPin != -1) {
        m_reverseLimit = DigitalInputHandler::Get(reverseLimitPin);
    }

    // Create motor controllers of specified template type
    m_motors.emplace_back(std::make_unique<frc::CANTalon>(motor1));
    if (motor2 != -1) {
        m_motors.emplace_back(std::make_unique<frc::CANTalon>(motor2));
    }
    if (motor3 != -1) {
        m_motors.emplace_back(std::make_unique<frc::CANTalon>(motor3));
    }

    for (unsigned int i = 0; i < m_motors.size(); i++) {
        if (i == 0) {
            m_motors[i]->SetControlMode(frc::CANTalon::kPercentVbus);
            m_motors[i]->SetFeedbackDevice(frc::CANTalon::QuadEncoder);
            m_motors[i]->ConfigEncoderCodesPerRev(1);
            m_motors[i]->SetSensorDirection(m_isEncoderReversed);
            ResetEncoder();
            // m_motors[i]->SetVoltageRampRate(k_voltRampRate);
            m_motors[i]->SelectProfileSlot(0);
            m_motors[i]->EnableControl();
        } else {
            // Use all other CANTalons as slaves
            m_motors[i]->SetControlMode(frc::CANTalon::kFollower);

            // Set first CANTalon as master
            m_motors[i]->Set(motor1);
        }
    }
}

void GearBox::Set(double value) {
#if 0
    if (m_forwardLimit != nullptr && m_limitOnHigh == m_forwardLimit->Get() &&
        value > 0) {
        /* If stopping motor in same limit switch state that limit switch is in
         * now and motor is rotating into limit switch
         */
        m_motors[0]->Set(0);
    } else if (m_reverseLimit != nullptr &&
             m_limitOnHigh == m_reverseLimit->Get() && value < 0) {
        /* If stopping motor in same limit switch state that limit switch is in
         * now and motor is rotating into limit switch
         */
        m_motors[0]->Set(0);
    } else if (GetPosition() >= m_max && value > 0) {
        /* If the current position is past the soft limit and motor is rotating
         * into limit switch
         */
        m_motors[0]->Set(0);
    } else if (GetPosition() <= m_min && value < 0) {
        /* If the current position is past the soft limit and motor is rotating
         * into limit switch
         */
        m_motors[0]->Set(0);
    } else {
        m_motors[0]->Set(value);
    }
#endif
    m_motors[0]->Set(value);  // TODO: Remove once #endif above is removed
}

double GearBox::Get() const { return m_motors[0]->GetPosition(); }

double GearBox::GetPosition() const {
    return m_motors[0]->GetPosition() * m_distancePerPulse;
}

double GearBox::GetSpeed() const {
    if (m_feedbackDevice == frc::CANTalon::CtreMagEncoder_Relative ||
        m_feedbackDevice == frc::CANTalon::CtreMagEncoder_Absolute) {
        // RPM * degrees/rev / (seconds/min)
        return m_motors[0]->GetSpeed() * m_distancePerPulse / 60.0;
    } else {
        // std::cout << "Else: " << m_feedbackDevice << std::endl;
        return m_motors[0]->GetSpeed() * m_distancePerPulse * 100.0;
    }
}

void GearBox::SetDistancePerPulse(double distancePerPulse) {
    m_distancePerPulse = distancePerPulse;
}

void GearBox::SetFeedbackDevice(frc::CANTalon::FeedbackDevice device) {
    std::cout << "SetFeedbackDevice " << device << std::endl;
    m_feedbackDevice = device;
    m_motors[0]->SetFeedbackDevice(device);
}

void GearBox::ResetEncoder() { m_motors[0]->SetPosition(0); }

void GearBox::SetInverted(bool reverse) { m_motors[0]->SetInverted(reverse); }

bool GearBox::GetInverted() const { return m_motors[0]->GetInverted(); }

void GearBox::SetSensorDirection(bool reverse) {
    m_isEncoderReversed = reverse;
    m_motors[0]->SetSensorDirection(m_isEncoderReversed);
}

bool GearBox::IsEncoderReversed() const { return m_isEncoderReversed; }

void GearBox::SetLimitOnHigh(bool limitOnHigh) { m_limitOnHigh = limitOnHigh; }

void GearBox::SetSoftPositionLimits(double min, double max) {
    m_min = min;
    m_max = max;
}

void GearBox::SetGear(bool gear) {
    if (m_shifter != nullptr) {
        m_shifter->Set(gear);
    }
}

bool GearBox::GetGear() const {
    if (m_shifter != nullptr) {
        return m_shifter->Get();
    } else {
        return false;
    }
}

frc::CANTalon* GearBox::GetMaster() const { return m_motors[0].get(); }

void GearBox::PIDWrite(double output) {
    if (m_forwardLimit != nullptr && m_limitOnHigh == m_forwardLimit->Get() &&
        output > 0) {
        /* If stopping motor in same limit switch state that limit switch is in
         * now and motor is rotating into limit switch
         */
        m_motors[0]->PIDWrite(0);
        std::cout << "forwardLimit" << std::endl;
    } else if (m_reverseLimit != nullptr &&
               m_limitOnHigh == m_reverseLimit->Get() && output < 0) {
        /* If stopping motor in same limit switch state that limit switch is in
         * now and motor is rotating into limit switch
         */
        m_motors[0]->PIDWrite(0);
        std::cout << "reverseLimit" << std::endl;
    } else if (GetPosition() >= m_max && output > 0) {
        /* If the current position is past the soft limit and motor is rotating
         * into limit switch
         */
        m_motors[0]->PIDWrite(0);
        // std::cout << "max" << std::endl;
    } else if (GetPosition() <= m_min && output < 0) {
        /* If the current position is past the soft limit and motor is rotating
         * into limit switch
         */
        m_motors[0]->PIDWrite(0);
        // std::cout << "min" << std::endl;
    } else {
        m_motors[0]->PIDWrite(output);
    }
}

double GearBox::PIDGet() {
    if (GetPIDSourceType() == PIDSourceType::kRate) {
        return GetSpeed();
    } else {
        return GetPosition();
    }
}
