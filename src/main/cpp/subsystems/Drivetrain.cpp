// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

Drivetrain::Drivetrain() {
    m_sensitivity = k_lowGearSensitive;

    m_leftGrbx.SetInverted(true);

    m_leftEncoder.SetDistancePerPulse(k_driveDpP);
    m_rightEncoder.SetDistancePerPulse(k_driveDpP);

    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);
}

int32_t Drivetrain::GetLeftRaw() const { return m_leftGrbx.Get(); }

int32_t Drivetrain::GetRightRaw() const { return m_rightGrbx.Get(); }

void Drivetrain::Drive(double throttle, double turn, bool isQuickTurn) {
    m_drive.CurvatureDrive(throttle, turn, isQuickTurn);
}

void Drivetrain::SetDeadband(double band) { m_deadband = band; }

void Drivetrain::ReloadPID() {}

void Drivetrain::ResetEncoders() {
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
}

void Drivetrain::SetLeftManual(double value) { m_leftGrbx.Set(value); }

void Drivetrain::SetRightManual(double value) { m_rightGrbx.Set(value); }

units::inch_t Drivetrain::GetLeftDisplacement() {
    return units::inch_t{m_leftEncoder.GetDistance()};
}

units::inch_t Drivetrain::GetRightDisplacement() {
    return units::inch_t{m_rightEncoder.GetDistance()};
}

units::feet_per_second_t Drivetrain::GetLeftRate() {
    return units::feet_per_second_t{m_leftEncoder.GetRate()};
}

units::feet_per_second_t Drivetrain::GetRightRate() {
    return units::feet_per_second_t{m_rightEncoder.GetRate()};
}

double Drivetrain::DiffPIDGet() { return m_drive.PIDGet(); }

/*
 *  PIDState Drivetrain::GetLeftSetpoint() const {
 *   //std::cout << m_leftPID->IsEnabled() << std::endl;
 *   //std::cout << "LeftPID Get: " <<m_leftPID->Get() << std::endl;
 *   return m_leftPID->GetSetpoint();
 *  }
 *
 *  PIDState Drivetrain::GetRightSetpoint() const {
 *   return m_rightPID->GetSetpoint();
 *  }
 *
 *  PIDState Drivetrain::GetLeftGoal() const {
 *   return m_leftProfile->GetGoal();
 *  }
 *  void Drivetrain::SetGoal(PIDState goal) {
 *   m_leftProfile->SetGoal(goal);
 *   m_rightProfile->SetGoal(goal);
 *  }
 *
 *  bool Drivetrain::AtGoal() const {
 *   return m_leftProfile->AtGoal() && m_rightProfile->AtGoal();
 *  }
 *
 *  void Drivetrain::ResetProfile() {
 *   m_leftProfile->ResetProfile();
 *   m_rightProfile->ResetProfile();
 *  }
 */
