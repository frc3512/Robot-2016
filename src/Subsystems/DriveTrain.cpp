// =============================================================================
// Description: Provides an interface for this year's drive train
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include "DriveTrain.hpp"

#include <cmath>
#include <CANTalon.h>

const float DriveTrain::maxWheelSpeed = 80.0;

DriveTrain::DriveTrain() : BezierTrapezoidProfile(maxWheelSpeed, 2) {
    m_sensitivity = m_settings.GetDouble("LOW_GEAR_SENSITIVE");

    m_leftGrbx.SetMotorReversed(true);
    m_leftGrbx.SetEncoderReversed(true);

    m_rightGrbx.SetEncoderReversed(true);

    m_leftGrbx.SetDistancePerPulse(72.0 / 2800.0);
    m_rightGrbx.SetDistancePerPulse(72.0 / 2800.0);

    m_leftGrbx.SetManual(0.0);
    m_rightGrbx.SetManual(0.0);

    SetWidth(27.0);

    ReloadPID();
}

void DriveTrain::Drive(float throttle, float turn, bool isQuickTurn) {
    // Modified Cheesy Drive; base code courtesy of FRC Team 254

    throttle *= -1;

    // Limit values to [-1 .. 1]
    throttle = Limit(throttle, 1.f);
    turn = Limit(turn, 1.f);

    /* Apply joystick deadband
     * (Negate turn since joystick X-axis is reversed)
     */
    throttle = ApplyDeadband(throttle, m_deadband);
    turn = ApplyDeadband(turn, m_deadband);

    double negInertia = turn - m_oldTurn;
    m_oldTurn = turn;

    float turnNonLinearity = m_settings.GetDouble("TURN_NON_LINEARITY");

    /* Apply a sine function that's scaled to make turning sensitivity feel better.
     * turnNonLinearity should never be zero, but can be close
     */
    turn = std::sin(M_PI / 2.0 * turnNonLinearity * turn) /
           std::sin(M_PI / 2.0 * turnNonLinearity);

    double angularPower = 0.f;
    double linearPower = throttle;
    double leftPwm = linearPower, rightPwm = linearPower;

    // Negative inertia!
    double negInertiaScalar;
    if (turn * negInertia > 0) {
        negInertiaScalar = m_settings.GetDouble("INERTIA_DAMPEN");
    }
    else {
        if (fabs(turn) > 0.65) {
            negInertiaScalar = m_settings.GetDouble("INERTIA_HIGH_TURN");
        }
        else {
            negInertiaScalar = m_settings.GetDouble("INERTIA_LOW_TURN");
        }
    }

    m_negInertiaAccumulator += negInertia * negInertiaScalar; // adds negInertiaPower

    // Apply negative inertia
    turn += m_negInertiaAccumulator;
    if (m_negInertiaAccumulator > 1) {
        m_negInertiaAccumulator -= 1;
    }
    else if (m_negInertiaAccumulator < -1) {
        m_negInertiaAccumulator += 1;
    }
    else {
        m_negInertiaAccumulator = 0;
    }

    // QuickTurn!
    if (isQuickTurn) {
        if (fabs(linearPower) < 0.2) {
            double alpha = 0.1;
            m_quickStopAccumulator = (1 - alpha) * m_quickStopAccumulator +
                                     alpha * Limit(turn, 1.f) * 5;
        }

        angularPower = turn;
    }
    else {
        angularPower = fabs(throttle) * turn * m_sensitivity -
                       m_quickStopAccumulator;

        if (m_quickStopAccumulator > 1) {
            m_quickStopAccumulator -= 1;
        }
        else if (m_quickStopAccumulator < -1) {
            m_quickStopAccumulator += 1;
        }
        else {
            m_quickStopAccumulator = 0.0;
        }
    }

    // Adjust straight path for turn
    leftPwm += angularPower;
    rightPwm -= angularPower;

    // Limit PWM bounds to [-1..1]
    if (leftPwm > 1.0) {
        // If overpowered turning enabled
        if (isQuickTurn) {
            rightPwm -= (leftPwm - 1.f);
        }

        leftPwm = 1.0;
    }
    else if (rightPwm > 1.0) {
        // If overpowered turning enabled
        if (isQuickTurn) {
            leftPwm -= (rightPwm - 1.f);
        }

        rightPwm = 1.0;
    }
    else if (leftPwm < -1.0) {
        // If overpowered turning enabled
        if (isQuickTurn) {
            rightPwm += (-leftPwm - 1.f);
        }

        leftPwm = -1.0;
    }
    else if (rightPwm < -1.0) {
        // If overpowered turning enabled
        if (isQuickTurn) {
            leftPwm += (-rightPwm - 1.f);
        }

        rightPwm = -1.0;
    }
    m_leftGrbx.SetManual(leftPwm);
    m_rightGrbx.SetManual(rightPwm);
}

void DriveTrain::SetDeadband(float band) {
    m_deadband = band;
}

void DriveTrain::ReloadPID() {
    m_settings.Update();

    float p = 0.f;
    float i = 0.f;
    float d = 0.f;

    p = m_settings.GetDouble("PID_DRIVE_LEFT_P");
    i = m_settings.GetDouble("PID_DRIVE_LEFT_I");
    d = m_settings.GetDouble("PID_DRIVE_LEFT_D");
    m_leftGrbx.SetPID(p, i, d);

    p = m_settings.GetDouble("PID_DRIVE_RIGHT_P");
    i = m_settings.GetDouble("PID_DRIVE_RIGHT_I");
    d = m_settings.GetDouble("PID_DRIVE_RIGHT_D");
    m_rightGrbx.SetPID(p, i, d);
}

void DriveTrain::ResetEncoders() {
    m_leftGrbx.ResetEncoder();
    m_rightGrbx.ResetEncoder();
}

void DriveTrain::SetLeftSetpoint(double setpt) {
    m_leftGrbx.SetSetpoint(setpt);
}

void DriveTrain::SetRightSetpoint(double setpt) {
    m_rightGrbx.SetSetpoint(setpt);
}

void DriveTrain::SetLeftManual(float value) {
    m_leftGrbx.SetManual(value);
}

void DriveTrain::SetRightManual(float value) {
    m_rightGrbx.SetManual(value);
}

double DriveTrain::GetLeftDist() const {
    return m_leftGrbx.Get(Grbx::Position);
}

double DriveTrain::GetRightDist() const {
    return m_rightGrbx.Get(Grbx::Position);
}

double DriveTrain::GetLeftRate() const {
    return m_leftGrbx.Get(Grbx::Speed);
}

double DriveTrain::GetRightRate() const {
    return m_rightGrbx.Get(Grbx::Speed);
}

double DriveTrain::GetLeftSetpoint() const {
    return m_leftGrbx.GetSetpoint();
}

double DriveTrain::GetRightSetpoint() const {
    return m_rightGrbx.GetSetpoint();
}

void DriveTrain::SetControlMode(CANTalon::ControlMode ctrlMode) {
    m_leftGrbx.setControlMode(ctrlMode);
    m_rightGrbx.setControlMode(ctrlMode);
}
