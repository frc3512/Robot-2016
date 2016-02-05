// =============================================================================
// Description: Represents a gear box with up to 3 motors and an encoder
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef GEARBOX_HPP
#define GEARBOX_HPP

#include "GearBoxBase.hpp"
#include <PIDOutput.h>
#include <PIDSource.h>
#include <CANTalon.h>

class GearBox : public GearBoxBase {
public:
    GearBox(int shifterChan,
            int motor1,
            int motor2 = -1,
            int motor3 = -1);

    // Enables PID controller automatically and sets its setpoint
    void SetSetpoint(PIDState setpoint);

    PIDState GetSetpoint() const;

    // Disables PID controller and sets the motor speeds manually
    void SetManual(float value);

    // Returns current speed/position/voltage setting of motor controller(s)
    float Get(Grbx::PIDMode mode = Grbx::Raw) const;

    // Set P, I, and D terms for PID controller
    void SetPID(float p, float i, float d);

    // Set feed-forward term on PID controller
    void SetF(float f);

    void SetDistancePerPulse(double distancePerPulse);

    // Resets encoder distance to 0
    void ResetEncoder();

    // Reverses gearbox drive direction
    void SetEncoderReversed(bool reverse);

    bool OnTarget() const;

    void ResetPID();

    // Determines whether encoder returns distance or rate from PIDGet()
    void SetControlMode(CANTalon::ControlMode ctrlMode =
                            CANTalon::kPercentVbus);

    // Set soft limits of PID controller
    void SetSoftPositionLimits(double forwardLimit, double backwardLimit);

    bool IsFwdLimitSwitchClosed() const;
    bool IsRevLimitSwitchClosed() const;

    void SetIZone(unsigned int value);

    void SetCloseLoopRampRate(double value);
    void SetProfile(bool secondProfile);
};

#endif // GEARBOX_HPP
