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

class Encoder;
class PIDController;

template <class T>
class GearBox : public GearBoxBase<T>, public PIDOutput {
public:
    GearBox(int shifterChan, int encA, int encB,
            int motor1, int motor2 = -1, int motor3 = -1);

    // Enables PID controller automatically and sets its setpoint
    void SetSetpoint(float setpoint);

    float GetSetpoint() const;

    // Disables PID controller and sets the motor speeds manually
    void SetManual(float value);

    // Returns current speed/position/voltage setting of motor controller(s)
    float Get(Grbx::PIDMode mode = Grbx::Raw) const;

    // Set P, I, and D terms for PID controller
    void SetPID(float p, float i, float d);

    // Set feed-forward term on PID controller
    void SetF(float f);

    // Calls Encoder::SetDistancePerPulse internally
    void SetDistancePerPulse(double distancePerPulse);

    // Resets encoder distance to 0
    void ResetEncoder();

    // Reverses gearbox drive direction
    void SetMotorReversed(bool reverse);

    // Returns motor reversal state of gearbox
    bool IsMotorReversed() const;

    // Reverses gearbox drive direction
    void SetEncoderReversed(bool reverse);

    // Shifts gearbox to another gear if available
    void SetGear(bool gear);

    // Gets current gearbox gear if available (false if not)
    bool GetGear() const;

    bool OnTarget() const;

    void ResetPID();

    // Determines whether encoder returns distance or rate from PIDGet()
    void SetPIDSourceType(PIDSourceType pidSource);

private:
    // Sets motor speed to 'output'
    void PIDWrite(float output);

    std::unique_ptr<PIDController> m_pid;
    std::shared_ptr<Encoder> m_encoder;
};

#include "GearBox.inl"

template <>
class GearBox<CANTalon> : public GearBoxBase<CANTalon> {
public:
    GearBox(int shifterChan,
            int motor1,
            int motor2 = -1,
            int motor3 = -1);

    // Enables PID controller automatically and sets its setpoint
    void SetSetpoint(float setpoint);

    float GetSetpoint() const;

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
    void setControlMode(CANTalon::ControlMode ctrlMode =
                            CANTalon::kPercentVbus);

    // Set soft limits of PID controller
    void setSoftPositionLimits(double forwardLimit, double backwardLimit);

    bool isFwdLimitSwitchClosed() const;
    bool isRevLimitSwitchClosed() const;

    void setIZone(unsigned int value);

    void setCloseLoopRampRate(double value);
    void setProfile(bool secondProfile);
};

#include "GearBoxCANTalon.inl"

#endif // GEARBOX_HPP
