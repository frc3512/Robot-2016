// =============================================================================
// Description: Represents a gear box with up to 3 motors and an encoder
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef GEARBOX_BASE_HPP
#define GEARBOX_BASE_HPP

#include <PIDOutput.h>
#include <PIDSource.h>
#include <Solenoid.h>
#include "../CANTalon.h"
#include "../MotionProfile/ProfileBase.hpp"
#include <vector>
#include <memory>

/* Notes:
 * The template type of this template class is only used for creating the right
 * type of motor controllers for the gearbox.
 *
 * It is assumed that only one type of motor controller is used per gearbox.
 *
 * Up to three motors can be specified per gearbox, since drive train gearboxes
 * will use up to three and other gearboxes will use less.
 */

namespace Grbx {
enum PIDMode {
    Position,
    Speed,
    Raw // Returns voltage [0..1] when used
};
}

class GearBoxBase {
public:
    GearBoxBase(int shifterChan, int encA, int encB,
                int motor1, int motor2 = -1, int motor3 = -1);
    virtual ~GearBoxBase() = default;

    // Enables PID controller automatically and sets its setpoint
    virtual void SetSetpoint(PIDState setpoint) = 0;

    virtual PIDState GetSetpoint() const = 0;

    // Disables PID controller and sets the motor speeds manually
    virtual void SetManual(float value) = 0;

    // Returns current speed/position/voltage setting of motor controller(s)
    virtual float Get(Grbx::PIDMode mode = Grbx::Raw) const = 0;

    // Set P, I, and D terms for PID controller
    virtual void SetPID(float p, float i, float d) = 0;

    // Set velocity feed-forward term on PID controller
    virtual void SetV(float v) = 0;

    // Set acceleration feed-forward term on PID controller
    virtual void SetA(float a) = 0;

    // Calls Encoder::SetDistancePerPulse internally
    virtual void SetDistancePerPulse(double distancePerPulse) = 0;

    // Resets encoder distance to 0
    virtual void ResetEncoder() = 0;

    // Reverses gearbox drive direction
    void SetMotorReversed(bool reverse);

    // Returns motor reversal state of gearbox
    bool IsMotorReversed() const;

    // Reverses gearbox drive direction
    virtual void SetEncoderReversed(bool reverse) = 0;

    // Returns motor reversal state of gearbox
    bool IsEncoderReversed() const;

    // Shifts gearbox to another gear if available
    void SetGear(bool gear);

    // Gets current gearbox gear if available (false if not)
    bool GetGear() const;

    virtual bool OnTarget() const = 0;

    virtual void ResetPID() = 0;

protected:
    bool m_isEncoderReversed = false;

    // Conversion factor for setpoints with respect to encoder readings
    double m_distancePerPulse = 1.0;

    // Store the setpoint, because CANTalon won't give it to us
    float m_setpoint = 0.f;

    float m_feedforward = 0.f;

    std::unique_ptr<Solenoid> m_shifter;

    std::vector<std::unique_ptr<CANTalon>> m_motors;
};

#endif // GEARBOX_BASE_HPP
